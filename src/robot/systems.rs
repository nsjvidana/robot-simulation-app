use crate::prelude::Real as Real;
use crate::prelude::*;
use bevy::prelude::{Changed, GlobalTransform, ResMut};
use bevy::prelude::{Commands, Entity, Query, Transform, With, Without};
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::data::Index;
use bevy_rapier3d::rapier::dynamics::RigidBodyType;
use bevy_rapier3d::rapier::prelude::{ColliderHandle, ImpulseJointHandle, MultibodyJointHandle, SPATIAL_DIM};
use bevy_rapier3d::utils::iso_to_transform;
use rapier3d_urdf::{UrdfJointHandle, UrdfRobot, UrdfRobotHandles};
use std::ops::DerefMut;
use std::path::Path;
use bevy_rapier3d::na::Isometry3;

macro_rules! rapier_collider_to_components {
    ($coll:expr, $coll_handle:expr, $context_link:expr) => {
        (
            //ColliderShape
            bevy_rapier3d::prelude::Collider::from($coll.shared_shape().clone()),
            //ColliderMassProps
            ColliderMassProperties::MassProperties(MassProperties::from_rapier(
                $coll.mass_properties(),
            )),
            //ColliderPosition
            Transform::from_translation($coll.position_wrt_parent().unwrap().translation.into())
                .with_rotation($coll.position_wrt_parent().unwrap().rotation.into()),
            //ColliderMaterial
            Friction {
                coefficient: $coll.friction(),
                combine_rule: $coll.friction_combine_rule().into_bevy(),
            },
            Restitution {
                coefficient: $coll.restitution(),
                combine_rule: $coll.restitution_combine_rule().into_bevy(),
            },
            //ColliderFlags
            $coll.active_collision_types().into_bevy(),
            CollisionGroups {
                filters: $coll.collision_groups().filter.into_bevy(),
                memberships: $coll.collision_groups().memberships.into_bevy(),
            },
            SolverGroups {
                filters: $coll.solver_groups().filter.into_bevy(),
                memberships: $coll.solver_groups().memberships.into_bevy(),
            },
            $coll.active_hooks().into_bevy(),
            $coll.active_events().into_bevy(),
            RapierColliderHandle($coll_handle),
            $context_link,
        )
    };
}

macro_rules! rapier_rb_to_components {
    ($rb:expr, $rb_handle:expr, $context_link:expr) => {
        (
            RigidBody::from($rb.body_type()),
            Transform::from_translation($rb.position().translation.into())
                .with_rotation($rb.position().rotation.into()),
            Damping {
                linear_damping: $rb.linear_damping(),
                angular_damping: $rb.angular_damping(),
            },
            GravityScale($rb.gravity_scale()),
            Ccd {
                enabled: $rb.is_ccd_enabled(),
            },
            Sleeping {
                sleeping: $rb.activation().sleeping,
                normalized_linear_threshold: $rb.activation().normalized_linear_threshold,
                angular_threshold: $rb.activation().angular_threshold,
            },
            Dominance::group($rb.dominance_group()),
            AdditionalSolverIterations($rb.additional_solver_iterations()),
            RapierRigidBodyHandle($rb_handle),
            $context_link,
        )
    };
}

pub fn init_robots(
    mut commands: Commands,
    mut robot_q: Query<
        (
            Entity,
            &mut Robot,
            Option<&RapierContextEntityLink>,
            Option<&GlobalTransform>,
        ),
        Without<RobotHandle>,
    >,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    mut context_q: WriteRapierContext<()>,
    mut robot_set: ResMut<RobotSet>,
) -> Result<()> {
    //TODO: parallelize this?
    for (robot_entity, mut robot, ctx_link, transform) in robot_q.iter_mut() {
        let context_link = RapierContextEntityLink(
            ctx_link.map_or_else(|| default_context_q.single(), |link| link.0),
        );
        let (
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        ) = context_q.rapier_context.get_mut(context_link.0)
            .expect("Invalid rapier context entity!");
        let mut context = RapierContextMut {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        };
        let (robot_links, handles) = import_robot(
            &mut context,
            &context_link,
            &robot,
            robot_entity,
            transform,
            &mut commands,
        )?;

        let robot_idx = robot_set.robots.insert(RobotSerData {
            robot_entity,
            entities: RobotEntities {
                link_entities: robot_links,
            },
            transform: transform.map_or_else(|| GlobalTransform::default(), |v| *v),
        });
        commands
            .entity(robot_entity)
            .insert((
                RobotHandle(robot_idx),
                RapierRobotHandles(handles),
                RobotKinematics {
                    chain: k::Chain::from(robot.urdf.clone()),
                },
            ))
            .insert_if_new(Transform::default());
    }
    Ok(())
}

fn import_robot(
    rapier_context: &mut RapierContextMut,
    context_link: &RapierContextEntityLink,
    robot: &Robot,
    robot_entity: Entity,
    robot_transform: Option<&GlobalTransform>,
    commands: &mut Commands,
) -> Result<(Vec<RobotLink>, UrdfRobotHandles<Index>)> {
    let mesh_dir = robot.mesh_dir
        .as_ref()
        .map(|v| v.as_path())
        .or_else(|| robot.robot_file_path.parent())
        .unwrap_or_else(|| Path::new("./"));
    let mut rapier_robot = UrdfRobot::from_robot(
        &robot.urdf,
        robot.urdf_loader_options.clone(),
        mesh_dir
    );
    if let Some(robot_transform) = robot_transform {
        rapier_robot.append_transform(&Isometry3 {
            translation: robot_transform.translation().into(),
            rotation: robot_transform.rotation().into(),
        });
    }
    for transmission in robot.urdf.transmissions.iter() {
        for trans_joint in transmission.joints.iter() {
            let joint_idx = robot.urdf.joints
                .iter()
                .position(|v| v.name == trans_joint.name)
                .ok_or_else(|| Error::Urdf {
                    robot_name: robot.robot_file_path.display().to_string(),
                    error: format!(
                        "Import failed: transmission {0} has unknown joint {1}",
                        transmission.name,
                        trans_joint.name
                    )
                })?;
            let urdf_joint = &robot.urdf.joints[joint_idx];
            let rapier_joint = &mut rapier_robot.joints[joint_idx];
            let joint = &mut rapier_joint.joint;
            joint.motor_axes = !joint.locked_axes;
            for i in 0..SPATIAL_DIM {
                let curr_bit = 1 << i;
                if (joint.motor_axes.bits() & curr_bit) != 0 {
                    let motor = &mut joint.motors[i];
                    motor.max_force = urdf_joint.limit.effort as Real;
                    if let Some(ctrl) = &urdf_joint.safety_controller {
                        motor.target_pos = ctrl.k_position as Real;
                        motor.target_vel = ctrl.k_velocity as Real;
                    }
                    else {
                        motor.target_vel = urdf_joint.limit.velocity as Real;
                    }
                }
            }
        }
    }

    let RapierContextMut {
        joints,
        colliders,
        rigidbody_set: bodies,
        ..
    } = rapier_context;
    let RapierContextJoints {
        multibody_joints,
        impulse_joints,
        ..
    } = joints.deref_mut();

    let mut handles =
        match robot.robot_joint_type {
            RobotJointType::ImpulseJoints => {
                let impulse_handles = rapier_robot.clone().insert_using_impulse_joints(
                    &mut bodies.bodies,
                    &mut colliders.colliders,
                    impulse_joints,
                );
                let joints: Vec<_> = impulse_handles.joints
                    .iter()
                    .map(|j| UrdfJointHandle {
                        joint: j.joint.0,
                        link1: j.link1,
                        link2: j.link2,
                    })
                    .collect();
                UrdfRobotHandles {
                    links: impulse_handles.links,
                    joints,
                }
            },
            RobotJointType::MultibodyJoints(mb_options) => {
                let mb_handles = rapier_robot.clone().insert_using_multibody_joints(
                    &mut bodies.bodies,
                    &mut colliders.colliders,
                    multibody_joints,
                    mb_options
                );
                if mb_handles.joints.iter().any(|v| v.joint.is_none()) {
                    return Err(Error::FailedOperation("Import failed: joint loop detected. Multibodies don't support loops.".to_string()));
                }
                let joints: Vec<_> = mb_handles.joints
                    .iter()
                    .map(|j| {
                        UrdfJointHandle {
                            joint: j.joint.unwrap().0,
                            link1: j.link1,
                            link2: j.link2,
                        }
                    })
                    .collect();
                UrdfRobotHandles {
                    links: mb_handles.links,
                    joints,
                }
            }
        };
    let robot_links = create_rapier_robot_entities(
        &mut handles,
        rapier_context,
        context_link,
        &robot.robot_joint_type,
        robot_entity,
        commands
    );
    Ok((robot_links, handles))
}


/// Returns list of created rigid body entities.
fn create_rapier_robot_entities(
    handles: &mut UrdfRobotHandles<Index>,
    context: &mut RapierContextMut,
    context_link: &RapierContextEntityLink,
    robot_joint_type: &RobotJointType,
    robot_entity: Entity,
    commands: &mut Commands,
) -> Vec<RobotLink> {
    let RapierContextMut {
        rigidbody_set,
        colliders,
        joints,
        ..
    } = context;
    let crate::convert::RapierRigidBodySet {
        bodies,
        entity2body,
        ..
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierRigidBodySet>(&mut **rigidbody_set) };
    let crate::convert::RapierContextColliders {
        colliders,
        entity2collider
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierContextColliders>(&mut **colliders) };
    let crate::convert::RapierContextJoints {
        impulse_joints,
        multibody_joints,
        entity2impulse_joint,
        entity2multibody_joint,
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierContextJoints>(&mut **joints) };

    let mut link_entities = Vec::with_capacity(handles.links.len());
    //Spawning link & collider entities
    for link in handles.links.iter() {
        let rb = bodies.get_mut(link.body).unwrap();
        let mut rb_e_cmd = commands.spawn((
            iso_to_transform(rb.position()),
            ColliderMassProperties::MassProperties(MassProperties {
                mass: rb.mass(),
                local_center_of_mass: rb.mass_properties().local_mprops.local_com.coords.into(),
                principal_inertia: W(rb.mass_properties().local_mprops.principal_inertia()).into(),
                principal_inertia_local_frame: rb.mass_properties().local_mprops.principal_inertia_local_frame.into(),
            }),
            Velocity {
                angvel: W(*rb.angvel()).into(),
                linvel: W(*rb.linvel()).into(),
            },
            Damping {
                linear_damping: rb.linear_damping(),
                angular_damping: rb.angular_damping(),
            },
            Ccd { enabled: rb.is_ccd_enabled() },
            Sleeping {
                sleeping: rb.activation().sleeping,
                angular_threshold: rb.activation().angular_threshold,
                normalized_linear_threshold: rb.activation().normalized_linear_threshold
            },
            match rb.body_type() {
                RigidBodyType::Dynamic => {RigidBody::Dynamic}
                RigidBodyType::Fixed => {RigidBody::Fixed}
                RigidBodyType::KinematicVelocityBased => {RigidBody::KinematicPositionBased}
                RigidBodyType::KinematicPositionBased => {RigidBody::KinematicVelocityBased}
            },
            Dominance { groups: rb.dominance_group() },
            AdditionalSolverIterations(rb.additional_solver_iterations()),
            RapierRigidBodyHandle(link.body),
            *context_link,

            RobotPart(robot_entity),
        ));
        // Rigid body enable/disable
        if !rb.is_enabled() { rb_e_cmd.insert(RigidBodyDisabled); }

        // Adding colliders
        let mut collider_entities = Vec::with_capacity(link.colliders.len());
        let collider_components = |coll: &bevy_rapier3d::rapier::prelude::Collider, coll_handle: ColliderHandle| {
            (
                Collider::from(coll.shared_shape().clone()),
                RapierColliderHandle(coll_handle),
                *context_link,
            )
        };
        if let Some(first_coll_handle) = link.colliders.first().cloned() {
            let coll = colliders.get_mut(first_coll_handle).unwrap();
            rb_e_cmd.insert(collider_components(coll, first_coll_handle));
            coll.user_data = rb_e_cmd.id().to_bits() as u128;
            entity2collider.insert(rb_e_cmd.id(), first_coll_handle);
            collider_entities.push(rb_e_cmd.id());
        }
        let rb_e = rb_e_cmd.id();
        std::mem::drop(rb_e_cmd);
        for coll_handle in link.colliders.iter().cloned().skip(1) {
            let coll = colliders.get_mut(coll_handle).unwrap();
            let coll_e = commands.spawn(collider_components(coll, coll_handle))
                .id();
            coll.user_data = coll_e.to_bits() as u128;
            entity2collider.insert(coll_e, coll_handle);
            collider_entities.push(coll_e);
        }
        rb.user_data = rb_e.to_bits() as u128;
        entity2body.insert(rb_e, link.body);

        commands.entity(rb_e).insert(LinkEntity { colliders: collider_entities.clone() });
        link_entities.push(RobotLink {
            rigid_body: rb_e,
            colliders: collider_entities,
            initial_pos: iso_to_transform(rb.position()),
        });
    }

    // Adding joint components
    for UrdfJointHandle {
        joint: joint,
        link1,
        link2,
    } in handles.joints.iter()
    {
        let par_ent = bodies
            .get(*link1)
            .map(|rb| Entity::from_bits(rb.user_data as u64))
            .unwrap();
        let child_ent = bodies
            .get(*link2)
            .map(|rb| Entity::from_bits(rb.user_data as u64))
            .unwrap();

        let mut child_ent_cmd = commands.entity(child_ent);
        match robot_joint_type {
            RobotJointType::ImpulseJoints => {
                let handle = ImpulseJointHandle(*joint);
                let impulse_joint = impulse_joints.get(handle).unwrap();
                child_ent_cmd.insert((
                    RapierImpulseJointHandle(handle),
                    ImpulseJoint::new(
                        par_ent,
                        TypedJoint::GenericJoint(GenericJoint {
                            raw: impulse_joint.data,
                        }),
                    ),
                ));
                entity2impulse_joint
                    .insert(child_ent_cmd.id(), handle);
            }
            RobotJointType::MultibodyJoints(..) => {
                let handle = MultibodyJointHandle(*joint);
                let (mb, link_id) = multibody_joints.get(handle).unwrap();
                child_ent_cmd.insert((
                    RapierMultibodyJointHandle(handle),
                    MultibodyJoint::new(
                        par_ent,
                        TypedJoint::GenericJoint(GenericJoint {
                            raw: mb.link(link_id).unwrap().joint.data,
                        }),
                    ),
                ));
                entity2multibody_joint
                    .insert(child_ent_cmd.id(), handle);
            }
        }
    }

    link_entities
}

pub fn sync_robot_changes(
    mut changed_robot_transforms_q: Query<
        (&GlobalTransform, &RobotHandle),
        (With<Robot>, Changed<GlobalTransform>),
    >,
    mut rb_transform_q: Query<
        &mut GlobalTransform,
        (With<RigidBody>, With<RobotPart>, Without<Robot>),
    >,
    mut robot_set: ResMut<RobotSet>,
) {
    for (new_transform, robot_handle) in changed_robot_transforms_q.iter() {
        let RobotSerData {
            entities: robot_entities,
            transform: ser_transform,
            ..
        } = robot_set.robots.get_mut(robot_handle.0).unwrap();
        for RobotLink {
            rigid_body,
            initial_pos,
            ..
        } in robot_entities.link_entities.iter()
        {
            let rb_transform = &mut *rb_transform_q.get_mut(*rigid_body).unwrap();
            *rb_transform = new_transform.mul_transform(*initial_pos);
        }
        *ser_transform = *new_transform;
    }
}
