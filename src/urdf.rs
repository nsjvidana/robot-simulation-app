use crate::convert::IntoBevy;
use bevy::{prelude::{Commands, Component, Entity, Parent, Query, Transform, With, Without}, hierarchy::BuildChildren};
use bevy::ptr::UnsafeCellDeref;
use bevy_rapier3d::geometry::SolverGroups;
use bevy_rapier3d::prelude::{AdditionalMassProperties, AdditionalSolverIterations, Ccd, Collider, ColliderDisabled, ColliderMassProperties, CollisionGroups, Damping, DefaultRapierContext, Dominance, Friction, GravityScale, MassProperties, RapierColliderHandle, RapierContext, RapierContextEntityLink, RapierRigidBodyHandle, Restitution, RigidBody, RigidBodyDisabled, Sensor, Sleeping};
use bevy_rapier3d::rapier::prelude::RigidBodyAdditionalMassProps;
use rapier3d_urdf::{UrdfJoint, UrdfLink, UrdfLoaderOptions, UrdfMultibodyOptions};

#[derive(Component)]
pub struct UrdfRobot {
    /// The bodies and colliders loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::links`]
    pub links: Vec<UrdfLink>,
    /// The joints loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::joints`].
    pub joints: Vec<UrdfJoint>,
    pub robot_joint_type: RobotJointType,
    pub import_options: UrdfLoaderOptions
}

impl UrdfRobot {
    pub fn with_impulse_joints(mut self) -> Self {
        self.robot_joint_type = RobotJointType::ImpulseJoints;
        self
    }

    pub fn with_multibody_joints(mut self, joint_options: UrdfMultibodyOptions) -> Self {
        self.robot_joint_type = RobotJointType::MultibodyJoints(joint_options);
        self
    }
}

impl From<rapier3d_urdf::UrdfRobot> for UrdfRobot {
    fn from(value: rapier3d_urdf::UrdfRobot) -> Self {
        Self {
            links: value.links,
            joints: value.joints,
            robot_joint_type: RobotJointType::ImpulseJoints,
            import_options: UrdfLoaderOptions::default(),
        }
    }
}

pub enum RobotJointType {
    ImpulseJoints,
    MultibodyJoints(UrdfMultibodyOptions),
}

//TODO: Add system that deals with this struct
#[derive(Component)]
pub struct RobotEntities;

pub fn init_robots(
    mut commands: Commands,
    robot_q: Query<(Entity, &UrdfRobot, Option<&RapierContextEntityLink>), Without<RobotEntities>>,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    mut context_q: Query<&mut RapierContext>,
) {
    //TODO: parallelize this?
    for (entity, robot, ctx_link) in robot_q.iter() {
        let context_link = RapierContextEntityLink(
            ctx_link.map_or_else(
                || default_context_q.single(),
                |link| link.0
            )
        );

        let unsafe_ctx = std::cell::UnsafeCell::new(
            context_q.get_mut(context_link.0).unwrap()
        );
        let handles = unsafe {
            let robo = rapier3d_urdf::UrdfRobot {
                links: robot.links.clone(),
                joints: robot.joints.clone(),
            };
            robo.insert_using_impulse_joints(
                &mut unsafe_ctx.deref_mut().bodies,
                &mut unsafe_ctx.deref_mut().colliders,
                &mut unsafe_ctx.deref_mut().impulse_joints,
            )
        };
        let context = unsafe { unsafe_ctx.deref_mut() };

        //Spawning link & collider entities
        for link in handles.links.iter() {
            let rb = context.bodies.get_mut(link.body).unwrap();
            let mut rb_ent = commands.spawn((
                RigidBody::from(rb.body_type()),
                Transform::from_translation(rb.position().translation.into())
                    .with_rotation(rb.position().rotation.into()),
                Damping {
                    linear_damping: rb.linear_damping(),
                    angular_damping: rb.angular_damping()
                },
                GravityScale(rb.gravity_scale()),
                Ccd { enabled: rb.is_ccd_enabled() },
                Sleeping {
                    sleeping: rb.activation().sleeping,
                    normalized_linear_threshold: rb.activation().normalized_linear_threshold,
                    angular_threshold: rb.activation().angular_threshold,
                },
                Dominance::group(rb.dominance_group()),
                AdditionalSolverIterations(rb.additional_solver_iterations()),

                RapierRigidBodyHandle(link.body),
                context_link
            ));
            if !rb.is_enabled() { rb_ent.insert(RigidBodyDisabled); }
            if let Some(additional_mprops) = &rb.mass_properties().additional_local_mprops {
                match *(*additional_mprops) {
                    RigidBodyAdditionalMassProps::MassProps(mprops) => {
                        rb_ent.insert(
                            AdditionalMassProperties::MassProperties(MassProperties::from_rapier(mprops))
                        );
                    },
                    RigidBodyAdditionalMassProps::Mass(mass) => {
                        rb_ent.insert(AdditionalMassProperties::Mass(mass));
                    },
                }
            }
            rb.user_data = rb_ent.id().to_bits() as u128;
            let rb_ent = rb_ent.id();

            //spawning colliders
            for coll_handle in link.colliders.iter() {
                let coll = context.colliders.get_mut(*coll_handle).unwrap();
                let mut coll_ent = commands.spawn((
                    //ColliderShape
                    bevy_rapier3d::prelude::Collider::from(coll.shared_shape().clone()),
                    //ColliderMassProps
                    ColliderMassProperties::MassProperties(
                        MassProperties::from_rapier(coll.mass_properties())
                    ),
                    //ColliderPosition
                    Transform::from_translation(coll.position().translation.into())
                        .with_rotation(coll.position().rotation.into()),
                    //ColliderMaterial
                    Friction {
                        coefficient: coll.friction(),
                        combine_rule: coll.friction_combine_rule().into_bevy()
                    },
                    Restitution {
                        coefficient: coll.restitution(),
                        combine_rule: coll.restitution_combine_rule().into_bevy()
                    },
                    //ColliderFlags
                    coll.active_collision_types().into_bevy(),
                    CollisionGroups {
                        filters: coll.collision_groups().filter.into_bevy(),
                        memberships: coll.collision_groups().memberships.into_bevy(),
                    },
                    SolverGroups {
                        filters: coll.solver_groups().filter.into_bevy(),
                        memberships: coll.solver_groups().memberships.into_bevy(),
                    },
                    coll.active_hooks().into_bevy(),
                    coll.active_events().into_bevy(),

                    RapierColliderHandle(*coll_handle),
                    context_link,
                ));
                //ColliderType
                if coll.is_sensor() { coll_ent.insert(Sensor); }
                //ColliderFlags::enabled
                if !coll.is_enabled() { coll_ent.insert(ColliderDisabled); }
                coll_ent.set_parent(rb_ent);
                coll.user_data = coll_ent.id().to_bits() as u128;
            }
        }
        commands.entity(entity).insert(RobotEntities);
        //TODO: add joints
    }
}
