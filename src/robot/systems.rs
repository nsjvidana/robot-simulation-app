use crate::convert::IntoBevy;
use bevy::prelude::{Changed, GlobalTransform, Mut, Res, ResMut};
use bevy::ptr::UnsafeCellDeref;
use bevy::{hierarchy::BuildChildren, prelude::{Commands, Component, Entity, Query, Transform, With, Without}};
use bevy_rapier3d::geometry::SolverGroups;
use bevy_rapier3d::prelude::{AdditionalMassProperties, AdditionalSolverIterations, Ccd, ColliderDisabled, ColliderMassProperties, CollisionGroups, Damping, DefaultRapierContext, Dominance, Friction, GenericJoint, GravityScale, ImpulseJoint, MassProperties, MultibodyJoint, RapierColliderHandle, RapierContext, RapierContextEntityLink, RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle, Restitution, RigidBody, RigidBodyDisabled, Sensor, Sleeping, TypedJoint};
use bevy_rapier3d::rapier::data::Index;
use bevy_rapier3d::rapier::prelude::{ImpulseJointHandle, MultibodyJointHandle, RigidBodyAdditionalMassProps};
use rapier3d_urdf::{UrdfJointHandle, UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};
use std::cell::UnsafeCell;
use std::ops::DerefMut;
use bevy_rapier3d::utils::iso_to_transform;
use crate::robot::*;

macro_rules! rapier_collider_to_components {
    ($coll:expr, $coll_handle:expr, $context_link:expr) => {
        (
            //ColliderShape
            bevy_rapier3d::prelude::Collider::from($coll.shared_shape().clone()),
            //ColliderMassProps
            ColliderMassProperties::MassProperties(
                MassProperties::from_rapier($coll.mass_properties())
            ),
            //ColliderPosition
            Transform::from_translation($coll.position_wrt_parent().unwrap().translation.into())
                .with_rotation($coll.position_wrt_parent().unwrap().rotation.into()),
            //ColliderMaterial
            Friction {
                coefficient: $coll.friction(),
                combine_rule: $coll.friction_combine_rule().into_bevy()
            },
            Restitution {
                coefficient: $coll.restitution(),
                combine_rule: $coll.restitution_combine_rule().into_bevy()
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
                angular_damping: $rb.angular_damping()
            },
            GravityScale($rb.gravity_scale()),
            Ccd { enabled: $rb.is_ccd_enabled() },
            Sleeping {
                sleeping: $rb.activation().sleeping,
                normalized_linear_threshold: $rb.activation().normalized_linear_threshold,
                angular_threshold: $rb.activation().angular_threshold,
            },
            Dominance::group($rb.dominance_group()),
            AdditionalSolverIterations($rb.additional_solver_iterations()),

            RapierRigidBodyHandle($rb_handle),
            $context_link
        )
    };
}

pub fn init_robots(
    mut commands: Commands,
    mut robot_q: Query<
        (Entity, &mut Robot, Option<&RapierContextEntityLink>, Option<&GlobalTransform>),
        Without<RobotHandle>
    >,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    mut context_q: Query<&mut RapierContext>,
    mut robot_set: ResMut<RobotSet>,
) {
    //TODO: parallelize this?
    for (robot_entity, mut robot, ctx_link, transform) in robot_q.iter_mut() {
        let context_link = RapierContextEntityLink(
            ctx_link.map_or_else(
                || default_context_q.single(),
                |link| link.0
            )
        );
        let unsafe_ctx = UnsafeCell::new(context_q.get_mut(context_link.0).unwrap());

        // Use rapier data Index to make it easier to handle multibody and impulse joint handles together
        // JointHandle is an option to account for multibody joint handles
        // SAFETY: since the rapier context is accessed mutably, bevy restricts any other threads from
        //         accessing the context, so this should be safe
        let handles: UrdfRobotHandles<Option<Index>> = unsafe {
            match robot.robot_joint_type {
                RobotJointType::ImpulseJoints => {
                    let handles = robot.rapier_urdf_robot.take().unwrap()
                        .insert_using_impulse_joints(
                            &mut unsafe_ctx.deref_mut().bodies,
                            &mut unsafe_ctx.deref_mut().colliders,
                            &mut unsafe_ctx.deref_mut().impulse_joints,
                        );
                    let joints: Vec<_> = handles.joints.iter()
                        .map(|j| UrdfJointHandle {
                            link1: j.link1,
                            link2: j.link2,
                            joint: Some(j.joint.0)
                        })
                        .collect();
                    UrdfRobotHandles {
                        links: handles.links,
                        joints,
                    }
                },
                RobotJointType::MultibodyJoints(options) => {
                    let handles = robot.rapier_urdf_robot.take().unwrap()
                        .insert_using_multibody_joints(
                            &mut unsafe_ctx.deref_mut().bodies,
                            &mut unsafe_ctx.deref_mut().colliders,
                            &mut unsafe_ctx.deref_mut().multibody_joints,
                            options
                        );
                    let joints: Vec<_> = handles.joints.iter()
                        .map(|j| UrdfJointHandle {
                            link1: j.link1,
                            link2: j.link2,
                            joint: j.joint.map(|j| j.0)
                        })
                        .collect();
                    UrdfRobotHandles {
                        links: handles.links,
                        joints,
                    }
                },
            }
        };
        // SAFETY: This is ok since crate::convert::RapierContext is implemented exactly the same as
        //         the bevy_rapier RapierContext
        let mut context: Mut<crate::convert::RapierContext> = unsafe {
            std::mem::transmute(unsafe_ctx.into_inner())
        };

        //Spawning link & collider entities
        let mut robot_links = Vec::with_capacity(handles.links.len());
        for link in handles.links.iter() {
            let rb = context.bodies.get_mut(link.body).unwrap();
            let mut rb_ent = commands.spawn((
                rapier_rb_to_components!(rb, link.body, context_link),
                RobotPart(robot_entity)
            ));
            rb_ent.insert_if(RigidBodyDisabled, || !rb.is_enabled());
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
            let rb_ent = rb_ent.id();
            rb.user_data = rb_ent.to_bits() as u128;
            let initial_rb_pos = iso_to_transform(rb.position());
            context.entity2body.insert(rb_ent, link.body);

            //spawning colliders
            let mut collider_entities = Vec::with_capacity(link.colliders.len());
            for coll_handle in link.colliders.iter().copied() {
                let coll = context.colliders.get_mut(coll_handle).unwrap();
                let mut coll_ent = commands.spawn((
                    rapier_collider_to_components!(coll, coll_handle, context_link),
                    RobotPart(robot_entity),
                ));
                coll_ent
                    .insert_if(Sensor, || coll.is_sensor())//ColliderType
                    .insert_if(ColliderDisabled, || !coll.is_enabled())//ColliderFlags::enabled
                    .set_parent(rb_ent);
                coll.user_data = coll_ent.id().to_bits() as u128;
                collider_entities.push(coll_ent.id());
                context.entity2collider.insert(rb_ent, coll_handle);
            }

            commands.entity(rb_ent)
                .insert((
                    LinkEntity { colliders: collider_entities.clone() },
                    RobotPart(robot_entity)
                ));

            robot_links.push(RobotLink {
                rigid_body: rb_ent,
                colliders: collider_entities,
                initial_pos: initial_rb_pos,
            });
        }

        //Adding joint components
        for UrdfJointHandle {joint, link1, link2} in handles.joints.iter() {
            if joint.is_none() { continue; }

            let par_idx = handles
                .links
                .iter()
                .enumerate()
                .find(|(_, link)| link.body == *link1)
                .unwrap()
                .0;
            let child_idx = handles
                .links
                .iter()
                .enumerate()
                .find(|(_, link)| link.body == *link2)
                .unwrap()
                .0;
            let par_ent = robot_links[par_idx].rigid_body;
            let mut child_ent_cmd = commands.entity(robot_links[child_idx].rigid_body);
            match robot.robot_joint_type {
                RobotJointType::ImpulseJoints => {
                    let handle = ImpulseJointHandle(joint.unwrap());
                    let imp_j = context.impulse_joints.get(handle).unwrap();
                    child_ent_cmd.insert((
                        RapierImpulseJointHandle(handle),
                        ImpulseJoint::new(par_ent, TypedJoint::GenericJoint(GenericJoint {
                            raw: imp_j.data
                        }))
                    ));

                    context.entity2impulse_joint.insert(child_ent_cmd.id(), handle);
                },
                RobotJointType::MultibodyJoints(..) => {
                    let handle = MultibodyJointHandle(joint.unwrap());
                    let (mb, link_id) = context.multibody_joints.get(handle).unwrap();
                    child_ent_cmd.insert((
                        RapierMultibodyJointHandle(handle),
                        MultibodyJoint::new(par_ent, TypedJoint::GenericJoint(GenericJoint {
                            raw: mb.link(link_id).unwrap().joint.data
                        })),
                    ));

                    context.entity2multibody_joint.insert(child_ent_cmd.id(), handle);
                },
            }
        }

        let robot_idx = robot_set.robots.insert(RobotSerData {
            robot_entity,
            entities: RobotEntities {
                link_entities: robot_links,
            },
            transform: transform.map_or_else(|| GlobalTransform::default(), |v| *v),
        });
        commands.entity(robot_entity)
            .insert(RobotHandle(robot_idx))
            .insert_if_new(Transform::default());
    }
}

pub fn sync_robot_changes(
    mut changed_robot_transforms_q: Query<
        (&GlobalTransform, &RobotHandle),
        (With<Robot>, Changed<GlobalTransform>)
    >,
    mut rb_transform_q: Query<
        &mut GlobalTransform,
        (With<RigidBody>, With<RobotPart>,  Without<Robot>)
    >,
    mut robot_set: ResMut<RobotSet>
) {
    for (new_transform, robot_handle) in changed_robot_transforms_q.iter() {
        let robot_ser_data =  robot_set.robots.get_mut (robot_handle.0).unwrap();
        for RobotLink {rigid_body, initial_pos, ..} in robot_ser_data.entities.link_entities.iter() {
            let mut rb_transform = rb_transform_q.get_mut(*rigid_body).unwrap();
            let _ = std::mem::replace(rb_transform.deref_mut(), new_transform.mul_transform(*initial_pos));
        }
        robot_ser_data.transform = *new_transform;
    }
}
