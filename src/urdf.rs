use crate::convert::IntoBevy;
use bevy::ptr::UnsafeCellDeref;
use bevy::{hierarchy::BuildChildren, prelude::{Commands, Component, Entity, Query, Transform, With, Without}};
use bevy_rapier3d::geometry::SolverGroups;
use bevy_rapier3d::prelude::{AdditionalMassProperties, AdditionalSolverIterations, Ccd, ColliderDisabled, ColliderMassProperties, CollisionGroups, Damping, DefaultRapierContext, Dominance, Friction, GravityScale, MassProperties, RapierColliderHandle, RapierContext, RapierContextEntityLink, RapierRigidBodyHandle, Restitution, RigidBody, RigidBodyDisabled, Sensor, Sleeping};
use bevy_rapier3d::rapier::data::Index;
use bevy_rapier3d::rapier::prelude::RigidBodyAdditionalMassProps;
use rapier3d_urdf::{UrdfJointHandle, UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobotHandles};
use std::cell::UnsafeCell;

#[derive(Component)]
pub struct UrdfRobot {
    pub rapier_urdf_robot: rapier3d_urdf::UrdfRobot,
    pub robot_joint_type: RobotJointType,
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
            rapier_urdf_robot: value,
            robot_joint_type: RobotJointType::ImpulseJoints,
        }
    }
}

pub enum RobotJointType {
    ImpulseJoints,
    MultibodyJoints(UrdfMultibodyOptions),
}

impl Default for RobotJointType {
    fn default() -> Self {
        Self::ImpulseJoints
    }
}

//TODO: Add system that deals with this struct
#[derive(Component)]
pub struct RobotEntities {
    link_entities: Vec<RobotLink>,
}

#[derive(Component)]
pub struct LinkEntity {
    pub colliders: Vec<Entity>,
}

#[derive(Component)]
pub struct RobotPart(pub Entity);

pub struct RobotLink {
    rigid_body: Entity,
    colliders: Vec<Entity>,
}

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
            Transform::from_translation($coll.position().translation.into())
                .with_rotation($coll.position().rotation.into()),
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

            RapierColliderHandle(*$coll_handle),
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
    robot_q: Query<(Entity, &UrdfRobot, Option<&RapierContextEntityLink>), Without<RobotEntities>>,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    mut context_q: Query<&mut RapierContext>,
) {
    //TODO: parallelize this?
    for (robot_entity, robot, ctx_link) in robot_q.iter() {
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
                    let handles = robot.rapier_urdf_robot
                        .clone()
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
                    let handles = robot.rapier_urdf_robot
                        .clone()
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
        let context = unsafe { unsafe_ctx.deref_mut() };

        //Spawning link & collider entities
        let mut robot_links = Vec::with_capacity(handles.links.len());
        for link in handles.links.iter() {
            let rb = context.bodies.get_mut(link.body).unwrap();
            let mut rb_ent = commands.spawn(
                rapier_rb_to_components!(rb, link.body, context_link)
            );
            rb_ent.insert(RobotPart(robot_entity));
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
            let mut collider_entities = Vec::with_capacity(link.colliders.len());
            for coll_handle in link.colliders.iter() {
                let coll = context.colliders.get_mut(*coll_handle).unwrap();
                let mut coll_ent = commands.spawn(
                    rapier_collider_to_components!(coll, coll_handle, context_link)
                );
                //ColliderType
                if coll.is_sensor() { coll_ent.insert(Sensor); }
                //ColliderFlags::enabled
                if !coll.is_enabled() { coll_ent.insert(ColliderDisabled); }
                coll_ent.set_parent(rb_ent);
                coll.user_data = coll_ent.id().to_bits() as u128;
                collider_entities.push(coll_ent.id());
            }

            commands.entity(rb_ent)
                .insert(LinkEntity { colliders: collider_entities.clone() })
                .insert(RobotPart(robot_entity));

            robot_links.push(RobotLink {
                rigid_body: rb_ent,
                colliders: collider_entities
            });
        }
        commands.entity(robot_entity).insert(RobotEntities {
            link_entities: robot_links,
        });

        //TODO: Add MultibodyJoint components
        for UrdfJointHandle {joint, link1, link2} in handles.joints.iter() {
            if let RobotJointType::MultibodyJoints(..) =  robot.robot_joint_type {
                if joint.is_none() { continue; }
            }

            // context.entity2body()
        }
    }
}
