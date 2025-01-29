use crate::convert::IntoBevy;
use bevy::prelude::{Commands, Component, Entity, Query, Transform, With, Without};
use bevy_rapier3d::geometry::SolverGroups;
use bevy_rapier3d::prelude::{AdditionalMassProperties, AdditionalSolverIterations, Ccd, ColliderDisabled, ColliderMassProperties, CollisionGroups, Damping, DefaultRapierContext, Dominance, Friction, GravityScale, MassProperties, RapierContextEntityLink, Restitution, RigidBody, RigidBodyDisabled, Sensor, Sleeping};
use bevy_rapier3d::rapier::prelude::RigidBodyAdditionalMassProps;
use rapier3d_urdf::{UrdfJoint, UrdfLink};

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
    pub robot_joint_type: RobotJointType
}

impl UrdfRobot {
    pub fn with_impulse_joints(mut self) -> Self {
        self.robot_joint_type = RobotJointType::ImpulseJoints;
        self
    }

    pub fn with_multibody_joints(mut self) -> Self {
        self.robot_joint_type = RobotJointType::MultibodyJoints;
        self
    }
}

impl From<rapier3d_urdf::UrdfRobot> for UrdfRobot {
    fn from(value: rapier3d_urdf::UrdfRobot) -> Self {
        Self {
            links: value.links,
            joints: value.joints,
            robot_joint_type: RobotJointType::ImpulseJoints
        }
    }
}

pub enum RobotJointType {
    ImpulseJoints,
    MultibodyJoints,
}

//TODO: Add system that deals with this struct
#[derive(Component)]
pub struct RobotEntities;

pub fn init_robots(
    mut commands: Commands,
    robot_q: Query<(Entity, &UrdfRobot, Option<&RapierContextEntityLink>), Without<RobotEntities>>,
    default_context_q: Query<Entity, With<DefaultRapierContext>>
) {
    //TODO: parallelize this?
    for (entity, robot, ctx_link) in robot_q.iter() {
        let context_link = RapierContextEntityLink(
            ctx_link.map_or_else(
                || default_context_q.single(),
                |link| link.0
            )
        );
        for UrdfLink { body, colliders } in robot.links.iter() {
            //set rigid body data
            let mut link_ent = commands.spawn((
                RigidBody::from(body.body_type()),
                Transform::from_translation(body.position().translation.into())
                    .with_rotation(body.position().rotation.into()),
                Damping {
                    linear_damping: body.linear_damping(),
                    angular_damping: body.angular_damping()
                },
                GravityScale(body.gravity_scale()),
                Ccd { enabled: body.is_ccd_enabled() },
                Sleeping {
                    sleeping: body.activation().sleeping,
                    normalized_linear_threshold: body.activation().normalized_linear_threshold,
                    angular_threshold: body.activation().angular_threshold,
                },
                Dominance::group(body.dominance_group()),
                AdditionalSolverIterations(body.additional_solver_iterations()),

                context_link
            ));
            if !body.is_enabled() { link_ent.insert(RigidBodyDisabled); }
            if let Some(additional_mprops) = &body.mass_properties().additional_local_mprops {
                match *(*additional_mprops) {
                    RigidBodyAdditionalMassProps::MassProps(mprops) => {
                        link_ent.insert(
                            AdditionalMassProperties::MassProperties(MassProperties::from_rapier(mprops))
                        );
                    },
                    RigidBodyAdditionalMassProps::Mass(mass) => {
                        link_ent.insert(AdditionalMassProperties::Mass(mass));
                    },
                }
            }

            //Adding link colliders
            for collider in colliders.iter() {
                let mut coll_ent = commands.spawn((
                    //ColliderShape
                    bevy_rapier3d::prelude::Collider::from(collider.shared_shape().clone()),
                    //ColliderMassProps
                    ColliderMassProperties::MassProperties(
                        MassProperties::from_rapier(collider.mass_properties())
                    ),
                    //ColliderPosition
                    Transform::from_translation(collider.position().translation.into())
                        .with_rotation(collider.position().rotation.into()),
                    //ColliderMaterial
                    Friction {
                        coefficient: collider.friction(),
                        combine_rule: collider.friction_combine_rule().into_bevy()
                    },
                    Restitution {
                        coefficient: collider.restitution(),
                        combine_rule: collider.restitution_combine_rule().into_bevy()
                    },
                    //ColliderFlags
                    collider.active_collision_types().into_bevy(),
                    CollisionGroups {
                        filters: collider.collision_groups().filter.into_bevy(),
                        memberships: collider.collision_groups().memberships.into_bevy(),
                    },
                    SolverGroups {
                        filters: collider.solver_groups().filter.into_bevy(),
                        memberships: collider.solver_groups().memberships.into_bevy(),
                    },
                    collider.active_hooks().into_bevy(),
                    collider.active_events().into_bevy(),

                    context_link
                ));
                //ColliderType
                if collider.is_sensor() { coll_ent.insert(Sensor); }
                //ColliderFlags::enabled
                if !collider.is_enabled() { coll_ent.insert(ColliderDisabled); }
            }

            //TODO: add joints
        }

        commands.entity(entity).insert(RobotEntities);
    }
}
