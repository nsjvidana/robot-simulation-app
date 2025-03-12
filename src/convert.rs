use bevy::prelude::{Component, Entity, GlobalTransform};
use bevy_rapier3d::geometry::ActiveHooks;
use bevy_rapier3d::prelude::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule, CollisionEvent, ContactForceEvent, Group};
use bevy_rapier3d::rapier;
use bevy_rapier3d::rapier::prelude::{CCDSolver, ColliderHandle, ColliderSet, DefaultBroadPhase, EventHandler, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointHandle, MultibodyJointSet, NarrowPhase, PhysicsPipeline, QueryPipeline, RigidBodyHandle, RigidBodySet};
use bevy_rapier3d::na::{ArrayStorage, Matrix3, Vector, Vector3};
use std::collections::HashMap;

pub trait IntoBevy {
    type Target;

    fn into_bevy(self) -> Self::Target;
}

impl IntoBevy for rapier::prelude::CoefficientCombineRule {
    type Target = CoefficientCombineRule;
    fn into_bevy(self) -> Self::Target {
        match self {
            rapier::prelude::CoefficientCombineRule::Average =>
                CoefficientCombineRule::Average,
            rapier::prelude::CoefficientCombineRule::Min =>
                CoefficientCombineRule::Min,
            rapier::prelude::CoefficientCombineRule::Max =>
                CoefficientCombineRule::Max,
            rapier::prelude::CoefficientCombineRule::Multiply =>
                CoefficientCombineRule::Multiply,
        }
    }
}

impl IntoBevy for rapier::prelude::ActiveCollisionTypes {
    type Target = ActiveCollisionTypes;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::Group {
    type Target = Group;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::ActiveHooks {
    type Target = ActiveHooks;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::ActiveEvents {
    type Target = ActiveEvents;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

pub trait IntoXurdf {
    type Target;

    fn into_xurdf(self) -> Self::Target;
}

impl IntoXurdf for urdf_rs::Pose {
    type Target = xurdf::Pose;

    fn into_xurdf(self) -> Self::Target {
        xurdf::Pose {
            rpy: Vector3::from_data(ArrayStorage([self.rpy])),
            xyz: Vector3::from_data(ArrayStorage([self.xyz])),
        }
    }
}

impl IntoXurdf for urdf_rs::Geometry {
    type Target = xurdf::Geometry;

    fn into_xurdf(self) -> Self::Target {
        use xurdf::Geometry;
        match self {
            Self::Box { size } => Geometry::Box {
                size: Vector::from_data(ArrayStorage([size])),
            },
            Self::Cylinder { radius, length } => Geometry::Cylinder {
                radius,
                length
            },
            Self::Sphere { radius } => Geometry::Sphere {
                radius
            },
            Self::Mesh { filename, scale} => Geometry::Mesh {
                filename,
                scale: scale.map(|s| Vector::from_data(ArrayStorage([s])))
            },
            _ => unimplemented!(),
        }
    }
}

impl IntoXurdf for urdf_rs::Inertial {
    type Target = xurdf::Inertial;

    fn into_xurdf(self) -> Self::Target {
        let inertia = &self.inertia;
        xurdf::Inertial {
            origin: self.origin.into_xurdf(),
            inertia: Matrix3::from_data(ArrayStorage([
                [inertia.ixx, inertia.ixy, inertia.ixz],
                [0.         , inertia.iyy, inertia.iyz],
                [0.         , 0.         , inertia.izz]
            ])),
            mass: self.mass.value
        }
    }
}

#[allow(unused_variables, dead_code)]
#[derive(Component)]
pub struct RapierContext {
    /// The island manager, which detects what object is sleeping
    /// (not moving much) to reduce computations.
    pub islands: IslandManager,
    /// The broad-phase, which detects potential contact pairs.
    pub broad_phase: DefaultBroadPhase,
    /// The narrow-phase, which computes contact points, tests intersections,
    /// and maintain the contact and intersection graphs.
    pub narrow_phase: NarrowPhase,
    /// The set of rigid-bodies part of the simulation.
    pub bodies: RigidBodySet,
    /// The set of colliders part of the simulation.
    pub colliders: ColliderSet,
    /// The set of impulse joints part of the simulation.
    pub impulse_joints: ImpulseJointSet,
    /// The set of multibody joints part of the simulation.
    pub multibody_joints: MultibodyJointSet,
    /// The solver, which handles Continuous Collision Detection (CCD).
    pub ccd_solver: CCDSolver,
    /// The physics pipeline, which advance the simulation step by step.
    pub pipeline: PhysicsPipeline,
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipeline,
    /// The integration parameters, controlling various low-level coefficient of the simulation.
    pub integration_parameters: IntegrationParameters,
    pub(crate) event_handler: Option<Box<dyn EventHandler>>,
    // For transform change detection.
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
    // NOTE: these maps are needed to handle despawning.
    pub(crate) entity2body: HashMap<Entity, RigidBodyHandle>,
    pub(crate) entity2collider: HashMap<Entity, ColliderHandle>,
    pub(crate) entity2impulse_joint: HashMap<Entity, ImpulseJointHandle>,
    pub(crate) entity2multibody_joint: HashMap<Entity, MultibodyJointHandle>,
    // This maps the handles of colliders that have been deleted since the last
    // physics update, to the entity they was attached to.
    pub(crate) deleted_colliders: HashMap<ColliderHandle, Entity>,

    pub(crate) collision_events_to_send: Vec<CollisionEvent>,
    pub(crate) contact_force_events_to_send: Vec<ContactForceEvent>,
    pub(crate) character_collisions_collector: Vec<rapier::control::CharacterCollision>,
}

pub fn urdf_rs_robot_to_xurdf(robot: urdf_rs::Robot) -> xurdf::Robot {
    let mut links = Vec::with_capacity(robot.links.len());
    let mut joints = Vec::with_capacity(robot.joints.len());
    for l in robot.links.into_iter() {
        links.push(xurdf::Link {
            name: l.name,
            visuals: l.visual.into_iter()
                .map(|v| {
                    xurdf::Visual {
                        name: v.name,
                        origin: v.origin.into_xurdf(),
                        geometry: v.geometry.into_xurdf(),
                    }
                })
                .collect(),
            inertial: l.inertial.into_xurdf(),
            collisions: l.collision.into_iter()
                .map(|co| {
                    xurdf::Collision {
                        name: co.name,
                        origin: co.origin.into_xurdf(),
                        geometry: co.geometry.into_xurdf(),
                    }
                })
                .collect(),
        })
    }

    for j in robot.joints.into_iter() {
        joints.push(xurdf::Joint {
            name: j.name,
            joint_type: match j.joint_type {
                urdf_rs::JointType::Revolute => "revolute".to_string(),
                urdf_rs::JointType::Continuous => "continuous".to_string(),
                urdf_rs::JointType::Prismatic => "prismatic".to_string(),
                urdf_rs::JointType::Fixed => "fixed".to_string(),
                urdf_rs::JointType::Floating => "floating".to_string(),
                urdf_rs::JointType::Planar => "planar".to_string(),
                urdf_rs::JointType::Spherical => "spherical".to_string(),
            },
            origin: j.origin.into_xurdf(),
            parent: j.parent.link,
            child: j.child.link,
            axis: Vector::from_data(ArrayStorage([j.axis.xyz])),
            limit: xurdf::JointLimit {
                lower: j.limit.lower,
                upper: j.limit.upper,
                effort: j.limit.effort,
                velocity: j.limit.velocity,
            },
        });
    }

    xurdf::Robot {
        name: robot.name,
        links,
        joints,
    }
}
