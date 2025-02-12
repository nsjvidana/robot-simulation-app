use std::collections::HashMap;
use bevy::prelude::{Component, Dir3, Entity, GlobalTransform};
use bevy_rapier3d::geometry::ActiveHooks;
use bevy_rapier3d::prelude::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule, CollisionEvent, ContactForceEvent, Group};
use bevy_rapier3d::rapier;
use bevy_rapier3d::rapier::prelude::{CCDSolver, ColliderHandle, ColliderSet, DefaultBroadPhase, EventHandler, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointHandle, MultibodyJointSet, NarrowPhase, PhysicsPipeline, QueryPipeline, RigidBodyHandle, RigidBodySet};
use nalgebra::Vector3;
use crate::math::Real;

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
