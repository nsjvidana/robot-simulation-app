use crate::prelude::Real;
use crate::prelude::*;
use bevy::prelude::{Component, Entity, GlobalTransform, Ray3d};
use bevy_rapier3d::prelude::{GenericJoint, TypedJoint, Vect};
use bevy_rapier3d::rapier::dynamics::{ImpulseJointHandle, ImpulseJointSet, MultibodyJointHandle, RigidBodyHandle, RigidBodySet};
use bevy_rapier3d::rapier::geometry::{ColliderHandle, ColliderSet};
use bevy_rapier3d::rapier;
use std::collections::HashMap;
use std::ops::DerefMut;
use bevy_rapier3d::parry::query::Ray;
use serde::{Deserialize, Serialize};

impl Into<Vect> for W<rapier::prelude::AngVector<Real>> {
    fn into(self) -> Vect {
        Vect::new(self.x, self.y, self.z)
    }
}

impl AsMut<GenericJoint> for W<&mut TypedJoint> {
    fn as_mut(&mut self) -> &mut GenericJoint {
        match self.deref_mut() {
            TypedJoint::FixedJoint(j) => &mut j.data,
            TypedJoint::GenericJoint(j) => j,
            TypedJoint::PrismaticJoint(j) => &mut j.data,
            TypedJoint::RevoluteJoint(j) => &mut j.data,
            TypedJoint::RopeJoint(j) => &mut j.data,
            TypedJoint::SphericalJoint(j) => &mut j.data,
            TypedJoint::SpringJoint(j) => &mut j.data,
        }
    }
}

impl W<&rapier::prelude::GenericJoint> {
    /// Get the joint position of a generic joint using
    /// the isometries of the parent and child rigid bodies of the joint.
    pub fn get_joint_position(
        self,
        rb1: &rapier::na::Isometry3<Real>,
        rb2: &rapier::na::Isometry3<Real>
    ) -> [Real; rapier::prelude::SPATIAL_DIM] {
        use rapier::prelude::JointAxesMask;
        match self.locked_axes {
            JointAxesMask::LOCKED_REVOLUTE_AXES => {
                [0., 0., 0., self.as_revolute().unwrap().angle(&rb1.rotation, &rb2.rotation), 0., 0.]
            },
            JointAxesMask::LOCKED_PRISMATIC_AXES => {
                let par = rb1 * self.local_frame1;
                let child = rb2 * self.local_frame2;
                let diff = par.inverse() * child;
                [diff.translation.vector.x, 0., 0., 0., 0., 0.]
            },
            _ => unimplemented!()
        }
    }
}

impl Into<Ray> for W<Ray3d> {
    fn into(self) -> Ray {
        Ray {
            origin: self.origin.into(),
            dir: self.direction.as_vec3().into(),
        }
    }
}

#[derive(Component, Serialize, Deserialize, Default, Clone)]
#[allow(dead_code)]
pub struct RapierRigidBodySet {
    /// The set of rigid-bodies part of the simulation.
    pub bodies: RigidBodySet,
    /// NOTE: this map is needed to handle despawning.
    pub(crate) entity2body: HashMap<Entity, RigidBodyHandle>,

    /// For transform change detection.
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
}

#[derive(Component, Serialize, Deserialize, Default, Debug, Clone)]
pub struct RapierContextColliders {
    /// The set of colliders part of the simulation.
    pub colliders: ColliderSet,
    pub(crate) entity2collider: HashMap<Entity, ColliderHandle>,
}

#[derive(Component, Serialize, Deserialize, Default, Debug, Clone)]
pub struct RapierContextJoints {
    /// The set of impulse joints part of the simulation.
    pub impulse_joints: ImpulseJointSet,
    /// The set of multibody joints part of the simulation.
    pub multibody_joints: rapier::prelude::MultibodyJointSet,

    pub(crate) entity2impulse_joint: HashMap<Entity, ImpulseJointHandle>,
    pub(crate) entity2multibody_joint: HashMap<Entity, MultibodyJointHandle>,
}