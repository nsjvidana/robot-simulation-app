use std::ops::Mul;
use bevy_rapier3d::dynamics::JointAxesMask;
use bevy_rapier3d::na::Isometry3;
use crate::prelude::*;
use crate::prelude::Real as Real;
use bevy_rapier3d::rapier;
use bevy_rapier3d::rapier::prelude::SPATIAL_DIM;

impl W<&rapier::prelude::GenericJoint> {
    // pub fn angle(&self, rb_rot1: &Rotation<Real>, rb_rot2: &Rotation<Real>) -> Real {
    /// Get the joint position of a generic joint using
    /// the isometries of the parent and child rigid bodies of the joint.
    pub fn get_joint_position(self, rb1: &Isometry3<Real>, rb2: &Isometry3<Real>) -> [Real; SPATIAL_DIM] {
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