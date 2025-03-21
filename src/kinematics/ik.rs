use std::ops::{Deref, DerefMut};
use bevy::prelude::Component;
use crate::math::{angle_to, project_onto_plane, Real};
use k::{Constraints, Error, InverseKinematicsSolver, Isometry3, JointType, SerialChain, Vector3};

#[derive(Component)]
pub struct KinematicNode(pub k::Node<Real>);

impl Deref for KinematicNode {
    type Target = k::Node<Real>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for KinematicNode {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

macro_rules! cyclic_impl {
    ($struct_name:ident) => {
        pub struct $struct_name {
            pub allowable_target_distance: Real,
            pub allowable_target_angle: Real,
            pub max_iterations: usize,
            /// 0.0 for no damping, 1.0 for damping
            pub per_joint_dampening: Real
        }

        impl Default for $struct_name {
            fn default() -> Self {
                Self {
                    allowable_target_distance: 0.1,
                    allowable_target_angle: 0.1,
                    max_iterations: 1,
                    per_joint_dampening: 0.
                }
            }
        }
    };
}

cyclic_impl!(ForwardAscentCyclic);
cyclic_impl!(ForwardDescentCyclic);
cyclic_impl!(BackwardsCyclic);

impl InverseKinematicsSolver<Real> for ForwardAscentCyclic {
    fn solve(&self, arm: &SerialChain<Real>, target_pose: &Isometry3<Real>) -> Result<(), Error> {
        let mut position_diff = Vector3::new(0.,0.,0.);
        let mut rotation_diff = Vector3::new(0., 0., 0.);
        let mut dist_to_target;
        let mut angle_to_target;

        for _ in 0..self.max_iterations {
            for (i, node) in arm.iter().enumerate() {
                let curr_joint = node.joint();

                let joint_space = {
                    let mut transform = Isometry3::identity();
                    for node in arm.iter().take(i) {
                        transform *= node.joint().local_transform();
                    }
                    transform *= curr_joint.local_transform();
                    transform
                };

                let joint_axis = match curr_joint.joint_type {
                    JointType::Rotational { axis } => axis,
                    JointType::Fixed => continue,
                    #[allow(unused)]
                    JointType::Linear { axis } => todo!()
                };

                let local_end = {
                    let mut transform = Isometry3::identity();
                    for node in arm.iter().skip(i+1) {
                        transform *= node.joint().local_transform()
                    }
                    transform
                };
                let local_target = joint_space.inverse() * target_pose;

                let end_projected = project_onto_plane(&local_end.translation.vector, &joint_axis);
                let target_projected = project_onto_plane(&local_target.translation.vector, &joint_axis);
                let adjustment = angle_to(&end_projected, &target_projected, &joint_axis);
                std::mem::drop(curr_joint);

                node.set_joint_position_clamped(
                    node.joint_position().unwrap() + (adjustment * (1. - self.per_joint_dampening))
                );
            }

            let end_world_space = {
                let mut transform = Isometry3::identity();
                for node in arm.iter() {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            position_diff = target_pose.translation.vector - end_world_space.translation.vector;
            let (x,y,z) = (target_pose.rotation * end_world_space.rotation.inverse())
                .euler_angles();
            rotation_diff = Vector3::new(x,y,z);
            dist_to_target = position_diff.norm();
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                return Ok(());
            }
        }

        Err(Error::NotConvergedError {
            num_tried: self.max_iterations,
            position_diff: Vector3::new(position_diff.x as f64, position_diff.y as f64, position_diff.z as f64),
            rotation_diff: Vector3::new(rotation_diff.x as f64, rotation_diff.y as f64, rotation_diff.z as f64),
        })
    }
    fn solve_with_constraints(&self, _arm: &SerialChain<Real>, _target_pose: &Isometry3<Real>, _constraints: &Constraints) -> Result<(), Error> {
        todo!()
    }
}

impl InverseKinematicsSolver<Real> for ForwardDescentCyclic {
    fn solve(&self, arm: &SerialChain<Real>, target_pose: &Isometry3<Real>) -> Result<(), Error> {
        let mut position_diff = Vector3::new(0.,0.,0.);
        let mut rotation_diff = Vector3::new(0., 0., 0.);
        let mut dist_to_target;
        let mut angle_to_target;

        for _ in 0..self.max_iterations {

            let iter_vec = arm.iter().enumerate().collect::<Vec<_>>();
            for (i, node) in iter_vec.iter().rev() {
                let curr_joint = node.joint();

                let joint_space = {
                    let mut transform = Isometry3::identity();
                    for node in arm.iter().take(*i) {
                        transform *= node.joint().local_transform();
                    }
                    transform *= curr_joint.local_transform();
                    transform
                };

                let joint_axis = match curr_joint.joint_type {
                    JointType::Rotational { axis } => axis,
                    JointType::Fixed => continue,
                    #[allow(unused)]
                    JointType::Linear { axis } => todo!()
                };

                let local_target = joint_space.inverse() * target_pose;
                let local_end = {
                    let mut transform = Isometry3::identity();
                    for node in arm.iter().skip(i+1) {
                        transform *= node.joint().local_transform()
                    }
                    transform
                };

                let target_projected = project_onto_plane(&local_target.translation.vector, &joint_axis);
                let end_projected = project_onto_plane(&local_end.translation.vector, &joint_axis);
                //the angle between the projected vectors is the joint's position (limited by joint limits)
                let adjustment = angle_to(&end_projected, &target_projected, &joint_axis);
                std::mem::drop(curr_joint);
                node.set_joint_position_clamped(
                    node.joint_position().unwrap() + (adjustment * (1. - self.per_joint_dampening))
                );
            }

            let end_world_space = {
                let mut transform = Isometry3::identity();
                for node in arm.iter() {
                    transform *= node.joint().local_transform();
                }
                transform
            };
            position_diff = target_pose.translation.vector - end_world_space.translation.vector;
            let (x,y,z) = (target_pose.rotation * end_world_space.rotation.inverse())
                .euler_angles();
            rotation_diff = Vector3::new(x,y,z);
            dist_to_target = end_world_space.translation.vector.metric_distance(&target_pose.translation.vector);
            angle_to_target = end_world_space.rotation.angle_to(&target_pose.rotation);
            if dist_to_target <= self.allowable_target_distance &&
                angle_to_target <= self.allowable_target_angle
            {
                return Ok(());
            }
        }


        Err(Error::NotConvergedError {
            num_tried: self.max_iterations,
            position_diff: Vector3::new(position_diff.x as f64, position_diff.y as f64, position_diff.z as f64),
            rotation_diff: Vector3::new(rotation_diff.x as f64, rotation_diff.y as f64, rotation_diff.z as f64),
        })
    }
    fn solve_with_constraints(&self, _arm: &SerialChain<Real>, _target_pose: &Isometry3<Real>, _constraints: &Constraints) -> Result<(), Error> {
        todo!()
    }
}
