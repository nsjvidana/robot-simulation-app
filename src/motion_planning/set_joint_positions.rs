use std::ops::DerefMut;
use approx::relative_eq;
use bevy::prelude::Entity;
use bevy_rapier3d::prelude::{RapierContextJoints, RapierContextMut};
use bevy_rapier3d::rapier::prelude::{ImpulseJointHandle, MultibodyJointHandle, SPATIAL_DIM};
use crate::motion_planning::Instruction;
use crate::PhysicsData;
use crate::prelude::*;

#[derive(Clone)]
pub struct SetJointPositionsInstruction {
    pub joints_and_positions: Vec<(Entity, [Real; 6])>,
    /// The margin of error for checking when joints have reached their target position.
    pub eps: Real,
    finished: bool
}

impl SetJointPositionsInstruction {
    pub fn new(joints_and_positions: Vec<(Entity, [Real; 6])>) -> Self {
        Self {
            joints_and_positions,
            ..Default::default()
        }
    }
}

impl Default for SetJointPositionsInstruction {
    fn default() -> Self {
        Self {
            joints_and_positions: vec![],
            eps: 0.001,
            finished: false
        }
    }
}

impl Instruction for SetJointPositionsInstruction {
    fn execute(
        &mut self,
        robot: &Robot,
        rapier_context_entity: Entity,
        rapier_handles: &RapierRobotHandles,
        robot_entities: &RobotEntities,
        physics: &mut PhysicsData
    ) {
        let mut finished_joints = 0;
        let mut context: RapierContextMut = W(physics.ctx.rapier_context.get_mut(rapier_context_entity).unwrap()).into();
        let RapierContextJoints {
            impulse_joints,
            multibody_joints,
            ..
        } = context.joints.deref_mut();
        let bodies = &mut context.rigidbody_set.bodies;
        // Iterate over all joints and their target positions in all 6 degrees of freedom
        for (joint_idx, positions) in robot_entities.link_entities
            .iter()
            .enumerate()
            .map(|v| {
                (
                    v.0,
                    self.joints_and_positions.iter()
                        .find(|(joint, _)| *joint == v.1.rigid_body)
                        .map(|v| v.1)
                )
            })
            .filter(|(_, positions)| positions.is_some())
            .map(|(joint_idx, positions)| (joint_idx, positions.unwrap()))
        {
            let link_data = &rapier_handles.joints[joint_idx];
            let joint_handle = link_data.joint;
            let joint = match robot.robot_joint_type {
                RobotJointType::ImpulseJoints => {
                    &mut impulse_joints.get_mut(ImpulseJointHandle(joint_handle), true).unwrap()
                        .data
                }
                RobotJointType::MultibodyJoints(_) => {
                    let (mb, link_id) = multibody_joints.get_mut(MultibodyJointHandle(joint_handle))
                        .unwrap();
                    &mut mb.link_mut(link_id).unwrap().joint.data
                }
            };

            // Setting target positions & counting num of joints that reach their target.
            let curr_joint_positions = W(&*joint).get_joint_position(
                bodies.get(link_data.link1).unwrap().position(),
                bodies.get(link_data.link2).unwrap().position(),
            );
            for motor_idx in 0..SPATIAL_DIM {
                if (joint.motor_axes.bits() & 1 << motor_idx) != 0 {
                    let target_motor_pos = positions[motor_idx];
                    let curr_pos = curr_joint_positions[motor_idx];

                    let speed = joint.motors[motor_idx].target_vel.abs();
                    joint.motors[motor_idx].target_pos = target_motor_pos;
                    joint.motors[motor_idx].target_vel = speed * (target_motor_pos - curr_pos).signum();
                    if relative_eq!(curr_pos, target_motor_pos, epsilon = self.eps) {
                        finished_joints += 1;
                    }
                }
            }
        }

        if finished_joints == self.joints_and_positions.len() {
            self.finished = true;
        }
    }

    fn is_finished(&self) -> bool {
        self.finished
    }

    fn reset_finished_state(&mut self) {
        self.finished = false;
    }

    fn instruction_name(&self) -> &'static str {
        "Set Joint Positions"
    }
}
