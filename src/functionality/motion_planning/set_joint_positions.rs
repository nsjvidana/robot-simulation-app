use approx::relative_eq;
use bevy::prelude::*;
use bevy_rapier3d::rapier::prelude::SPATIAL_DIM;
use derivative::Derivative;
use crate::prelude::*;
use crate::prelude::Real;
use crate::functionality::motion_planning::{Instruction, InstructionExecuteParameters};
use crate::functionality::robot::RobotQueryDataItem;

#[derive(Component, Reflect, Derivative, Clone)]
#[reflect(Component)]
#[derivative(Default)]
pub struct SetJointPositionsInstruction {
    pub joints_and_positions: Vec<(String, Entity, [Real; SPATIAL_DIM])>,
    #[derivative(Default(value = "0.001"))]
    pub eps: Real,
    finished: bool
}

impl Instruction for SetJointPositionsInstruction {
    fn execute(&mut self, resources: &mut InstructionExecuteParameters, _robot_data: &RobotQueryDataItem) {
        let mut ctx = resources.rapier_context.single_mut();

        let mut finished_joints = 1usize;
        for (_, child_e, motor_target_positions) in &self.joints_and_positions {
            let mb = resources.multibody_handles.get(*child_e).ok();
            let imp = resources.impulse_handles.get(*child_e).ok();
            let mut j = mb
                .map(|h| {
                    let (mb, link_id) = ctx.joints.multibody_joints.get_mut(h.0).unwrap();
                    let parent_id = mb.link(link_id).unwrap().parent_id().unwrap();
                    let parent_rb = mb.link(parent_id).unwrap().rigid_body_handle();
                    let child_l = mb.link_mut(link_id).unwrap();
                    let child_rb = child_l.rigid_body_handle();
                    (parent_rb, child_rb, &mut child_l.joint.data)
                });
            if j.is_none() {
                j = Some({
                    let h = imp.unwrap();
                    let j = ctx.joints.impulse_joints.get_mut(h.0, true).unwrap();
                    (j.body1, j.body2, &mut j.data)
                })
            }
            let (rb1_handle, rb2_handle, joint) = j.unwrap();
            // Checking num of motors that reached their target positions
            let mut finished_motors = 0;
            for motor_idx in 0..SPATIAL_DIM {
                if joint.motor_axes.bits() & 1 << motor_idx == 0 { continue; }
                let rb1 = ctx
                    .rigidbody_set
                    .bodies
                    .get(rb1_handle)
                    .unwrap();
                let rb2 = ctx
                    .rigidbody_set
                    .bodies
                    .get(rb2_handle)
                    .unwrap();
                let curr_pos = W(&*joint).get_joint_position(rb1.position(), rb2.position())[motor_idx];
                let target_pos = motor_target_positions[motor_idx];
                joint.motors[motor_idx].target_pos = target_pos;

                if relative_eq!(curr_pos, target_pos, epsilon = self.eps) {
                    finished_motors |= 1 << motor_idx;
                }
            }
            if joint.motor_axes.bits() == finished_motors {
                finished_joints += 1;
            }
        }

        self.finished = finished_joints == self.joints_and_positions.len();
    }

    fn is_finished(&self) -> bool { self.finished }

    fn reset_finished_state(&mut self) { self.finished = false; }

    fn instruction_name(&self) -> &'static str { "Set Joint Positions" }
}
