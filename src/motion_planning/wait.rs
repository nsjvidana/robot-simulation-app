use std::time::Duration;
use crate::motion_planning::Instruction;
use crate::PhysicsData;
use crate::prelude::{RapierRobotHandles, Robot, RobotEntities};

#[derive(Clone)]
pub struct WaitInstruction {
    pub delay: Duration,
    time_waited: Duration,
}

impl Instruction for WaitInstruction {
    fn execute(&self, robot: &Robot, rapier_handles: &RapierRobotHandles, robot_entities: &RobotEntities, physics: PhysicsData) {
        todo!()
    }

    fn instruction_name(&self) -> &'static str {
        "Wait"
    }
}
