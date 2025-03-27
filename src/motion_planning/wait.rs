use std::time::Duration;
use crate::motion_planning::Instruction;
use crate::PhysicsData;
use crate::prelude::{RapierRobotHandles, Robot, RobotEntities};

#[derive(Clone)]
pub struct WaitInstruction {
    pub delay: Duration,
    time_waited: Duration,
    finished: bool,
}

impl Instruction for WaitInstruction {
    fn execute(&mut self, robot: &Robot, rapier_handles: &RapierRobotHandles, robot_entities: &RobotEntities, physics: PhysicsData) {
        self.time_waited += physics.time.delta();
        if self.time_waited >= self.delay {
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
        "Wait"
    }
}

impl Default for WaitInstruction {
    fn default() -> Self {
        Self {
            delay: Duration::from_secs(1),
            time_waited: Duration::from_secs(0),
            finished: false,
        }
    }
}
