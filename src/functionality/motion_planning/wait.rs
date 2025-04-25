use std::time::Duration;
use bevy::prelude::*;
use derivative::Derivative;
use crate::functionality::motion_planning::{Instruction, InstructionExecuteParameters};
use crate::functionality::robot::RobotQueryDataItem;

#[derive(Component, Reflect, Derivative, Clone)]
#[reflect(Component)]
#[derivative(Default)]
pub struct WaitInstruction {
    #[derivative(Default(value = "Duration::from_secs(1)"))]
    pub wait_duration: Duration,
    time_waited: Duration,
    finished: bool,
}

impl WaitInstruction {
    pub fn new(wait_duration: Duration) -> Self {
        Self {
            wait_duration,
            ..Default::default()
        }
    }
}

impl Instruction for WaitInstruction {
    fn execute(&mut self, resources: &mut InstructionExecuteParameters, _robot_data: &RobotQueryDataItem) {
        self.time_waited += resources.time.delta();
        self.finished = self.time_waited >= self.wait_duration;
    }

    fn is_finished(&self) -> bool { self.finished }

    fn reset_finished_state(&mut self) {
        self.time_waited = Duration::from_secs(0);
        self.finished = false;
    }

    fn instruction_name(&self) -> &'static str {
        "Wait"
    }
}
