use std::time::Duration;
use bevy::prelude::Entity;
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
    fn execute(
        &mut self,
        _robot: &Robot,
        _rapier_context_entity: Entity,
        _rapier_handles: &RapierRobotHandles,
        _robot_entities: &RobotEntities,
        physics: &mut PhysicsData
    ) {
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
        self.time_waited = Duration::from_secs(0);
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
