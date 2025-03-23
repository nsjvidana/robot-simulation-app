use std::sync::{Arc, Mutex};
use bevy::prelude::*;
use crate::prelude::*;

pub struct MotionPlanningPlugin;

impl Plugin for MotionPlanningPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<CreatePlanEvent>();

        app.add_systems(PostUpdate, init_plans);
    }
}

#[derive(Component, Default)]
pub struct Plan {
    pub instructions: Vec<Box<dyn Instruction>>,
}

pub trait Instruction: Send + Sync {
    fn execute(
        &self,
        robot: &Robot,
        rapier_handles: &RapierRobotHandles,
        robot_entities: &RobotEntities,
    );
}

#[derive(Event)]
pub struct CreatePlanEvent {
    pub robot_entity: Entity,
    pub plan: Plan,
}

pub fn init_plans(
    mut commands: Commands,
    mut plan_creations: ResMut<Events<CreatePlanEvent>>,
) {
    for creation in plan_creations.drain() {
        commands.entity(creation.robot_entity)
            .insert(creation.plan);
    }
}
