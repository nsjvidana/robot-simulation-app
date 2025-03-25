use crate::prelude::*;
use bevy::prelude::*;

pub struct MotionPlanningPlugin;

impl Plugin for MotionPlanningPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<PlanEvent>();

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

    fn instruction_name(&self) -> &'static str;
}

#[derive(Event)]
pub enum PlanEvent {
    CreatePlanEvent {
        robot_entity: Entity,
        plan: Plan,
    },
    ReorderInstructions {
        robot_entity: Entity,
        instruction_order: Vec<usize>
    }
}

pub fn init_plans(
    mut commands: Commands,
    mut plan_events: ResMut<Events<PlanEvent>>,
    mut plans: Query<&mut Plan>
) {
    for event in plan_events.drain() {
        match event {
            PlanEvent::CreatePlanEvent { robot_entity, plan } => {
                commands.entity(robot_entity)
                    .insert(plan);
            },
            PlanEvent::ReorderInstructions { robot_entity, instruction_order } => {
                let instructions = &mut plans.get_mut(robot_entity).unwrap()
                    .instructions;
                *instructions = instruction_order.into_iter()
                    .map(|v| instructions.remove(v))
                    .collect();
            }
            _ => {}
        }
    }
}
