pub mod wait;
pub mod set_joint_positions;

use std::ops::{Deref, DerefMut};
use std::sync::Arc;
use bevy::ecs::schedule::{InternedScheduleLabel, ScheduleLabel};
use crate::prelude::*;
use bevy::prelude::*;
use bevy_rapier3d::plugin::DefaultRapierContext;
use bevy_rapier3d::prelude::{PhysicsSet, RapierContextEntityLink};
use dyn_clone::DynClone;
use parking_lot::{Mutex, MutexGuard};
use crate::general::SimulationEvent;
use crate::PhysicsData;
use crate::ui::general::simulation::Simulation;

pub struct MotionPlanningPlugin {
    physics_schedule: InternedScheduleLabel,
}

impl MotionPlanningPlugin {
    pub fn new(physics_schedule: impl ScheduleLabel) -> Self {
        Self { physics_schedule: physics_schedule.intern() }
    }
}

impl Plugin for MotionPlanningPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<PlanEvent>();

        app.init_resource::<AllInstructions>()
            .init_resource::<RunPlansState>();
        app.world_mut().resource_mut::<AllInstructions>()
            .instructions
            .push(Box::new(wait::WaitInstruction::default()));

        app.add_systems(
            PostUpdate,
            (sync_plans, plan_execution_prep)
                .chain()
        );

        app.add_systems(
            self.physics_schedule,
            run_plans
                .before(PhysicsSet::SyncBackend)
        );
    }
}

impl Default for MotionPlanningPlugin {
    fn default() -> Self {
        Self {
            physics_schedule: FixedUpdate.intern()
        }
    }
}

#[derive(Resource, Default)]
pub struct AllInstructions {
    pub instructions: Vec<Box<dyn Instruction>>,
}

impl Deref for AllInstructions {
    type Target = Vec<Box<dyn Instruction>>;
    fn deref(&self) -> &Self::Target {
        &self.instructions
    }
}

impl DerefMut for AllInstructions {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.instructions
    }
}

#[derive(Component, Default)]
pub struct Plan {
    pub instructions: Arc<Mutex<Vec<InstructionObject>>>,
}

impl Plan {
    pub fn instructions(&self) -> MutexGuard<Vec<InstructionObject>> {
        self.instructions.lock()
    }
}

#[derive(Clone)]
pub struct InstructionObject(pub Arc<Mutex<Box<dyn Instruction>>>);

impl InstructionObject {
    pub fn lock(&self) -> MutexGuard<'_, Box<dyn Instruction>> {
        self.0.lock()
    }
}

impl<T: Instruction + 'static> From<T> for InstructionObject {
    fn from(value: T) -> Self {
        Self(Arc::new(Mutex::new(Box::new(value))))
    }
}

impl From<Box<dyn Instruction + 'static>> for InstructionObject {
    fn from(value: Box<dyn Instruction + 'static>) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }
}

pub trait Instruction: Send + Sync + DynClone {
    fn execute(
        &mut self,
        robot: &Robot,
        rapier_context_entity: Entity,
        rapier_handles: &RapierRobotHandles,
        robot_entities: &RobotEntities,
        physics: &mut PhysicsData,
    );

    fn is_finished(&self) -> bool;

    fn reset_finished_state(&mut self);

    fn instruction_name(&self) -> &'static str;
}

dyn_clone::clone_trait_object!(Instruction);

#[derive(Event)]
pub enum PlanEvent {
    CreatePlanEvent {
        robot_entity: Entity,
        plan: Plan,
    },
    ReorderInstructions {
        robot_entity: Entity,
        instruction_order: Vec<usize>
    },
}

pub fn sync_plans(
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
                let plan = plans.get_mut(robot_entity).unwrap();
                let mut instructions = plan.instructions();
                *instructions = instruction_order.into_iter()
                    .map(|v| instructions.remove(v))
                    .collect();
            },
        }
    }
}

#[derive(Default, Resource)]
struct RunPlansState {
    physics_active: bool
}

pub fn plan_execution_prep(
    mut state: ResMut<RunPlansState>,
    robots: Query<(&Plan, &Robot, &RobotHandle, &RapierRobotHandles)>,
    mut simulation_events: EventReader<SimulationEvent>,
) {
    // TODO: add support for stepping physics
    for event in simulation_events.read() {
        if let SimulationEvent::PhysicsActive(active) = event {
            state.physics_active = *active;
            if !active {
                // Reset all robot plans when physics gets disabled.
                for (plan, ..) in robots.iter() {
                    for instruction in plan.instructions.lock().iter() {
                        instruction.lock().reset_finished_state();
                    }
                }
            }
        }
    }
}

pub fn run_plans(
    mut state: ResMut<RunPlansState>,
    mut physics: PhysicsData,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    robots: Query<(&Plan, &Robot, Option<&RapierContextEntityLink>, &RobotHandle, &RapierRobotHandles)>,

    robot_set: Res<RobotSet>,
) {
    if !state.physics_active {
        return;
    }

    for (
        plan,
        robot,
        ctx_link,
        robot_handle,
        rapier_handles
    ) in robots.iter() {
        let context_link = ctx_link.map_or_else(|| default_context_q.single(), |link| link.0);
        let mut instructions = plan.instructions.lock();
        for mut instruction in instructions
            .iter()
            .map(|v| v.0.lock())
        {
            //TODO: add support for a loop instruction (maybe pass instruction list into instruction?)
            if instruction.is_finished() { continue; }
            instruction.execute(
                robot,
                context_link,
                rapier_handles,
                &robot_set.robots.get(robot_handle.0).unwrap().entities,
                &mut physics
            );
            if !instruction.is_finished() { break; }
        }
    }
}
