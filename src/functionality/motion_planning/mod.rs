pub mod wait;
pub mod set_joint_positions;

use crate::functionality::motion_planning::set_joint_positions::SetJointPositionsInstruction;
use crate::functionality::motion_planning::wait::WaitInstruction;
use crate::functionality::robot::{RobotEntity, RobotQueryData, RobotQueryDataItem};
use crate::functionality::simulation::{simulation_runner, SimulationState};
use bevy::ecs::system::SystemParam;
use bevy::prelude::{App, Component, Entity, Event, FixedUpdate, IntoSystemConfigs, OnEnter, Query, Reflect, Res, State, SystemSet, Time, With};
use bevy::utils::HashMap;
use bevy_rapier3d::prelude::{RapierImpulseJointHandle, RapierMultibodyJointHandle, WriteRapierContext};
use downcast_rs::{impl_downcast, Downcast};
use dyn_clone::DynClone;
use parking_lot::Mutex;
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

pub fn build_app(app: &mut App) {
    app.init_non_send_resource::<AllInstructions>();

    app
        .register_instruction::<WaitInstruction>()
        .register_instruction::<SetJointPositionsInstruction>();

    app.add_systems(
        OnEnter(SimulationState::Reset),
        reset_plans
    );

    app.add_systems(
        FixedUpdate,
        execute_plans.before(simulation_runner)
    );
}

impl ManageInstructions for App {
    fn register_instruction<T: Instruction + Default + 'static>(&mut self) -> &mut Self {
        let default = T::default();
        self.init_non_send_resource::<AllInstructions>();

        if self
            .world()
            .non_send_resource::<AllInstructions>()
            .iter()
            .any(|(name, _)| name == default.instruction_name())
        { return self; }

        self.world_mut().non_send_resource_mut::<AllInstructions>()
            .insert(default.instruction_name().to_string(), Box::new(default));
        self
    }

    fn unregister_instruction<T: Instruction + Default>(&mut self) -> &mut Self {
        let default = T::default();
        let mut all_instructions = self.world_mut().non_send_resource_mut::<AllInstructions>();
        if all_instructions
            .iter()
            .any(|(name, _)| name == default.instruction_name())
        {
            all_instructions.remove(default.instruction_name());
        };
        self
    }
}

pub trait ManageInstructions {
    fn register_instruction<T: Instruction + Component + Default + 'static>(&mut self) -> &mut Self;
    fn unregister_instruction<T: Instruction + Default>(&mut self) -> &mut Self;
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum MotionPlanningSet {
    UpdateInstructions,
    ExecutePlans
}

#[derive(Component, Default)]
pub struct Plan {
    pub instructions: Vec<InstructionObject>,
}

#[derive(Event)]
pub struct UpdatePlanEvent {
    pub robot: Entity,
    pub instructions: Vec<(Entity, String, Option<InstructionObject>)>,
}

#[derive(Default)]
pub struct AllInstructions(HashMap<String, Box<dyn Instruction>>);

impl Deref for AllInstructions {
    type Target = HashMap<String, Box<dyn Instruction>>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl DerefMut for AllInstructions {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

#[derive(Clone)]
pub struct InstructionObject(Arc<Mutex<Box<dyn Instruction>>>);

impl From<Box<dyn Instruction>> for InstructionObject {
    fn from(value: Box<dyn Instruction>) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }
}

impl Deref for InstructionObject {
    type Target = Arc<Mutex<Box<dyn Instruction>>>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl DerefMut for InstructionObject {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

#[bevy_trait_query::queryable]
pub trait Instruction: Downcast + DynClone + Reflect {
    fn execute(&mut self, resources: &mut InstructionExecuteParameters, robot_data: &RobotQueryDataItem);

    fn is_finished(&self) -> bool;

    fn reset_finished_state(&mut self);

    fn instruction_name(&self) -> &'static str;
}
impl_downcast!(Instruction);
dyn_clone::clone_trait_object!(Instruction);

#[derive(SystemParam)]
pub struct InstructionExecuteParameters<'w, 's> {
    pub time: Res<'w, Time>,
    pub multibody_handles: Query<'w, 's, &'static RapierMultibodyJointHandle>,
    pub impulse_handles: Query<'w, 's, &'static RapierImpulseJointHandle>,
    pub rapier_context: WriteRapierContext<'w, 's>,
}

pub fn execute_plans(
    mut instruction_params: InstructionExecuteParameters,
    robot_q: Query<RobotQueryData>,
    mut plans: Query<(Entity, &mut Plan), With<RobotEntity>>,
    state: Res<State<SimulationState>>
) {
    if *state == SimulationState::Paused || *state == SimulationState::Reset {
        return;
    }
    // TODO: parallelize this?
    for (entity, plan) in plans.iter_mut() {
        for mut instruction in plan.instructions
            .iter()
            .map(|v| v.lock())
        {
            if instruction.is_finished() {
                continue;
            }
            else {
                let robot_query_data = robot_q.get(entity)
                    .unwrap();
                instruction.execute(&mut instruction_params, &robot_query_data);
            }
        }
    }
}

pub fn reset_plans(
    mut plans: Query<&mut Plan>,
) {
    for plan in plans.iter_mut() {
        for instruction in plan.instructions.iter() {
            instruction.lock().reset_finished_state();
        }
    }
}
