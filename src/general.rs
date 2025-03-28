use std::ops::DerefMut;
use std::path::PathBuf;
use bevy::app::App;
use bevy::core::Name;
use bevy::ecs::schedule::{InternedScheduleLabel, ScheduleLabel};
use bevy::prelude::{Commands, Entity, Event, EventWriter, Events, FixedUpdate, IntoSystemConfigs, Plugin, PostUpdate, Query, ResMut, Resource, With};
use bevy_rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use crate::error::{Error, ErrorEvent};
use crate::prelude::{Robot, RobotSet};
use crate::ui::general::import::ImportJointType;

pub struct GeneralTabPlugin {
    pub physics_schedule: InternedScheduleLabel
}

impl GeneralTabPlugin {
    pub fn new(physics_schedule: impl ScheduleLabel) -> Self {
        Self {
            physics_schedule: physics_schedule.intern(),
        }
    }
}

impl Plugin for GeneralTabPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_event::<ImportEvent>()
            .add_event::<SimulationEvent>();

        app.init_resource::<SimulationSnapshot>();

        app.add_systems(
            PostUpdate,
            import_robots,
        );

        app.add_systems(
            self.physics_schedule,
            handle_simulation_events
                .before(PhysicsSet::StepSimulation)
                .after(PhysicsSet::SyncBackend),
        );
    }
}

#[derive(Resource, Default)]
pub struct SimulationSnapshot {
    pub rapier_context: Vec<u8>,
    pub robot_set: Vec<u8>,
}

#[derive(Event)]
pub struct ImportEvent {
    pub urdf_loader_options: UrdfLoaderOptions,
    pub import_joint_type: ImportJointType,
    pub mb_loader_options: UrdfMultibodyOptions,
    
    pub file_path: PathBuf,
    pub mesh_dir: PathBuf,
}

#[derive(Event)]
pub enum SimulationEvent {
    SimulationAction(SimulationAction),
    // Change whether physics is active or not
    PhysicsActive(bool),
}

pub enum SimulationAction {
    Load,
    Save,
}

pub fn import_robots(
    mut commands: Commands,
    mut imports: ResMut<Events<ImportEvent>>,
    mut errors: EventWriter<ErrorEvent>,
) {
    crate::send_err_events! {
        errors,
        {
            for import in imports.drain() {
                let path = PathBuf::from(&import.file_path);
                let robot_name = path.file_stem().unwrap().to_str().unwrap().to_string();
                let robot_urdf = urdf_rs::read_file(&path)
                    .map_err(|e| Error::Urdf {
                        error: e.to_string(),
                        robot_name: "<Unknown Robot>".to_string(),
                    })?;

                let mesh_dir = import.mesh_dir;
                let mesh_dir =
                    if mesh_dir.exists() && mesh_dir.is_dir() { Some(mesh_dir) } else { None };
                let robot_cmp = Robot::new(robot_urdf, import.urdf_loader_options, path.clone(), mesh_dir);
                commands.spawn((
                    match import.import_joint_type {
                        ImportJointType::Impulse => robot_cmp.with_impulse_joints(),
                        ImportJointType::Multibody => {
                            robot_cmp.with_multibody_joints(import.mb_loader_options)
                        }
                    },
                    Name::new(robot_name),
                ));
            }
            Ok(())
        }
    }
}

pub fn handle_simulation_events(
    mut commands: Commands,
    mut events: ResMut<Events<SimulationEvent>>,
    mut snapshot: ResMut<SimulationSnapshot>,
    mut rapier_config_q: Query<
        (&mut RapierConfiguration, &mut RapierContext),
        With<DefaultRapierContext>,
    >,
    mut robot_set: ResMut<RobotSet>,
) {
    let (mut rapier_config, mut rapier_context) = rapier_config_q.single_mut();
    for event in events.drain() {
        match event {
            SimulationEvent::SimulationAction(action) => {
                match action {
                    SimulationAction::Load => {
                        let resetted_ctx =
                            bincode::deserialize::<RapierContext>(&snapshot.rapier_context).unwrap();
                        let _ = std::mem::replace(rapier_context.deref_mut(), resetted_ctx);
                        // Reset robot transforms
                        let resetted_robot_set = bincode::deserialize::<RobotSet>(&snapshot.robot_set).unwrap();
                        for robot_data in resetted_robot_set.robots.iter().map(|v| v.1) {
                            commands
                                .entity(robot_data.robot_entity)
                                .insert(robot_data.transform);
                        }
                    }
                    SimulationAction::Save => {
                        snapshot.rapier_context.clear();
                        snapshot.robot_set.clear();
                        snapshot.rapier_context.append(&mut bincode::serialize(&*rapier_context).unwrap());
                        snapshot.robot_set.append(&mut bincode::serialize(&*robot_set).unwrap());
                    }
                }
            },
            SimulationEvent::PhysicsActive(physics_active) => {
                rapier_config.physics_pipeline_active = physics_active;
            },
        }
    }
}
