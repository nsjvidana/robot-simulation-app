use crate::robot::RobotSet;
use crate::ui::RobotLabUiAssets;
use bevy::prelude::{Commands, Query, ResMut, Resource, With};
use bevy::utils::default;
use bevy_egui::egui::load::SizedTexture;
use bevy_egui::egui::{Button, Ui};
use bevy_egui::egui;
use bevy_rapier3d::plugin::{DefaultRapierContext, RapierConfiguration, RapierContext};
use std::ops::DerefMut;

/// Contains the data needed for the physics simulation window
#[derive(Resource)]
pub struct PhysicsSimulation {
    pub physics_active: bool,
    pub step_sim: bool,
    pub sim_resetted: bool,
    /// If the egui window for controlling the physics simulation is open.
    pub control_window_open: bool,
    pub snapshot: SimulationSnapshot,
    pub snapshot_action: SnapshotAction,
}

impl PhysicsSimulation {
    pub fn reset_simulation(&mut self) {
        self.snapshot_action = SnapshotAction::Load;
        self.sim_resetted = true;
        self.physics_active = false;
        self.step_sim = false;
    }
}

impl Default for PhysicsSimulation {
    fn default() -> Self {
        Self {
            physics_active: default(),
            step_sim: default(),
            sim_resetted: true,
            snapshot: default(),
            snapshot_action: default(),
            control_window_open: false,
        }
    }
}

#[derive(Default)]
pub enum SnapshotAction {
    Load,
    Save,
    #[default]
    None
}

#[derive(Default)]
pub struct SimulationSnapshot {
    pub rapier_context: Vec<u8>,
    pub robot_set: Vec<u8>,
}

pub fn simulation_ribbon_ui(
    ui: &mut Ui,
    physics_sim: &mut PhysicsSimulation,
    ui_assets: &RobotLabUiAssets
) {
    let btn = ui.add(
        Button::image(
            SizedTexture::new(ui_assets.new_window_img, [64.0, 64.0])
        ).fill(egui::Color32::TRANSPARENT)
    );
    ui.label("Run Simulation");
    if btn.clicked() {
        physics_sim.control_window_open = true;
    }
}

pub fn simulation_control_window(
    egui_ctx: &mut egui::Context,
    physics_sim: &mut PhysicsSimulation,
) {
    let mut window_open = physics_sim.control_window_open;
    egui::Window::new("Simulation")
        .open(&mut window_open)
        .show(egui_ctx, |ui| {
            let play_toggle_clicked = ui.button(if physics_sim.physics_active {"Pause"} else {"Play"}).clicked();
            let step_clicked = ui.button("Step").clicked();
            let reset_clicked = ui.button("Reset").clicked();

            if play_toggle_clicked { physics_sim.physics_active = !physics_sim.physics_active }
            physics_sim.step_sim = step_clicked;
            if reset_clicked {
                physics_sim.reset_simulation();
            }
            else if physics_sim.physics_active || physics_sim.step_sim {
                if physics_sim.sim_resetted {
                    physics_sim.snapshot_action = SnapshotAction::Save;
                }
                physics_sim.sim_resetted = false;
            }
        });
    physics_sim.control_window_open = window_open;

    //TODO: Let user edit timestep
}

/// The system that lets the physics simulation window control the rapier simulation.
pub fn control_simulation(
   mut commands: Commands,
   mut rapier_config_q: Query<
       (&mut RapierConfiguration, &mut RapierContext),
       With<DefaultRapierContext>
   >,
   mut physics_sim: ResMut<PhysicsSimulation>,
   mut robot_set: ResMut<RobotSet>
) {
    let (mut config, mut rapier_context) = rapier_config_q.single_mut();

    // Loading / saving snapshot for resetting physics simulation
    match physics_sim.snapshot_action {
        SnapshotAction::Load => {
            let snapshot = &physics_sim.snapshot;
            let resetted_ctx = bincode::deserialize::<RapierContext>(&snapshot.rapier_context).unwrap();
            let _ = std::mem::replace(rapier_context.deref_mut(), resetted_ctx);
            // Reset robot transforms
            let resetted_robot_set = bincode::deserialize::<RobotSet>(&snapshot.robot_set).unwrap();
            for robot_data in resetted_robot_set.robots.iter().map(|v| v.1) {
                commands.entity(robot_data.robot_entity).insert(robot_data.transform);
            }
        },
        SnapshotAction::Save => {
            physics_sim.snapshot = SimulationSnapshot {
                rapier_context: bincode::serialize(&*rapier_context).unwrap(),
                robot_set: bincode::serialize(&*robot_set).unwrap()
            }
        }
        SnapshotAction::None => {}
    }
    // Reset the snapshot action to prevent loading/saving every frame
    physics_sim.snapshot_action = SnapshotAction::None;

    if physics_sim.physics_active {
        // Pressing "Step" when the sim is already running pauses the simulation
        if physics_sim.step_sim {
            physics_sim.physics_active = false;
            config.physics_pipeline_active = false;
        }
        else { config.physics_pipeline_active = true; } // If step not pressed, let the sim play.
    }
    else {
        // If the sim is paused, only run it for one frame when the "Step" button is pressed.
        if physics_sim.step_sim { config.physics_pipeline_active = true; }
        else { config.physics_pipeline_active = false; }
    }
}
