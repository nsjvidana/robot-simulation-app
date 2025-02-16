use bevy::prelude::{Handle, Image, Resource};
use bevy::utils::default;
use bevy_egui::egui;
use bevy_egui::egui::{Align, Button, Label, Layout, Ui, UiBuilder};
use bevy_egui::egui::load::SizedTexture;
use crate::ui::RobotLabUiAssets;

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
    pub robots: Vec<u8>,
}

pub fn simulation_ribbon_ui(
    ui: &mut Ui,
    physics_sim: &mut PhysicsSimulation,
    ui_assets: &RobotLabUiAssets
) {
    let btn = ui.add(
        Button::image(
            SizedTexture::new(ui_assets.new_window_img, [64.0, 64.0])
        )
    );
    ui.label("Run Simulation");
    if btn.clicked() {
        physics_sim.control_window_open = true;
    }
}
