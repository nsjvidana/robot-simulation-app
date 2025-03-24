pub mod import;
pub mod position_tools;
pub mod simulation;

use bevy::ecs::system::SystemParam;
use bevy::prelude::{ResMut, Resource};
use bevy_egui::egui;
use bevy_egui::egui::{Context, Ui};
use import::Import;
use position_tools::PositionTools;
use simulation::Simulation;
use crate::prelude::*;
use crate::ui::ribbon::finish_ribbon_tab;
use crate::ui::{GizmosUi, GizmosUiParameters, RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};

#[derive(SystemParam)]
pub struct GeneralTab<'w> {
    pub import: ResMut<'w, Import>,
    pub position_tools: ResMut<'w, PositionTools>,
    pub simulation: ResMut<'w, Simulation>
}

impl<'w> View for GeneralTab<'w> {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets) {
        let num_cols = 3;
        let mut column_rects = Vec::with_capacity(num_cols);
        egui::Grid::new("General Grid")
            .num_columns(num_cols)
            .show(ui, |ui| {
                // Import
                ui.vertical(|ui| {
                    self.import.ui(ui, ui_assets);
                    column_rects.push((ui.min_rect(), "Import"));
                });
                // Position
                ui.vertical(|ui| {
                    self.position_tools.ui(ui, ui_assets);
                    column_rects.push((ui.min_rect(), "Position"));
                });
                // Simulation
                ui.vertical(|ui| {
                    self.simulation.ui(ui, ui_assets);
                    column_rects.push((ui.min_rect(), "Simulation"));
                });
            });

        finish_ribbon_tab!(ui, column_rects);
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> crate::prelude::Result<()> {
        Import::functionality(resources, events)?;
        Simulation::functionality(resources, events)?;
        Ok(())
    }
}

impl<'w> WindowUI for GeneralTab<'w> {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        self.import.window.window_ui(egui_ctx, ui_assets);
        self.simulation.window_ui(egui_ctx, ui_assets);
    }
}

impl<'w> GizmosUi for GeneralTab<'w> {
    fn gizmos_ui(
        ui_resources: &mut UiResources,
        gizmos_resources: &mut GizmosUiParameters
    ) {
        PositionTools::gizmos_ui(ui_resources, gizmos_resources);
    }

    fn gizmos_functionality(
        ui_resources: &mut UiResources,
        gizmos_resources: &mut GizmosUiParameters,
        events: &mut UiEvents,
    ) -> Result<()> {
        PositionTools::gizmos_functionality(ui_resources, gizmos_resources, events)?;
        Ok(())
    }
}
