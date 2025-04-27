use bevy::prelude::*;
use bevy_egui::egui;
use motion_planning::MotionPlanningUi;
use crate::ui::ribbon::importing::ImportUi;
use crate::ui::ribbon::run_simulation::RunSimulationUi;
use crate::ui::{FunctionalUiResources, View};
use crate::ui::ribbon::fluids::FluidsUi;
use crate::ui::ribbon::place_object::PlaceObjectUi;

pub mod importing;
pub mod run_simulation;
pub mod motion_planning;
pub mod place_object;
pub mod fluids;

pub struct Ribbon {
    pub sections: Vec<Box<dyn View>>,
    pub min_height:  f32,
}

impl Default for Ribbon {
    fn default() -> Self {
        Self {
            sections: vec![
                Box::new(ImportUi::default()),
                Box::new(PlaceObjectUi::default()),
                Box::new(RunSimulationUi::default()),
                Box::new(MotionPlanningUi::default()),
                Box::new(FluidsUi::default()),
            ],
            min_height: 50.,
        }
    }
}

impl View for Ribbon {
    fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Commands) {
        egui_extras::TableBuilder::new(ui)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .columns(
                egui_extras::Column::auto()
                    .at_least(40.0)
                    .resizable(false),
                self.sections.len()
            )
            .body(|mut body| {
                body.row(18.0, |mut row| {
                    for section in self.sections.iter_mut() {
                        row.col(|ui| {
                            ui.set_min_height(self.min_height);
                            ui.vertical_centered(|ui| {
                                section.ui(ui, commands);
                            });
                        });
                    }
                });
                body.row(18.0, |mut row| {
                    for section in self.sections.iter_mut() {
                        row.col(|ui| {
                            ui.vertical_centered(|ui| {
                                ui.label(section.view_name());
                            });
                        });
                    }
                });
            });
    }

    fn functionality(&mut self, functional_resources: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        for section in self.sections.iter_mut() {
            section.functionality(functional_resources)?
        }
        Ok(())
    }
}
