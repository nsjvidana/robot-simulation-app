use crate::functionality::simulation::SimulationState;
use crate::ui::{FunctionalUiResources, View};
use bevy::prelude::{Commands, CommandsStatesExt};
use bevy_egui::egui;

#[derive(Default)]
pub struct RunSimulationUi {
    window: SimulationControlWindow,
}

impl View for RunSimulationUi {
    fn ui(&mut self, ui: &mut egui::Ui, _commands: &mut Commands) {
        ui.vertical_centered(|ui| {
            if ui.button("Simulate").clicked() {
                self.window.open = true;
            }
        });
        
        let window = &mut self.window;
        if window.open {
            let mut open = window.open;
            egui::Window::new("Simulation")
                .open(&mut open)
                .show(ui.ctx(), |ui| {
                    window.play_toggled =
                        if window.simulation_state == RunSimulationState::Playing { ui.button("Pause").clicked() }
                        else { ui.button("Play").clicked() };
                    window.step_clicked = ui.button("Step").clicked();
                    window.reset_clicked = ui.button("Reset").clicked();
                });
            window.open = open;
        }
    }

    fn view_name(&self) -> &'static str {
        "Run Simulation"
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let window = &mut self.window;
        if window.play_toggled {
            match &**res.sim_state {
                SimulationState::Playing => {
                    res.commands.set_state(SimulationState::Paused);
                    window.simulation_state = RunSimulationState::Paused;
                },
                SimulationState::Paused => {
                    res.commands.set_state(SimulationState::Playing);
                    window.simulation_state = RunSimulationState::Playing;
                },
                SimulationState::Stepped => {
                    res.commands.set_state(SimulationState::Playing);
                    window.simulation_state = RunSimulationState::Playing;
                },
                SimulationState::Reset => {
                    res.commands.set_state(SimulationState::Playing);
                    window.simulation_state = RunSimulationState::Playing;
                },
            }
        }
        if window.step_clicked {
            res.commands.set_state(SimulationState::Stepped);
            window.simulation_state = RunSimulationState::Stepped;
        }
        else if window.reset_clicked {
            res.commands.set_state(SimulationState::Reset);
            window.simulation_state = RunSimulationState::Reset;
        }
        Ok(())
    }
}

#[derive(Default)]
pub struct SimulationControlWindow {
    open: bool,
    play_toggled: bool,
    step_clicked: bool,
    reset_clicked: bool,
    simulation_state: RunSimulationState,
}

#[derive(Default, PartialEq, Copy, Clone)]
pub enum RunSimulationState {
    Playing,
    Paused,
    Stepped,
    #[default]
    Reset,
}
