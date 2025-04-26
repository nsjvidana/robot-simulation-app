use crate::functionality::simulation::SimulationState;
use crate::ui::{drag_value, drag_value_decimals, FunctionalUiResources, View};
use bevy::prelude::*;
use bevy_egui::egui;
use bevy_rapier3d::plugin::TimestepMode;
use bevy_rapier3d::prelude::SimulationToRenderTime;
use derivative::Derivative;

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

                    ui.collapsing("Simulation Parameters", |ui| {
                        window.time_step_hz_resp = Some(drag_value!(ui, "Timestep frequency (Hz): ", window.time_step_hz));
                            window.time_step_hz = window.time_step_hz.max(0.0000000001);

                        let fps_resp = drag_value_decimals!(ui, "Frames per second: ", window.fps, 3);
                            window.fps = window.fps.max(0.0000000001);
                            if fps_resp.hovered() {
                                fps_resp.show_tooltip_text("The simulation will run this many times each second. \
                                                                 Doesn't alter the timestep in any way.");
                            }
                        window.fps_resp = Some(fps_resp);

                        window.substeps_resp = Some(drag_value_decimals!(ui, "Substeps: ", window.substeps, 0));
                            window.substeps = window.substeps.max(1);
                    });
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

        if let (
            Some(hz),
            Some(substeps),
            Some(fps),
        ) = (&window.time_step_hz_resp, &window.substeps_resp, &window.fps_resp) {
            if hz.changed() {
                let new_dt = 1.0 / window.time_step_hz;
                match &mut *res.timestep_mode {
                    TimestepMode::Fixed { dt, .. } => *dt = new_dt,
                    TimestepMode::Variable { max_dt,.. } => *max_dt = new_dt,
                    TimestepMode::Interpolated { dt,.. } => *dt = new_dt,
                }
            }
            else if substeps.changed() {
                match &mut *res.timestep_mode {
                    TimestepMode::Fixed { substeps, .. } => *substeps = window.substeps,
                    TimestepMode::Variable { substeps,.. } => *substeps = window.substeps,
                    TimestepMode::Interpolated { substeps,.. } => *substeps = window.substeps,
                }
            }
            else if fps.changed() {
                res.commands.insert_resource(Time::<Fixed>::from_hz(window.fps));
            }
        }

        Ok(())
    }
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct SimulationControlWindow {
    open: bool,
    play_toggled: bool,
    step_clicked: bool,
    reset_clicked: bool,
    #[derivative(Default(value = "60."))]
    time_step_hz: f32,
    #[derivative(Default(value = "60."))]
    fps: f64,
    substeps: usize,

    time_step_hz_resp: Option<egui::Response>,
    substeps_resp: Option<egui::Response>,
    fps_resp: Option<egui::Response>,
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
