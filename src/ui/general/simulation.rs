use std::ops::DerefMut;
use bevy::prelude::{default, Resource};
use bevy_egui::egui;
use bevy_egui::egui::{Button, Context, Ui};
use bevy_egui::egui::load::SizedTexture;
use crate::general::{SimulationAction, SimulationEvent};
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};

#[derive(Resource)]
pub struct Simulation {
    pub control_window_open: bool,
    pub physics_active: bool,
    pub step_sim: bool,
    pub sim_resetted: bool,

    pub play_toggle_clicked: bool,
    pub step_clicked: bool,
    pub reset_clicked: bool,
}

impl Default for Simulation {
    fn default() -> Self {
        Self {
            control_window_open: default(),
            physics_active: default(),
            step_sim: default(),
            sim_resetted: true,
            play_toggle_clicked: default(),
            step_clicked: default(),
            reset_clicked: default(),
        }
    }
}

impl Simulation {
    pub fn reset_simulation(&mut self) {
        self.sim_resetted = true;
        self.physics_active = false;
        self.step_sim = false;
    }
}

impl View for Simulation {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets) {
        let btn = ui.add(
            Button::image(SizedTexture::new(ui_assets.new_window_img, [64.0, 64.0]))
                .fill(egui::Color32::TRANSPARENT),
        );
        ui.label("Run Simulation");
        if btn.clicked() {
            self.control_window_open = true;
        }
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> crate::prelude::Result<()> {
        let physics_sim = resources.general_tab.simulation.deref_mut();
        let events = &mut events.simulation_events;

        let mut is_stepping_sim = false;
        let prev_physics_active = physics_sim.physics_active;
        if physics_sim.play_toggle_clicked {
            physics_sim.physics_active = !physics_sim.physics_active;
        }
        else if physics_sim.step_clicked {
            if physics_sim.physics_active {
                physics_sim.physics_active = false;
            }
            else {
                is_stepping_sim = true;
            }
        }
        else if physics_sim.reset_clicked {
            physics_sim.physics_active = true;
        }

        // Handling events
        if prev_physics_active != physics_sim.physics_active {
            if physics_sim.physics_active {
                // If exiting resetted state, save a physics snapshot
                if physics_sim.sim_resetted {
                    events.send(SimulationEvent::SimulationAction(SimulationAction::Save));
                }
                physics_sim.sim_resetted = false;
                events.send(SimulationEvent::PhysicsActive(true));
            }
            else {
                events.send(SimulationEvent::PhysicsActive(false));
            }
        }
        if is_stepping_sim {
            if physics_sim.sim_resetted {
                events.send(SimulationEvent::SimulationAction(SimulationAction::Save));
            }
            physics_sim.sim_resetted = false;
            events.send(SimulationEvent::StepOnce);
        }

        Ok(())
    }
}

impl WindowUI for Simulation {
    fn window_ui(&mut self, egui_ctx: &mut Context, _ui_assets: &RobotLabUiAssets) {
        let mut window_open = self.control_window_open;
        egui::Window::new("Simulation")
            .open(&mut window_open)
            .show(egui_ctx, |ui| {
                self.play_toggle_clicked = ui
                    .button(if self.physics_active {
                        "Pause"
                    } else {
                        "Play"
                    })
                    .clicked();
                self.step_clicked = ui.button("Step").clicked();
                self.reset_clicked = ui.button("Reset").clicked();

                
            });
        self.control_window_open = window_open;
    }
}