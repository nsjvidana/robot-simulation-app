use crate::functionality::motion_planning::wait::WaitInstruction;
use crate::impl_instruction_window;
use crate::ui::ribbon::motion_planning::InstructionInput;
use crate::ui::{NoOutput, View, WindowUi};
use bevy::prelude::{Commands, Resource};
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use derivative::Derivative;
use std::time::Duration;

#[derive(Resource, Derivative)]
#[derivative(Default)]
pub struct EditWaitWindow {
    input: Option<InstructionInput>,
    output: Option<()>,
    instruction_clone: Option<WaitInstruction>,
    should_close: bool,
    open: bool,
    #[derivative(Default(value = "true"))]
    canceled: bool,
}

impl WindowUi for EditWaitWindow {
    fn window_ui(&mut self, egui_context: &mut egui::Context, commands: &mut Commands) {
        let mut open = self.open;
        egui::Window::new("Wait Instruction")
            .open(&mut open)
            .show(egui_context, |ui| {
                self.ui(ui, commands);
            });
        self.open = open;

        if self.canceled || self.should_close {
            self.should_close = false;
            self.open = false;
            self.instruction_clone = None;
            self.input = None;
        }
        self.canceled = !self.open;
    }

    fn set_open(&mut self, open: bool) {
        self.open = open;
    }
}

impl View for EditWaitWindow {
    fn ui(&mut self, ui: &mut Ui, _commands: &mut Commands) {
        let Some(input) = &self.input else {
            return;
        };
        let mut original_instruction = input.instruction.lock();
        let wait = self.instruction_clone.as_mut();
        let wait =
            if wait.is_some() { wait.unwrap() }
            else {
                self.instruction_clone = Some((*original_instruction).downcast_mut::<WaitInstruction>().unwrap().clone());
                self.instruction_clone.as_mut().unwrap()
            };

        ui.horizontal(|ui| {
            ui.label("Duration (seconds): ");
            let mut duration = wait.wait_duration.as_secs_f64();
            ui.add(
                egui::DragValue::new(&mut duration)
                    .min_decimals(3)
            );
            if duration >= 0.0 {
                wait.wait_duration = Duration::from_secs_f64(duration);
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Save").clicked() {
                *original_instruction = Box::new(self.instruction_clone.take().unwrap());
                self.output = Some(());
                self.should_close = true;
            }
            if ui.button("Cancel").clicked() {
                self.canceled = true;
                self.should_close = true;
            }
        });
    }
}

impl_instruction_window!(EditWaitWindow, WaitInstruction, NoOutput, input, output, canceled);
