use std::time::Duration;
use bevy_egui::egui;
use bevy_egui::egui::{Context, Ui};
use crate::motion_planning::{InstructionObject, PlanEvent};
use crate::motion_planning::set_joint_positions::SetJointPositionsInstruction;
use crate::motion_planning::wait::WaitInstruction;
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};

#[derive(Default)]
pub struct WaitInstructionWindow {
    open: bool,
    instruction_copy: Option<WaitInstruction>,
    instruction_obj: Option<InstructionObject>,
    save_clicked: bool,
}

impl View for WaitInstructionWindow {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets) {
        let Some(instruction) = &mut self.instruction_copy else {
            return;
        };
        let mut delay = instruction.delay.as_secs_f64();
        ui.horizontal(|ui| {
            ui.label("Wait duration (seconds): ");
            if ui.add(egui::DragValue::new(&mut delay)).changed() {
                instruction.delay = Duration::from_secs_f64(delay);
            }
        });
        self.save_clicked = ui.button("Save").clicked();
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> crate::prelude::Result<()> {
        let window = &mut resources.motion_planning_tab.wait_instruction_window;

        for event in events.plan_events.get_cursor().read(&events.plan_events) {
            if let PlanEvent::EditInstructionEvent(instruction) = event {
                let lock = instruction.lock();
                if let Some(instruction_copy) = (*lock)
                    .downcast_ref::<WaitInstruction>()
                    .cloned()
                {
                    window.open = true;
                    window.instruction_copy = Some(instruction_copy);
                    window.instruction_obj = Some(instruction.clone());
                }
            }
        }

        if window.save_clicked {
            window.save_clicked = false;
            let (Some(instruction_copy), Some(instruction_obj)) =
                (window.instruction_copy.take(), window.instruction_obj.take())
                else { panic!("Missing instruction copy in WaitInstructionWindow!"); };
            *instruction_obj.lock() = Box::new(instruction_copy);
            window.open = false;
        }
        Ok(())
    }
}

impl WindowUI for WaitInstructionWindow {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        let mut open = self.open;
        egui::Window::new("Wait Instruction")
            .open(&mut open)
            .show(egui_ctx, |ui| {
                self.ui(ui, ui_assets)
            });
        self.open = open;
    }
}
