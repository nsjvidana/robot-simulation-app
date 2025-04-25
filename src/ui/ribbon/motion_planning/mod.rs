pub mod wait;
pub mod set_joint_positions;
pub mod ik;

use crate::error::Error;
use crate::functionality::motion_planning::{Instruction, InstructionObject, Plan};
use crate::prelude::*;
use crate::ui::ribbon::motion_planning::set_joint_positions::SetJointPositionsWindow;
use crate::ui::ribbon::motion_planning::wait::EditWaitWindow;
use crate::ui::{FunctionalUiResources, ManageUtilityWindowsExt, UtilityWindow, View};
use bevy::prelude::{App, Commands, Entity, Event, ResMut, Trigger};
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use derivative::Derivative;
use std::sync::Arc;

pub fn build_app(app: &mut App) {
    app.init_instruction_window(EditWaitWindow::default());
    app.init_instruction_window(SetJointPositionsWindow::default());
}

impl ManageInstructionWindows for App {
    fn init_instruction_window<Window: InstructionWindow<I>, I: Instruction>(&mut self, window: Window) -> &mut Self {
        self.init_utility_window(window);

        self.add_observer(
            |trigger: Trigger<EditInstruction>, mut window: ResMut<Window>| {
                let instruction = trigger.instruction_input.instruction.lock();
                if (*instruction).downcast_ref::<I>().is_some() {
                    window.take_input(trigger.instruction_input.clone());
                }
            }
        );
        self.add_observer(
            |trigger: Trigger<CloseInstructionWindow>, mut window: ResMut<Window>| {
                if window
                    .get_input_data()
                    .as_ref()
                    .is_some_and(|v| Arc::ptr_eq(&v.instruction, &trigger.instruction))
                {
                    window.clear_input();
                    window.set_canceled(true);
                }
            }
        );

        self
    }
}

#[derive(Default)]
pub struct MotionPlanningUi {
    plan_overwrite_modal_open: bool,
    overwrite_active_robot_plan: bool,
    new_plan_clicked: bool,
    edit_plan_clicked: bool,
    edit_plan_window: Option<EditPlanWindow>,
}

impl View for MotionPlanningUi {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        ui.vertical_centered(|ui| {
            self.new_plan_clicked = ui.button("New Plan").clicked();
            self.edit_plan_clicked = ui.button("Edit Plan").clicked();
        });

        if self.plan_overwrite_modal_open {
            egui::Modal::new(egui::Id::new("Overwrite Robot Plan"))
                .show(ui.ctx(), |ui| {
                    ui.heading("This robot already has a Plan.");
                    ui.label("Do you want to \
                               overwrite it with a new one?");
                    ui.label("(Deletes ALL Instructions this robot currently has)");
                    ui.horizontal(|ui| {
                        let yes_clicked = ui.button("Yes").clicked();
                        let cancel_clicked = ui.button("Cancel").clicked();
                        if yes_clicked || cancel_clicked {
                            self.plan_overwrite_modal_open = false;
                            self.overwrite_active_robot_plan = false;
                        }
                        self.overwrite_active_robot_plan = yes_clicked;
                    });
                });
        }

        if let Some(window) = self.edit_plan_window.as_mut() {
            let mut open = true;
            egui::Window::new("Edit Plan")
                .open(&mut open)
                .show(ui.ctx(), |ui| {
                    window.ui(ui, commands);
                });
            if !open || window.canceled {
                window.close_all_windows(commands);
                self.edit_plan_window = None;
            }
        }
    }

    fn functionality(&mut self, resources: &mut FunctionalUiResources) -> Result<()> {
        let Some(active_robot) = resources.selections.active_robot else {
            if self.new_plan_clicked {
                return Err(Error::FailedOperation("New Plan".to_string(), "No robot selected!".to_string()))
            }
            else if self.edit_plan_clicked {
                return Err(Error::FailedOperation("Edit Plan".to_string(), "No robot selected!".to_string()))
            }
            return Ok(());
        };

        if self.new_plan_clicked {
            if resources.robot_plans.contains(active_robot) { self.plan_overwrite_modal_open = true; }
            else {
                resources.commands.entity(active_robot)
                    .insert(Plan::default());
            }
        }

        if self.overwrite_active_robot_plan {
            self.overwrite_active_robot_plan = false;
            resources.commands.entity(active_robot)
                .insert(Plan::default());
        }

        if self.edit_plan_clicked {
            let Ok(plan) = resources.robot_plans.get(active_robot) else {
                return Err(Error::FailedOperation("Edit Plan".to_string(), "Robot has no Plan!".to_string()));
            };
            self.edit_plan_window = Some(EditPlanWindow {
                robot: active_robot,
                instructions: plan.instructions
                    .iter()
                    .map(|v| (v.clone(), None))
                    .collect(),
                ..Default::default()
            });
        }

        // Edit Plan functionality
        if let Some(window) = self.edit_plan_window.as_mut() {
            window.functionality(resources)?;
        }

        // Fire event for updating plan instructions when clicking "save" on Edit Plan window.
        if self.edit_plan_window.as_ref().is_some_and(|v| v.saved) {
            let window = self.edit_plan_window.take().unwrap();
            window.close_all_windows(&mut resources.commands);
            let instructions = &mut resources
                .robot_plans
                .get_mut(active_robot)
                .unwrap()
                .instructions;
            instructions.clear();
            instructions.append(
                &mut window.instructions
                    .into_iter()
                    .map(|(inst, inst_clone)| {
                        inst_clone.unwrap_or_else(|| inst)
                    })
                    .collect()
            );
        }

        Ok(())
    }

    fn view_name(&self) -> &'static str { "Motion Planning" }
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct EditPlanWindow {
    #[derivative(Default(value = "Entity::PLACEHOLDER"))]
    robot: Entity,
    /// The second member of the tuple is a Some value if that instruction in the list has
    /// been edited (right-clicked).
    instructions: Vec<(InstructionObject, Option<InstructionObject>)>,
    selected_instruction_idx: Option<usize>,
    secondary_clicked_instruction: Option<usize>,
    add_instruction_modal: Option<AddInstructionModal>,
    add_clicked: bool,
    saved: bool,
    canceled: bool,
}

impl EditPlanWindow {
    pub fn close_all_windows(&self, commands: &mut Commands) {
        for (_, instruction_clone) in self.instructions.iter() {
            if let Some(instruction) = instruction_clone {
                commands.trigger(CloseInstructionWindow {
                    instruction: instruction.clone()
                });
            }
        }
    }
}

impl View for EditPlanWindow {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        ui.label("Instructions:");
        ui.horizontal(|ui| {
            self.add_clicked = ui.button("+").clicked();
            if ui.button("-").clicked() {
                todo!()
            }
        });

        let instructions_frame = egui::Frame::group(ui.style());
        let mut from = None;
        let mut to = None;
        let (_, dropped_payload) = ui.dnd_drop_zone::<usize, ()>(instructions_frame, |ui| {
            for (idx, instruction) in self.instructions
                .iter()
                .map(|(instruction, _)| instruction.lock())
                .enumerate()
            {
                let drag_id = egui::Id::new(format!("edit_plan_instruction{idx}"));
                let dragging = ui.ctx().is_being_dragged(drag_id);
                ui.horizontal(|ui| {
                    if dragging { self.selected_instruction_idx = None; };
                    let mut instruction_ui = |ui: &mut Ui| {
                        let mut selected = self.selected_instruction_idx.is_some_and(|v| v == idx);
                        let label_clicked = ui.selectable_label(selected, instruction.instruction_name());
                        if label_clicked.clicked() {
                            selected = !selected;
                            if selected {
                                self.selected_instruction_idx = Some(idx);
                            }
                            else {
                                self.selected_instruction_idx = None;
                            }
                        }
                        label_clicked
                    };
                    let drag_resp = ui.dnd_drag_source(drag_id, idx, |ui| {
                        let _drag_handle = ui.label("---");
                        if dragging {
                            instruction_ui(ui);
                        }
                    }).response;
                    // Capture instruction double-click when instruction isn't being dragged.
                    if !dragging {
                        let instruction_resp =  instruction_ui(ui);
                        if instruction_resp.secondary_clicked() {
                            self.secondary_clicked_instruction = Some(idx);
                        }
                    };

                    // Detect items held over the instructions list
                    if let (Some(pointer), Some(hovered_payload)) = (
                        ui.input(|i| i.pointer.interact_pos()),
                        drag_resp.dnd_hover_payload::<usize>(),
                    ) {
                        let rect = drag_resp.rect;

                        // Draw the drop location preview
                        let stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
                        let insert_row_idx = if *hovered_payload == idx {
                            // Dragged onto this instruction
                            ui.painter().hline(rect.x_range(), rect.center().y, stroke);
                            idx
                        } else if pointer.y < rect.center().y {
                            // Above this instruction
                            ui.painter().hline(rect.x_range(), rect.top(), stroke);
                            idx
                        } else {
                            // Below this instruction
                            ui.painter().hline(rect.x_range(), rect.bottom(), stroke);
                            idx + 1
                        };

                        // When the user drops an instruction onto the list
                        if let Some(dragged_payload) = drag_resp.dnd_release_payload() {
                            // Set the from-to variables describing the change in instruction order
                            from = Some(dragged_payload);
                            to = Some(insert_row_idx);
                        }
                    }

                });
            }
        });

        // If the user dropped an instruction onto the instruction list,
        // but not into any of the positions in the list
        if let Some(dropped_payload) = dropped_payload {
            // Move the dropped instruction to the bottom of the list
            from = Some(dropped_payload);
            to = Some(usize::MAX);
        }

        if let (Some(from), Some(mut to)) = (from, to) {
            to -= (*from < to) as usize;
            let moved_instruction = self.instructions.remove(*from);

            to = to.min(self.instructions.len());
            self.instructions.insert(to, moved_instruction);
        }

        ui.horizontal(|ui| {
            self.saved = ui.button("Save").clicked();
            self.canceled = ui.button("Cancel").clicked();
        });

        if let Some(modal) = self.add_instruction_modal.as_mut() {
            egui::Modal::new(egui::Id::new("add_instruction_modal"))
                .show(ui.ctx(), |ui| {
                    modal.ui(ui, commands);
                });
            if modal.canceled {
                self.add_instruction_modal = None;
            }
        }
    }

    fn functionality(&mut self, resources: &mut FunctionalUiResources) -> Result<()> {
        if self.add_clicked {
            self.add_instruction_modal = Some(AddInstructionModal {
                all_instructions: resources
                    .all_instructions
                    .iter()
                    .map(|(_, instruction)| instruction.instruction_name().to_string())
                    .collect(),
                ..Default::default()
            });
        }

        if let Some(modal) = self.add_instruction_modal.as_mut() {
            if let Some(instruction) = modal.clicked_instruction.as_ref() {
                let instruction = resources.all_instructions.get(instruction).unwrap();
                self.instructions.push((
                    InstructionObject::from(dyn_clone::clone_box(&**instruction)),
                    None
                ));
                self.add_instruction_modal = None;
            }
        }

        if let Some(idx) = self.secondary_clicked_instruction.take() {
            let (instruction, instruction_clone) = self.instructions.get_mut(idx).unwrap();
            *instruction_clone = instruction_clone.take().or_else(|| {
                let instruction = dyn_clone::clone_box(&**instruction.lock());
                Some(InstructionObject::from(instruction))
            });

            resources.commands.trigger(EditInstruction {
                instruction_input: InstructionInput {
                    robot: self.robot,
                    instruction: instruction_clone.clone().unwrap()
                },
            });
        }

        Ok(())
    }
}

#[derive(Default)]
pub struct AddInstructionModal {
    all_instructions: Vec<String>,
    clicked_instruction: Option<String>,
    canceled: bool,
}

impl View for AddInstructionModal {
    fn ui(&mut self, ui: &mut Ui, _commands: &mut Commands) {
        ui.heading("Add Instruction");
        ui.label("Choose an instruction to add:");
        ui.group(|ui| {
            for instruction_name in self.all_instructions.iter() {
                if ui.button(instruction_name).clicked() {
                    self.clicked_instruction = Some(instruction_name.clone());
                }
            }
        });
        self.canceled = ui.button("Cancel").clicked()
    }
}

#[derive(Event)]
pub struct EditInstruction {
    pub instruction_input: InstructionInput,
}

#[derive(Event)]
pub struct CloseInstructionWindow {
    pub instruction: InstructionObject,
}

pub trait ManageInstructionWindows {
    fn init_instruction_window<Window: InstructionWindow<I>, I: Instruction>(&mut self, window: Window) -> &mut Self;
}

#[derive(Clone)]
pub struct InstructionInput {
    pub robot: Entity,
    pub instruction: InstructionObject,
}

pub trait InstructionWindow<I: Instruction>: UtilityWindow<Input = InstructionInput> {
    fn get_input_data(&self) -> &Option<InstructionInput>;
}

#[macro_export]
macro_rules! impl_instruction_window {
    ($window:ident, $instruction_type:ty, $output_type:ty, $input_field:ident, $output_field:ident, $canceled_field:ident) => {
        crate::impl_util_window!($window, InstructionInput, $output_type, $input_field, $output_field, $canceled_field);

        impl crate::ui::ribbon::motion_planning::InstructionWindow<$instruction_type> for $window {
            fn get_input_data(&self) -> &Option<crate::ui::ribbon::motion_planning::InstructionInput> {
                &self.$input_field
            }
        }
    };
}

#[derive(Default)]
pub struct InstructionWindows {
    pub windows: Vec<Box<dyn InstructionEditWindow>>,
}

impl View for InstructionWindows {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        for window in self.windows.iter_mut() {
            if window.has_instruction_object() {
                let mut open = true;
                egui::Window::new(window.window_title())
                    .open(&mut open)
                    .show(ui.ctx(), |ui| window.ui(ui, commands));
                if !open {
                    window.set_instruction_object(None);
                }
            }
        }
    }
}

#[macro_export]
macro_rules! impl_instruction_edit_window {
    ($window:ident, $window_title:expr, $instruction_type:ident, $instruction_object_field:ident) => {
        use crate::ui::ribbon::motion_planning::InstructionEditWindow;
        impl InstructionEditWindow for $window {
            fn set_instruction_object(&mut self, instruction: Option<InstructionObject>) {
                self.$instruction_object_field = instruction;
            }

            fn has_instruction_object(&self) -> bool {
                self.$instruction_object_field.is_some()
            }

            fn window_title(&self) -> &'static str {
                $window_title
            }

            fn instruction_type_matches(&self, instruction_object: &InstructionObject) -> bool {
                let mut guard = instruction_object.lock();
                (*guard).downcast_mut::<$instruction_type>().is_some()
            }
        }
    };
}

pub trait InstructionEditWindow: View {
    fn set_instruction_object(&mut self, instruction: Option<InstructionObject>);

    fn has_instruction_object(&self) -> bool;

    fn window_title(&self) -> &'static str;

    fn instruction_type_matches(&self, instruction_object: &InstructionObject) -> bool;
}

