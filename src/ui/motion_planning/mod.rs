mod ik;

pub use ik::*;

use crate::motion_planning::{Instruction, InstructionObject, PlanEvent};
use crate::prelude::*;
use crate::ui::{ribbon::finish_ribbon_tab, RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};
use bevy::prelude::{default, Entity, Resource};
use bevy_egui::egui;
use bevy_egui::egui::{Color32, Context, Frame, Id, Response, Ui};
use ik::InverseKinematicsWindow;
use parking_lot::Mutex;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

#[derive(Default)]
pub struct MotionPlanning {
    responses: Vec<Response>,
    ui_elems: UIElements,
    edit_plan_open: bool,
    ik_window: InverseKinematicsWindow,
    edit_plan_window: EditPlanWindow,
    //RRT?
    //Vehicle controller?
}

impl MotionPlanning {
    const NEW_PLAN: &'static str = "New Plan";
    const EDIT_PLAN: &'static str = "Edit Plan";
}

impl View for MotionPlanning {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        self.responses.clear();

        let num_cols = 2;
        let mut column_rects = Vec::with_capacity(num_cols);
        egui::Grid::new("planning_ribbon")
            .num_columns(num_cols)
            .show(ui, |ui| {
                // Plan
                ui.vertical(|ui| {
                    let new_plan = ui.button(Self::NEW_PLAN);
                        self.ui_elems.new_plan = Some(new_plan.id);
                    let edit_plan = ui.button(Self::EDIT_PLAN);
                        self.ui_elems.edit_plan = Some(edit_plan.id);
                    self.responses.append(&mut vec![new_plan, edit_plan]);
                    column_rects.push((ui.min_rect(), "Plan"));
                });
                // TODO: Kinematics
                ui.vertical(|ui| {

                });
            });

        finish_ribbon_tab!(ui, column_rects);
    }

    fn functionality(
        resources: &mut UiResources,
        events: &mut UiEvents,
    ) -> Result<()> {
        let motion_planning = resources.motion_planning_tab.deref_mut();
        let create_plan = motion_planning.responses
            .iter()
            .find(|v| v.id == motion_planning.ui_elems.new_plan.unwrap())
            .is_some_and(|btn| btn.clicked());
        let edit_plan = motion_planning.responses
            .iter()
            .find(|v| v.id == motion_planning.ui_elems.edit_plan.unwrap())
            .is_some_and(|btn| btn.clicked());

        let selected_entities = resources.selected_entities.deref_mut();
        if create_plan {
            if let Some(robot) = selected_entities.active_robot {
                events.plan_events.send(PlanEvent::CreatePlanEvent {
                    robot_entity: robot,
                    plan: default()
                });
                motion_planning.edit_plan_window.open_in_next_frame = true;
            }
            else {
                return Err(Error::FailedOperation("Create Plan failed: No robot selected!".to_string()));
            }
        }
        // Opening the Edit Plan window
        else if edit_plan || motion_planning.edit_plan_window.open_in_next_frame {
            motion_planning.edit_plan_window.open_in_next_frame = false;
            if let Some(robot) = selected_entities.active_robot {
                if let Ok(plan) = resources.plan_q.get(robot) {
                    motion_planning.edit_plan_window.get_robot_data(
                        robot,
                        plan.instructions.clone()
                    );
                    motion_planning.edit_plan_window.open = true;
                }
                else {
                    return Err(Error::FailedOperation("Edit Plan failed: Selected robot doesn't have a Plan!".to_string()));
                }
            }
            else {
                return Err(Error::FailedOperation("Edit Plan failed: No robot selected!".to_string()));
            }
        }

        EditPlanWindow::functionality(resources, events)?;
        // TODO: InverseKinematicsWindow::functionality(resources, events)?;

        Ok(())
    }
}

impl WindowUI for MotionPlanning {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        self.ik_window.window_ui(egui_ctx, ui_assets);
        self.edit_plan_window.window_ui(egui_ctx, ui_assets);
    }
}

#[derive(Default)]
pub struct UIElements {
    new_plan: Option<Id>,
    edit_plan: Option<Id>,
}

pub struct EditPlanWindow {
    /// Whether the window is open
    pub open: bool,
    /// Used when this window needs to appear upon creating a new plan
    open_in_next_frame: bool,
    pub target_robot: Entity,
    instructions: Arc<Mutex<Vec<InstructionObject>>>,
    pub display_instructions: Vec<InstructionObject>,
    pub instruction_order: Vec<usize>,
    pub new_instruction_modal: NewInstructionModal,

    pub add_instruction_clicked: bool,
    pub remove_instruction_clicked: bool,
    pub cancel_clicked: bool,
    pub save_clicked: bool,

    dragging_instruction: bool,
}

impl EditPlanWindow {
    pub fn get_robot_data(&mut self, target_robot: Entity, instructions: Arc<Mutex<Vec<InstructionObject>>>) {
        let instructions_guard = instructions.lock();
        self.target_robot = target_robot;
        self.instructions = instructions.clone();
        self.display_instructions = instructions_guard.clone();
        self.instruction_order = instructions_guard
            .iter()
            .enumerate()
            .map(|v| v.0)
            .collect();
    }
}

impl Default for EditPlanWindow {
    fn default() -> Self {
        Self {
            open: default(),
            open_in_next_frame: default(),
            target_robot: Entity::PLACEHOLDER,
            instructions: default(),
            display_instructions: default(),
            instruction_order: default(),
            new_instruction_modal: default(),

            add_instruction_clicked: default(),
            remove_instruction_clicked: default(),
            cancel_clicked: default(),
            save_clicked: default(),

            dragging_instruction: false,
        }
    }
}

impl View for EditPlanWindow {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets) {
        let frame = Frame::default().inner_margin(4.0);
        ui.label("Instructions");
        ui.horizontal(|ui| {
            self.add_instruction_clicked = ui.button("+").clicked();
            self.remove_instruction_clicked = ui.button("-").clicked();
        });
        
        let instructions = &mut self.display_instructions;

        let mut from = None;
        let mut to = None;
        let (_, dropped_payload) = ui.dnd_drop_zone::<usize, ()>(frame, |ui| {
            ui.vertical_centered(|ui| {
                for (loc, instruction) in instructions
                    .iter()
                    .map(|v| v.lock().instruction_name())
                    .enumerate()
                {
                    let (item_resp, resp) = ui.horizontal(|ui| {
                        let instruction_ui_id = Id::new(loc);
                        let item_resp =
                            if !self.dragging_instruction {
                                Some(ui.button(instruction))
                            }
                            else {
                                None
                            };
                        let resp = ui.dnd_drag_source(
                            instruction_ui_id,
                            loc,
                            |ui| {
                                if self.dragging_instruction {
                                    let _ = ui.button(instruction);
                                }
                                ui.label("---")
                            },
                        ).response;
                        self.dragging_instruction = ui.ctx().is_being_dragged(instruction_ui_id);
                        (item_resp, resp)
                    }).inner;

                    if item_resp.is_some_and(|v| v.double_clicked()) {
                        // TODO: allow user to edit this instruction.
                    }

                    // Detect items held over the instructions list
                    if let (Some(pointer), Some(hovered_payload)) = (
                        ui.input(|i| i.pointer.interact_pos()),
                        resp.dnd_hover_payload::<usize>(),
                    ) {
                        let rect = resp.rect;

                        // Draw the drop location preview
                        let stroke = egui::Stroke::new(1.0, Color32::WHITE);
                        let insert_row_idx = if *hovered_payload == loc {
                            // Dragged onto this instruction
                            ui.painter().hline(rect.x_range(), rect.center().y, stroke);
                            loc
                        } else if pointer.y < rect.center().y {
                            // Above this instruction
                            ui.painter().hline(rect.x_range(), rect.top(), stroke);
                            loc
                        } else {
                            // Below this instruction
                            ui.painter().hline(rect.x_range(), rect.bottom(), stroke);
                            loc + 1
                        };

                        // When the user drops an instruction onto the list
                        if let Some(dragged_payload) = resp.dnd_release_payload() {
                            // Set the from-to variables describing the change
                            from = Some(dragged_payload);
                            to = Some(insert_row_idx);
                        }
                    }
                }
            });
        });

        // If the user dropped an instruction onto the column,
        // but not into any of the positions in the list
        if let Some(dropped_payload) = dropped_payload {
            // Move the dropped instruction to the bottom of the list
            from = Some(dropped_payload);
            to = Some(usize::MAX);
        }

        if let (Some(from), Some(mut to)) = (from, to) {
            to -= (*from < to) as usize;
            let moved_instruction = instructions.remove(*from);

            to = to.min(instructions.len());
            instructions.insert(to, moved_instruction);
        }

        self.new_instruction_modal.ui(ui, ui_assets);

        ui.horizontal(|ui| {
            self.cancel_clicked = ui.button("Cancel").clicked();
            self.save_clicked = ui.button("Save").clicked();
        });
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> Result<()> {
        let window = &mut resources.motion_planning_tab.edit_plan_window;
        if !window.open { return Ok(()); }

        if window.cancel_clicked {
            window.open = false;
        }
        else if window.save_clicked {
            window.open = false;
            let mut instructions_list = window.instructions.lock();
            instructions_list.clear();
            instructions_list.append(&mut window.display_instructions.clone());
            // events.plan_events.send(PlanEvent::ReorderInstructions {
            //     robot_entity: window.target_robot,
            //     instruction_order: window.display_instructions
            //         .iter()
            //         .enumerate()
            //         .map(|v| v.0)
            //         .collect()
            // });
        }

        if window.add_instruction_clicked {
            window.new_instruction_modal.open = true;
            window.new_instruction_modal.instruction_list.clear();
            window.new_instruction_modal.instruction_list
                .extend(resources.instructions.instructions
                    .iter()
                    .map(|v| dyn_clone::clone_box(&**v))
                );
        }

        NewInstructionModal::functionality(resources, events)?;

        Ok(())
    }
}

impl WindowUI for EditPlanWindow {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        let mut open = self.open;
        egui::Window::new("Edit Plan")
            .open(&mut open)
            .show(egui_ctx, |ui| {
                self.ui(ui, ui_assets);
            });
        // Avoid borrow checker error
        self.open = open;
    }
}

#[derive(Default)]
pub struct NewInstructionModal {
    open: bool,
    instruction_list: Vec<Box<dyn Instruction>>,
    clicked_instruction: Option<Box<dyn Instruction>>,
}

impl View for NewInstructionModal {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        // Reset clicked instruction
        self.clicked_instruction = None;

        if !self.open { return; }

        egui::Modal::new(Id::new("New Instruction Modal")).show(ui.ctx(), |ui| {
            ui.set_min_width(250.);

            ui.heading("Create New Instruction");

            ui.label("Instructions:");
            Frame::default().inner_margin(4.0)
                .fill(Color32::DARK_GRAY)
                .corner_radius(4.0)
                .show(ui, |ui| {
                    for instruction in self.instruction_list.iter().cloned() {
                        let btn = ui.button(instruction.instruction_name());
                        if btn.clicked() {
                            self.clicked_instruction = Some(dyn_clone::clone_box(&*instruction));
                        }
                    }
                });

            ui.separator();

            egui::Sides::new().show(
                ui,
                |_| {},
                |ui| {
                    if ui.button("Create").clicked() {
                        self.open = false;
                    }
                }
            );
        });
    }
    fn functionality(resources: &mut UiResources, _events: &mut UiEvents) -> Result<()> {
        let window = &mut resources.motion_planning_tab.edit_plan_window;
        if let Some(instruction) = &window.new_instruction_modal.clicked_instruction {
            window.display_instructions.push(
                dyn_clone::clone(&*instruction).into()
            );
            window.new_instruction_modal.open = false;
        }
        Ok(())
    }
}
