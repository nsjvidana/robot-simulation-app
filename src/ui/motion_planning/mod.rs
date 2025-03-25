mod ik;

use crate::motion_planning::CreatePlanEvent;
use crate::prelude::*;
use crate::ui::{ribbon::finish_ribbon_tab, RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};
use bevy::prelude::{default, Resource};
use bevy_egui::egui;
use bevy_egui::egui::{Context, Id, Response, Ui};
use std::ops::DerefMut;
use ik::InverseKinematicsWindow;

#[derive(Default)]
pub struct MotionPlanning {
    responses: Vec<Response>,
    ui_elems: UIElements,
    edit_plan_open: bool,
    ik_window: InverseKinematicsWindow,
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
            if selected_entities.active_robot.is_none() {
                return Err(Error::FailedOperation("Create Plan failed: No robot selected!".to_string()));
            }
            events.create_plan_event.send(CreatePlanEvent {
                robot_entity: selected_entities.active_robot.unwrap(),
                plan: default()
            });
            motion_planning.edit_plan_open = true;
        }
        if edit_plan { motion_planning.edit_plan_open = true; }
        Ok(())
    }
}

impl WindowUI for MotionPlanning {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        self.ik_window.window_ui(egui_ctx, ui_assets);
    }
}

#[derive(Default)]
pub struct UIElements {
    new_plan: Option<Id>,
    edit_plan: Option<Id>,
}
