use crate::kinematics::ik::ForwardDescentCyclic;
use crate::motion_planning::CreatePlanEvent;
use crate::prelude::*;
use crate::ui::{ribbon::finish_ribbon_tab, SelectedEntities, UiEvents, UiResources, View};
use bevy::prelude::{default, EventWriter};
use bevy_egui::egui;
use bevy_egui::egui::{Align, Id, Layout, Response, Ui, UiBuilder};
use openrr_planner::JointPathPlanner;
use std::collections::HashMap;
use std::ops::DerefMut;

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
    fn ui(&mut self, ui: &mut Ui) {
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
        let motion_planning = resources.motion_planning.deref_mut();
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

#[derive(Default)]
pub struct UIElements {
    new_plan: Option<Id>,
    edit_plan: Option<Id>,
}

pub struct InverseKinematicsWindow {
    /// If the IK window is open
    open: bool,
    /// Selected IK solver for joint chain
    selected_solver: IKSolverType,
    solvers: HashMap<IKSolverType, Box<dyn k::InverseKinematicsSolver<Real>>>,
    /// Joint path planner
    planner: Option<JointPathPlanner<Real>>,
    selected_chain: Option<k::Chain<Real>>,
    create_ik_chain: bool,
}

impl Default for InverseKinematicsWindow {
    fn default() -> Self {
        Self {
            open: false,
            selected_solver: IKSolverType::Jacobian,
            solvers: HashMap::with_capacity(2),
            planner: None,
            selected_chain: None,
            create_ik_chain: false,
        }
    }
}

#[derive(PartialEq, Eq, Hash)]
pub enum IKSolverType {
    Jacobian,
    Cyclic,
}

pub fn motion_planning_functionality(
    create_plan_event: &mut EventWriter<CreatePlanEvent>,
    selected_entities: &SelectedEntities,
) {

}

pub fn ik_window(egui_ctx: &mut egui::Context, ui_resources: &mut UiResources) {
    let ik_window = &mut ui_resources.motion_planning.ik_window;
    egui::Window::new("Inverse Kinematcs")
        .open(&mut ik_window.open)
        .show(egui_ctx, |ui| {
            let solver_dropdown = egui::ComboBox::from_label("Solver Type")
                .selected_text(match &ik_window.selected_solver {
                    IKSolverType::Jacobian => "Jacobian",
                    IKSolverType::Cyclic => "Cyclic",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut ik_window.selected_solver,
                        IKSolverType::Jacobian,
                        "Jacobian",
                    );
                    ui.selectable_value(
                        &mut ik_window.selected_solver,
                        IKSolverType::Cyclic,
                        "Cyclic",
                    );
                });
            if solver_dropdown.response.clicked() {
                match ik_window.selected_solver {
                    IKSolverType::Jacobian => {
                        ik_window.solvers.insert(
                            IKSolverType::Jacobian,
                            Box::new(k::JacobianIkSolver::default()) as _,
                        );
                    }
                    IKSolverType::Cyclic => {
                        ik_window.solvers.insert(
                            IKSolverType::Cyclic,
                            Box::new(ForwardDescentCyclic::default()) as _,
                        );
                    }
                }
            }

            ik_window.create_ik_chain = ui.button("Create IK Chain").clicked();
        });
}
