use crate::kinematics::ik::ForwardDescentCyclic;
use crate::math::Real;
use crate::ui::ribbon::{finish_ribbon_tab, finish_ui_section_vertical};
use bevy_egui::egui;
use bevy_egui::egui::{Align, Layout, Separator, Ui, UiBuilder};
use openrr_planner::JointPathPlanner;
use std::collections::HashMap;
use crate::ui::{EntitySelectionMode, SelectedEntities};

#[derive(Default)]
pub struct MotionPlanning {
    ik_window: InverseKinematicsWindow,
    //RRT?
    //Vehicle controller?
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
    create_ik_chain: bool
}

impl Default for InverseKinematicsWindow {
    fn default() -> Self {
        Self {
            open: false,
            selected_solver: IKSolverType::Jacobian,
            solvers: HashMap::with_capacity(2),
            planner: None,
            selected_chain: None,
            create_ik_chain: false
        }
    }
}

#[derive(PartialEq, Eq, Hash)]
pub enum IKSolverType {
    Jacobian,
    Cyclic
}

pub fn motion_planning_ui(
    ui: &mut Ui,
    motion_planning: &mut MotionPlanning,
    ribbon_height: f32,
) {
    let mut rects = Vec::new();
    ui.horizontal(|ui| {
        egui::Grid::new("planning_ribbon")
            .num_columns(2)
            .show(ui, |ui| {
                rects.push(
                    ik_ui(ui, motion_planning, ribbon_height)
                );
                // rects.push(
                //     ik_ui(ui, motion_planning, ribbon_height)
                // );
            });
    });
    finish_ribbon_tab!(ui, rects);
}

pub fn ik_ui(
    ui: &mut Ui,
    motion_planning: &mut MotionPlanning,
    ribbon_height: f32
) -> (egui::Rect, &'static str) {
    ui.horizontal(|ui| {
        let ret = ui.vertical(|ui| {
            let ik_btn = ui.button("Inverse Kinematics");
            if ik_btn.clicked() {
                motion_planning.ik_window.open = true;
            }
            finish_ui_section_vertical!(ui, "Inverse Kinematics")
        }).inner;
        ui.add(Separator::default().grow(ribbon_height).spacing(0.));
        ret
    }).inner
}

pub fn ik_window(
    egui_ctx: &mut egui::Context,
    motion_planning: &mut MotionPlanning,
) {
    let ik_window = &mut motion_planning.ik_window;
    egui::Window::new("Inverse Kinematcs")
        .open(&mut ik_window.open)
        .show(egui_ctx, |ui| {
            let solver_dropdown = egui::ComboBox::from_label("Solver Type")
                .selected_text(match &ik_window.selected_solver {
                    IKSolverType::Jacobian => "Jacobian", IKSolverType::Cyclic => "Cyclic"
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut ik_window.selected_solver, IKSolverType::Jacobian, "Jacobian");
                    ui.selectable_value(&mut ik_window.selected_solver, IKSolverType::Cyclic, "Cyclic");
                });
            if solver_dropdown.response.clicked() {
                match ik_window.selected_solver {
                    IKSolverType::Jacobian => {
                        ik_window.solvers.insert(IKSolverType::Jacobian, Box::new(k::JacobianIkSolver::default()) as _);
                    },
                    IKSolverType::Cyclic => {
                        ik_window.solvers.insert(IKSolverType::Cyclic, Box::new(ForwardDescentCyclic::default()) as _);
                    }
                }
            }

            ik_window.create_ik_chain = ui.button("Create IK Chain").clicked();
        });
}
