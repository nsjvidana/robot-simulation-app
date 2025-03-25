use bevy_egui::egui::{Context, Ui};
use std::collections::HashMap;
use openrr_planner::JointPathPlanner;
use crate::kinematics::ik::ForwardDescentCyclic;
use crate::math::Real;
use crate::prelude;
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};

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

impl View for InverseKinematicsWindow {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        let solver_dropdown = egui::ComboBox::from_label("Solver Type")
            .selected_text(match &self.selected_solver {
                IKSolverType::Jacobian => "Jacobian",
                IKSolverType::Cyclic => "Cyclic",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.selected_solver,
                    IKSolverType::Jacobian,
                    "Jacobian",
                );
                ui.selectable_value(
                    &mut self.selected_solver,
                    IKSolverType::Cyclic,
                    "Cyclic",
                );
            });
        if solver_dropdown.response.clicked() {
            match self.selected_solver {
                IKSolverType::Jacobian => {
                    self.solvers.insert(
                        IKSolverType::Jacobian,
                        Box::new(k::JacobianIkSolver::default()) as _,
                    );
                }
                IKSolverType::Cyclic => {
                    self.solvers.insert(
                        IKSolverType::Cyclic,
                        Box::new(ForwardDescentCyclic::default()) as _,
                    );
                }
            }
        }

        self.create_ik_chain = ui.button("Create IK Chain").clicked();
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> prelude::Result<()> {
        todo!()
    }
}

impl WindowUI for InverseKinematicsWindow {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        let mut open = self.open;
        egui::Window::new("Inverse Kinematcs")
            .open(&mut open)
            .show(egui_ctx, |ui| {
                self.ui(ui, ui_assets)
            });
        // Avoid borrow checker error
        self.open = open;
    }
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