use bevy_egui::egui::{Context, Ui};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use bevy::prelude::{Event, Events, Mut, ResMut, Vec3};
use bevy_egui::{egui, EguiContexts};
use derivative::Derivative;
use k::Isometry3;
use openrr_planner::{JointPathPlanner, JointPathPlannerBuilder, JointPathPlannerWithIk};
use parking_lot::Mutex;
use crate::kinematics::ik::ForwardDescentCyclic;
use crate::math::Real;
use crate::prelude::*;
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};

#[derive(Derivative)]
#[derivative(Default)]
pub struct InverseKinematicsWindow {
    /// If the IK window is open
    open: bool,
    /// Is true when the window just got closed.
    was_closed: bool,
    /// Selected IK solver for joint chain
    selected_solver: IKSolverType,
    #[derivative(Default(value="HashMap::with_capacity(2)"))]
    solvers: HashMap<IKSolverType, Box<dyn k::InverseKinematicsSolver<Real>>>,
    solve_clicked: bool,

    ik_data: Option<IkData>,
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
                    let _ = self.solvers
                        .entry(IKSolverType::Jacobian)
                        .or_insert(Box::new(k::JacobianIkSolver::default()) as _);
                }
                IKSolverType::Cyclic => {
                    let _ = self.solvers
                        .entry(IKSolverType::Cyclic)
                        .or_insert(Box::new(ForwardDescentCyclic::default()) as _);
                }
            }
        }

        self.solve_clicked = ui.button("Solve").clicked();
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> Result<()> {
        let ik_window = &mut resources.motion_planning_tab.ik_window;

        // Flag the IK operation as "canceled" if the window was closed
        if ik_window.was_closed {
            if let Some(ik_data) = &ik_window.ik_data {
                *ik_data.is_finished.lock() = false;
                *ik_data.canceled.lock() = true;
            }
        }
        // Opening the IK window when it is requested.
        for event in events.ik_window_events.drain() {
            ik_window.open = true;
            ik_window.ik_data = Some(event.into());
        }

        if ik_window.solve_clicked {
            let Some(ik_data) = &mut ik_window.ik_data else {
                return Ok(());
            };
            let planner =
                JointPathPlannerBuilder::from_urdf_robot_with_base_dir(
                    ik_data.urdf.clone(),
                    Some(ik_data.urdf_path.parent().expect("invalid urdf path"))
                )
                    .reference_robot(Arc::new(ik_data.chain.clone()))
                    .finalize()
                    .unwrap();
            let solver = ik_window.solvers.remove(&ik_window.selected_solver).unwrap();
            let mut ik_planner = JointPathPlannerWithIk::new(planner, W(solver));
            let compound = ncollide3d::shape::Compound::new(vec![]);
            // TODO: handle the openrr-planner errors properly.
            *ik_data.positions.lock() = ik_planner
                .plan_with_ik("target", &ik_data.target_pose, &compound)
                .map_err(|e| {
                    // Cancel the ik operation if a planning error occurred.
                    *ik_data.is_finished.lock() = false;
                    *ik_data.canceled.lock() = true;
                    Error::FailedOperation(format!("Inverse Kinematics failed: {:?}", e))
                })?;
            *ik_data.is_finished.lock() = true;
        }

        Ok(())
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
        self.was_closed = open != self.open && !open;
        self.open = open;
    }
}

// TODO: impl IK preview

pub struct IkData {
    pub chain: k::Chain<Real>,
    pub is_finished: Arc<Mutex<bool>>,
    pub canceled: Arc<Mutex<bool>>,
    pub urdf: urdf_rs::Robot,
    pub urdf_path: PathBuf,

    pub target_pose: Isometry3<Real>,
    /// Resulting positions from solver. Stored for returning solver result &
    /// previewing IK result.
    pub positions: Arc<Mutex<Vec<Vec<Real>>>>
}

impl From<IkWindowUiEvent> for IkData {
    fn from(event: IkWindowUiEvent) -> Self {
        Self {
            chain: event.chain,
            is_finished: event.is_finished,
            canceled: event.canceled,
            urdf: event.urdf,
            urdf_path: event.urdf_path,

            target_pose: Isometry3::identity(),
            positions: Arc::new(Mutex::new(vec![])),
        }
    }
}

#[derive(PartialEq, Eq, Hash, Default, Copy, Clone, Debug)]
pub enum IKSolverType {
    #[default]
    Jacobian = 0,
    Cyclic,
}

#[derive(Event)]
pub struct IkWindowUiEvent {
    pub chain: k::Chain<Real>,
    /// The output of the IK solver. When not using collision avoidance planning, read
    /// the first element of this vec only.
    pub solver_output: Arc<Mutex<Vec<Vec<Real>>>>,
    pub is_finished: Arc<Mutex<bool>>,
    pub canceled: Arc<Mutex<bool>>,
    pub urdf: urdf_rs::Robot,
    pub urdf_path: PathBuf,
}
