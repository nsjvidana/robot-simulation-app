use crate::kinematics::ik::ForwardDescentCyclic;
use crate::math::Real;
use crate::prelude::*;
use crate::ui::{GizmosUi, GizmosUiParameters, RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};
use bevy::prelude::{Color, Entity, Event, Isometry3d, Quat, Vec3};
use bevy_egui::egui::{Context, Image, Ui};
use bevy_egui::egui;
use derivative::Derivative;
use k::Isometry3;
use openrr_planner::{JointPathPlannerBuilder, JointPathPlannerWithIk};
use parking_lot::Mutex;
use std::collections::HashMap;
use std::ops::Deref;
use std::path::PathBuf;
use std::sync::Arc;
use bevy_egui::egui::load::SizedTexture;
use bevy_rapier3d::prelude::GenericJoint;
use crate::ui::entity_selection::{EntitySelectionResources, SelectionRequest, SelectionResponse};

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
    end_effector_clicked: bool,

    end_effector_selection_response: Option<Arc<Mutex<SelectionResponse>>>,
    ik_data: Option<IkData>,
}

impl InverseKinematicsWindow {
    fn end_effector_selection_filter(clicked_entity: Entity, sel_resources: &EntitySelectionResources) -> bool {
        let Some(active_robot) = sel_resources.selected_entities.active_robot else {
            return false;
        };
        let is_from_active_robot =
            sel_resources.robot_part_q.get(clicked_entity)
                .is_ok_and(|(part, _)| part.0 == active_robot) &&
                sel_resources.kinematic_nodes.contains(clicked_entity);
        if !is_from_active_robot {
            return false;
        }

        let mut joint = sel_resources.joint_q.get(clicked_entity);
        while let Ok((imp, mb)) = joint {
            let (j, parent) =
                if let Some(imp) = imp { (W(&imp.data), imp.parent) }
                else if let Some(mb) = mb { (W(&mb.data), mb.parent) }
                else { return false; };
            let generic_j: &GenericJoint = j.as_ref();
            if generic_j.raw.motor_axes.bits() == 0 && generic_j.raw.as_fixed().is_none(){
                return false;
            }
            joint = sel_resources.joint_q.get(parent);
        }
        true
    }
}

impl View for InverseKinematicsWindow {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        let Some(ik_data) = &self.ik_data else {
            return;
        };
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
        self.solve_clicked = ui.button("Solve").clicked();

        self.end_effector_clicked =
            if let Some(end_effector) = &ik_data.end_effector {
                // TODO: add image to end effector btn
                ui.button("End Effector (Selected)").clicked()
            }
            else if self.end_effector_selection_response.is_some() {
                ui.button("End Effector (Selecting)").clicked()
            }
            else {
                ui.button("Select End Effector").clicked()
            };

        if solver_dropdown.response.clicked() || self.solve_clicked {
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
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> Result<()> {
        let window = &mut resources.motion_planning_tab.ik_window;

        // Flag the IK operation as "canceled" if the window was closed
        if window.was_closed {
            if let Some(ik_data) = &window.ik_data {
                *ik_data.canceled.lock() = true;
            }
            window.ik_data = None;
        }
        // Opening the IK window when it is requested.
        for event in events.ik_window_events.drain() {
            window.open = true;
            window.ik_data = Some(event.into());
        }

        let Some(ik_data) = &mut window.ik_data else {
            return Ok(())
        };

        if window.end_effector_clicked {
            if window.end_effector_selection_response.is_some() {
                window.end_effector_selection_response.as_ref().unwrap().lock().cancel_response();
                window.end_effector_selection_response = None;
            }
            else {
                ik_data.end_effector = None;
                window.end_effector_selection_response = Some(
                    SelectionRequest::new(Self::end_effector_selection_filter)
                        .send(&mut resources.entity_selection_server)
                );
            }
        }

        if let Some(resp) = window.end_effector_selection_response.to_owned() {
            let resp = resp.lock();
            if let Some(end_entity) = resp.get_selection() {
                ik_data.end_effector = Some(
                    resources.kinematic_nodes
                        .get(end_entity)
                        .unwrap()
                        .deref()
                        .clone()
                );
                window.end_effector_selection_response = None;
            }
        }

        if window.solve_clicked {
            let Some(end_effector) = ik_data.end_effector.clone() else {
                return Err(Error::FailedOperation("Inverse Kinematics Failed: No end effector selected!".to_string()));
            };

            let planner =
                JointPathPlannerBuilder::from_urdf_robot_with_base_dir(
                    ik_data.urdf.clone(),
                    Some(ik_data.urdf_path.parent().expect("invalid urdf path"))
                )
                    .collision_check_margin(0.01)
                    .reference_robot(Arc::new(ik_data.chain.clone()))
                    .finalize()
                    .unwrap();
            let solver = window.solvers.remove(&window.selected_solver).unwrap();
            let mut ik_planner = JointPathPlannerWithIk::new(planner, W(solver));
            let compound = ncollide3d::shape::Compound::new(vec![]);
            // TODO: handle the openrr-planner errors properly.
            let positions = ik_planner
                .plan_with_ik(end_effector.joint().name.as_str(), &ik_data.target_pose, &compound)
                .map_err(|e| {
                    // Cancel the ik operation if a planning error occurred.
                    *ik_data.canceled.lock() = true;
                    Error::FailedOperation(format!("Inverse Kinematics failed: {:?}", e))
                })?;
            *ik_data.output.lock() = Some(IkOutput {
                positions,
                serial_chain: k::SerialChain::from_end(&end_effector),
            });
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
impl GizmosUi for InverseKinematicsWindow {
    fn gizmos_ui(ui_resources: &mut UiResources, gizmos_resources: &mut GizmosUiParameters,) {
        let window = &mut ui_resources.motion_planning_tab.ik_window;
        let Some(ik_data) = &mut window.ik_data else {
            return;
        };

        for trans in ik_data.chain.update_transforms().into_iter() {
            let t = trans.translation;
            let r = trans.rotation;
            gizmos_resources.gizmos.sphere(
                Isometry3d {
                    translation: Vec3::new(t.x, t.y, t.z).into(),
                    rotation: Quat::from_array([r.i, r.k, r.j, r.w])
                },
                0.1,
                Color::WHITE
            );
        }

        let t = ik_data.target_pose.translation;
        gizmos_resources.gizmos.sphere(
            Isometry3d::from_translation(Vec3::new(t.x, t.y, t.z)),
            0.1,
            Color::linear_rgba(1., 1., 0., 1.),
        );
    }

    fn gizmos_functionality(ui_resources: &mut UiResources, gizmos_resources: &mut GizmosUiParameters, events: &mut UiEvents) -> Result<()> {
        Ok(())
    }
}

pub struct IkData {
    pub chain: k::Chain<Real>,
    pub canceled: Arc<Mutex<bool>>,
    pub urdf: urdf_rs::Robot,
    pub urdf_path: PathBuf,
    pub end_effector: Option<k::Node<Real>>,

    pub target_pose: Isometry3<Real>,
    /// Resulting positions from solver. Stored for returning solver result &
    /// previewing IK result.
    pub output: Arc<Mutex<Option<IkOutput>>>
}

impl From<IkWindowUiEvent> for IkData {
    fn from(event: IkWindowUiEvent) -> Self {
        Self {
            chain: event.chain,
            canceled: event.canceled,
            urdf: event.urdf,
            urdf_path: event.urdf_path,
            end_effector: None,

            target_pose: event.target_pose,
            output: Arc::new(Mutex::new(None)),
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
    pub solver_output: Arc<Mutex<Option<IkOutput>>>,
    pub canceled: Arc<Mutex<bool>>,
    pub urdf: urdf_rs::Robot,
    pub urdf_path: PathBuf,
    pub target_pose: Isometry3<Real>,
}

pub struct IkOutput {
    pub serial_chain: k::SerialChain<Real>,
    pub positions: Vec<Vec<Real>>,
}
