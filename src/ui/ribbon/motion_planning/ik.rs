use crate::error::Error;
use crate::impl_util_window;
use crate::prelude::Real;
use crate::ui::selecting::{PickingEvent, PickingFilterResources, PickingRequest, PickingRequestCommandExt, PickingResponse};
use crate::ui::{drag_value, GizmosUi, GizmosUiResources, UtilityOutputEvent, View, GIZMO_DISTANCE};
use crate::ui::{FunctionalUiResources, WindowUi};
use bevy::color::Color;
use bevy::prelude::{Commands, Entity, Resource, Vec3};
use bevy::utils::HashMap;
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use bevy_rapier3d::rapier::prelude::SPATIAL_DIM;
use derivative::Derivative;
use k::{Constraints, Isometry3, JointType, SerialChain};
use openrr_planner::{JointPathPlanner, JointPathPlannerBuilder, JointPathPlannerWithIk};
use std::fmt::Formatter;
use std::ops::{Deref, DerefMut, Mul};
use std::sync::Arc;

#[derive(Resource, Derivative)]
#[derivative(Default)]
pub struct IkWindow {
    robot: Option<Entity>,
    output: Option<IkOutput>,
    output_clone: Option<IkOutput>,
    end_picking_resp: Option<PickingResponse>,
    target_picking_resp: Option<PickingResponse>,

    end_effector: Option<Entity>,
    solvers: HashMap<String, IkSolver>,
    #[derivative(Default(value = "\"Jacobian\".to_string()"))]
    selected_solver: String,
    planner: Option<JointPathPlanner<Real>>,
    target_pos: Vec3,

    save_clicked: bool,
    solve_clicked: bool,
    should_close: bool,
    open: bool,
    #[derivative(Default(value = "true"))]
    canceled: bool,
}

impl WindowUi for IkWindow {
    fn window_ui(&mut self, egui_context: &mut egui::Context, commands: &mut Commands) {
        let mut open = self.open;
        egui::Window::new("Inverse Kinematics")
            .open(&mut open)
            .show(egui_context, |ui| {
                self.ui(ui, commands);
            });
        self.open = open;
        self.canceled = !self.open;

        if self.canceled || self.should_close && self.robot.is_some() {
            self.should_close = false;
            self.canceled = false;
            self.open = false;
            self.robot = None;
            self.end_effector = None;
            self.target_pos = Vec3::ZERO;
            self.target_picking_resp.take().map(|v| v.cancel_request());
            self.end_picking_resp.take().map(|v| v.cancel_request());
            commands.trigger(UtilityOutputEvent::<Self> {
                output: None
            });
        }
    }

    fn set_open(&mut self, open: bool) {
        self.open = open;
    }
}

impl View for IkWindow {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        //TODO: display positions or gizmos ui
        ui.label("Solved positions:");
        if let Some(output) = self.output_clone.as_ref() {
            if let Some(output) = output.last() {
                let table = egui_extras::TableBuilder::new(ui)
                    .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                    .column(
                        egui_extras::Column::auto()
                            .at_least(40.0)
                            .resizable(true)
                    )
                    .column(
                        egui_extras::Column::remainder()
                            .at_least(40.0)
                            .resizable(true)
                    );
                table
                    .header(20.0, |mut header_row| {
                        header_row.col(|ui| { ui.strong("Joint"); });
                        header_row.col(|ui| { ui.strong("Positions"); });
                    })
                    .body(|mut body| {
                        for (joint_name, _, pos) in output.iter() {
                            body.row(18.0, |mut row| {
                                row.col(|ui| { ui.label(format!("{}", joint_name)); });
                                row.col(|ui| { ui.label(format!("{:?}", pos)); });
                            });
                        }
                    });
            }
            else {
                ui.label("No positions");
            }
        }

        if let Some(solver) = self.solvers.get_mut(&self.selected_solver) {
            match solver {
                IkSolver::Jacobian(s) => {
                    ui.label("Jacobian Solver Options:");
                    ui.horizontal(|ui| {
                        ui.label("Maximum Iterations: ");
                        ui.add(egui::DragValue::new(&mut s.num_max_try).max_decimals(0));
                    });
                    drag_value!(ui, "Maximum Distance Error: ", s.allowable_target_distance);
                    drag_value!(ui, "Maximum Angle Error (radians): ", s.allowable_target_angle);
                    drag_value!(ui, "Jacobian Multiplier: ", s.jacobian_multiplier);
                }
                IkSolver::Cyclic => {}
            }
        }

        let end_effector_btn =
            if self.end_effector.is_some() { ui.button("End Effector (Selected)") }
            else if self.end_picking_resp.is_some() { ui.button("End Effector (Selecting)") }
            else { ui.button("Select an End Effector") };

        if end_effector_btn.clicked() {
            self.end_effector = None;
            if let Some(resp) = self.end_picking_resp.take() {
                resp.cancel_request();
            }
            else {
                self.end_picking_resp = Some(
                    commands.send_picking_request(PickingRequest::new(end_picking_filter))
                );
            }
        }

        let target_pos_btn =
            if self.target_picking_resp.is_some() { ui.button("Target Position (Selecting)") }
            else { ui.button("Target Position") };
        if target_pos_btn.clicked() {
            if let Some(resp) = self.target_picking_resp.take() {
                resp.cancel_request();
            }
            else {
                self.target_picking_resp = Some(commands.send_picking_request(PickingRequest::new(target_picking_filter)))
            }
        }

        self.solve_clicked = ui.button("Solve").clicked();
        self.save_clicked = ui.button("Save").clicked();
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let Some(robot_entity) = self.robot else {
            return Ok(());
        };

        // Initializing IK window.
        self.output_clone = self.output_clone.take()
            .or_else(|| {
                self.solvers.insert(self.selected_solver.clone(), IkSolver::try_from(&self.selected_solver).unwrap());
                Some(IkOutput(vec![]))
            });

        if let Some(resp) = &self.end_picking_resp {
            if let Some(pick_data) = resp.take_response() {
                self.end_effector = Some(pick_data.event.entity);
                self.end_picking_resp = None;
            }
        }

        if let Some(resp) = &self.target_picking_resp {
            if let Some(pick_data) = resp.take_response() {
                self.target_pos = pick_data.event.pointer_event.hit.position.unwrap();
                self.target_picking_resp = None;
            }
        }

        if self.solve_clicked {
            let Some(end_effector) = self.end_effector else {
                return Err(Error::FailedOperation("Inverse Kinematics Solve".to_string(), "no end effector selected!".to_string()))
            };
            let end_node = res.kinematic_nodes
                .get(end_effector)
                .unwrap()
                .1;
            let end_name = end_node.joint().name.clone();
            let robot = res.robots.get(robot_entity).unwrap();
            let planner = self.planner.take()
                .or_else(|| {
                    let chain = k::Chain::from_nodes(robot.robot.chain.iter().cloned().collect());
                    JointPathPlannerBuilder::from_urdf_robot_with_base_dir(
                        robot.robot.urdf.clone(),
                        Some(robot.robot.urdf_path.parent().unwrap())
                    )
                        .collision_check_margin(0.01)
                        .reference_robot(Arc::new(chain))
                        .finalize()
                        .ok()
                })
                .unwrap();
            let solver = self.solvers.remove(&self.selected_solver)
                .or_else(|| IkSolver::try_from(&self.selected_solver).ok())
                .unwrap();
            let compound = ncollide3d::shape::Compound::new(vec![]);
            let mut ik_planner = JointPathPlannerWithIk::new(planner, solver);
            let target_pose = k::Isometry3::translation(self.target_pos.x, self.target_pos.y, self.target_pos.z);
            let positions = ik_planner
                .plan_with_ik(end_name.as_str(), &target_pose, &compound);
            if let Err(e) = positions {
                self.solvers.insert(self.selected_solver.clone(), ik_planner.ik_solver);
                return Err(
                    Error::FailedOperation("Inverse Kinematics Solve Failed".to_string(), format!("{:?}", e))
                );
            }
            let positions = positions.unwrap();

            let solved_chain = k::SerialChain::from_end(end_node);
            self.output_clone = Some(IkOutput(
                positions
                    .into_iter()
                    .map(|v| {
                        v
                            .into_iter()
                            .enumerate()
                            .map(|(idx, pos)| {
                                let node = solved_chain
                                    .iter()
                                    .nth(idx)
                                    .unwrap();
                                let entity = res.kinematic_nodes
                                    .iter()
                                    .find(|(_, node)| node.0.eq(node))
                                    .map(|(entity, _)| entity)
                                    .unwrap();
                                let joint = node.joint();
                                let pos = match joint.joint_type {
                                    JointType::Rotational { .. } => {[0f32, 0., 0., pos, 0., 0.]}
                                    JointType::Linear { .. } => {[pos, 0., 0., 0., 0., 0.]}
                                    _ => {[0., 0., 0., 0., 0., 0.]}
                                };
                                (
                                    joint.name.clone(),
                                    entity,
                                    pos
                                )
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect()
            ));
            self.planner = Some(ik_planner.path_planner);
            self.solvers.insert(self.selected_solver.clone(), ik_planner.ik_solver);
        }

        if self.save_clicked {
            self.save_clicked = false;
            res.commands.trigger(UtilityOutputEvent::<Self> {
                output: self.output_clone.take()
            });
            self.robot = None;
            self.end_effector = None;
            self.target_pos = Vec3::ZERO;
            self.should_close = true;
            self.target_picking_resp.take().map(|v| v.cancel_request());
            self.end_picking_resp.take().map(|v| v.cancel_request());
        }

        Ok(())
    }
}

impl GizmosUi for IkWindow {
    fn gizmos_ui(&mut self, resources: &mut GizmosUiResources) {
        if !self.open { return; }
        let cam_to_target = self.target_pos - resources.camera.transform.translation()
            .normalize()
            .mul(GIZMO_DISTANCE);
        resources.gizmos.sphere(
            self.target_pos,
            0.1,
            Color::linear_rgb(1., 1., 0.2)
        );
    }
}

impl_util_window!(IkWindow, Entity, IkOutput, robot, output, canceled);

pub enum IkSolver {
    Jacobian(k::JacobianIkSolver<Real>),
    Cyclic,
}

impl TryFrom<&String> for IkSolver {
    type Error = ();
    fn try_from(value: &String) -> Result<Self, Self::Error> {
        match value.as_str() {
            "Jacobian" => Ok(IkSolver::Jacobian(k::JacobianIkSolver::default())),
            "Cyclic" => todo!(),
            _ => Err(())
        }
    }
}

impl std::fmt::Display for IkSolver {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", match self {
            IkSolver::Jacobian(_) => {"Jacobian"}
            IkSolver::Cyclic => {"Cyclic"}
        })
    }
}

impl k::InverseKinematicsSolver<Real> for IkSolver {
    fn solve(&self, arm: &SerialChain<Real>, target_pose: &Isometry3<Real>) -> Result<(), k::Error> {
        self.solve_with_constraints(arm, target_pose, &Constraints::default())
    }

    fn solve_with_constraints(&self, arm: &SerialChain<Real>, target_pose: &Isometry3<Real>, constraints: &Constraints) -> Result<(), k::Error> {
        match self {
            IkSolver::Jacobian(s) => {s.solve_with_constraints(arm, target_pose, constraints)}
            IkSolver::Cyclic => todo!()
        }
    }
}

impl Default for IkSolver {
    fn default() -> Self {
        Self::Jacobian(k::JacobianIkSolver::default())
    }
}

#[derive(Clone)]
pub struct IkOutput(pub Vec<Vec<(String, Entity, [Real; SPATIAL_DIM])>>);

impl Deref for IkOutput {
    type Target = Vec<Vec<(String, Entity, [Real; SPATIAL_DIM])>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for IkOutput {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

fn end_picking_filter(res: &PickingFilterResources, event: &PickingEvent) -> bool {
    let Some(active_robot) = res.selections.active_robot else {
        return false;
    };
    if !(
        res.robot_parts.get(event.entity).is_ok_and(|v| v.0 == active_robot) &&
            res.kinematic_nodes.contains(event.entity)
    ) {
        return false;
    }
    true
}

fn target_picking_filter(_res: &PickingFilterResources, event: &PickingEvent) -> bool {
    event.pointer_event.hit.position.is_some()
}
