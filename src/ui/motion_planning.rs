use crate::kinematics::ik::{ForwardDescentCyclic, KinematicNode};
use crate::prelude::*;
use crate::ui::{ribbon::{finish_ribbon_tab, finish_ui_section_vertical}, EntitySelectionMode, SelectedEntities, UiGizmoGroup, UiResources};
use bevy::prelude::{Commands, Gizmos, GlobalTransform, Or, Query, With};
use bevy_egui::egui;
use bevy_egui::egui::{Align, Layout, Separator, Ui, UiBuilder};
use bevy_rapier3d::dynamics::{ImpulseJoint, MultibodyJoint};
use bevy_rapier3d::prelude::{
    RapierContext, RapierImpulseJointHandle, RapierMultibodyJointHandle, TypedJoint,
};
use bevy_salva3d::bevy_rapier;
use k::{JointType, NodeBuilder};
use openrr_planner::JointPathPlanner;
use std::collections::HashMap;

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

pub fn motion_planning_ui(
    ui: &mut Ui,
    selected_entities: &mut SelectedEntities,
    ui_resources: &mut UiResources,
    ribbon_height: f32,
) -> Result<()> {
    let mut rects = Vec::new();
    ui.horizontal(|ui| -> Result<()> {
        egui::Grid::new("planning_ribbon")
            .num_columns(2)
            .show(ui, |ui| -> Result<_> {
                rects.push(ik_ui(ui, selected_entities, &mut ui_resources.motion_planning, ribbon_height)?);
                Ok(())
            }).inner?;
        Ok(())
    }).inner?;
    finish_ribbon_tab!(ui, rects);
    Ok(())
}

pub fn ik_ui(
    ui: &mut Ui,
    selected_ents: &mut SelectedEntities,
    motion_planning: &mut MotionPlanning,
    ribbon_height: f32,
) -> Result<(egui::Rect, &'static str)> {
    ui.horizontal(|ui| -> Result<_> {
        let ret = ui
            .vertical(|ui| -> Result<_> {
                let ik_btn = ui.button("Inverse Kinematics");
                if ik_btn.clicked() {
                    if selected_ents.active_robot.is_some() {
                        if !motion_planning.ik_window.open {
                            selected_ents.selected_joints.clear();
                            selected_ents.selection_mode =
                                EntitySelectionMode::SelectSerialChainLocal {
                                    selection_preview_joints: vec![],
                                    selected_joints: vec![],
                                    selection_root: None,
                                    hovered_joint: None,
                                };
                        }
                        motion_planning.ik_window.open = true;
                    } else {
                        return Err(Error::FailedOperation("IK failed: no selected robot!".to_string()));
                    }
                }
                Ok(finish_ui_section_vertical!(ui, "Inverse Kinematics"))
            }).inner?;
        ui.add(Separator::default().grow(ribbon_height).spacing(0.));
        Ok(ret)
    }).inner
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

pub fn ik_window_function(
    ui_resources: &mut UiResources,
    commands: &mut Commands,
    selected_ents: &mut SelectedEntities,
    robot_q: &Query<(&Robot, &RapierRobotHandles)>,
    joint_q: &Query<
        (
            Option<&RapierImpulseJointHandle>,
            Option<&RapierMultibodyJointHandle>,
            Option<&KinematicNode>,
        ),
        Or<(
            With<RapierImpulseJointHandle>,
            With<RapierMultibodyJointHandle>,
        )>,
    >,
) -> Result<()> {
    let motion_planning = &mut ui_resources.motion_planning;
    if !motion_planning.ik_window.open {
        return Ok(());
    }

    let ik_window = &mut motion_planning.ik_window;
    let active_robot = selected_ents.active_robot.unwrap();
    let (robot, rapier_handles) = robot_q
        .get(active_robot)
        .map_err(|_| Error::MissingComponent {
            entity_name: active_robot.to_string(),
            component_name: "(&Robot, &RapierRobotHandles)".to_string(),
        })?;

    let selected_joints = {
        match selected_ents.selection_mode {
            EntitySelectionMode::SelectSerialChainLocal {
                ref selected_joints,
                ..
            } => selected_joints,
            _ => return Ok(()),
        }
    };

    // If "Create IK Chain" button is clicked
    if ik_window.create_ik_chain {
        for ent in selected_joints.iter().copied() {
            let (impulse, mbj, k_node) = joint_q.get(ent).unwrap();
            let joint_handle_index = if let Some(imp) = impulse {
                imp.0 .0
            } else if let Some(mbj) = mbj {
                mbj.0 .0
            } else {
                continue;
            };

            if k_node.is_none() {
                let joint_idx = rapier_handles
                    .joints
                    .iter()
                    .position(|h| h.joint.is_some() && h.joint.unwrap() == joint_handle_index);
                if let Some(joint_idx) = joint_idx {
                    let urdf_joint = robot
                        .urdf
                        .joints
                        .get(joint_idx)
                        .expect("Joint indices don't match with robot!");
                    let node = k::Node::<Real>::new(k::Joint::from(urdf_joint));
                    println!("Adding kinematic node to joint {ent}");
                    commands.entity(ent).insert(KinematicNode(node));
                }
            } else {
                // TODO: add joint's kinematic node to robot kinematics component.
            }
        }
        ik_window.create_ik_chain = false;
    }
    Ok(())
}
