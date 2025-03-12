use crate::kinematics::ik::{ForwardDescentCyclic, KinematicNode};
use crate::math::Real;
use crate::ui::ribbon::{finish_ribbon_tab, finish_ui_section_vertical};
use bevy_egui::egui;
use bevy_egui::egui::{Align, Layout, Separator, Ui, UiBuilder};
use openrr_planner::JointPathPlanner;
use std::collections::HashMap;
use bevy::prelude::{Commands, Gizmos, Query};
use bevy_rapier3d::prelude::{RapierContext, RapierImpulseJointHandle, RapierMultibodyJointHandle, TypedJoint};
use bevy_salva3d::bevy_rapier;
use k::{JointType, NodeBuilder};
use crate::robot::{RapierRobotHandles, Robot, RobotJointType, RobotPart};
use crate::ui::{EntitySelectionMode, SelectedEntities, UiGizmoGroup};

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
    selected_entities: &mut SelectedEntities,
    motion_planning: &mut MotionPlanning,
    ribbon_height: f32,
) {
    let mut rects = Vec::new();
    ui.horizontal(|ui| {
        egui::Grid::new("planning_ribbon")
            .num_columns(2)
            .show(ui, |ui| {
                rects.push(
                    ik_ui(ui, selected_entities, motion_planning, ribbon_height)
                );
                // rects.push(
                //     ik_ui(ui, selected_entities, motion_planning, ribbon_height)
                // );
            });
    });
    finish_ribbon_tab!(ui, rects);
}

pub fn ik_ui(
    ui: &mut Ui,
    selected_ents: &mut SelectedEntities,
    motion_planning: &mut MotionPlanning,
    ribbon_height: f32
) -> (egui::Rect, &'static str) {
    ui.horizontal(|ui| {
        let ret = ui.vertical(|ui| {
            let ik_btn = ui.button("Inverse Kinematics");
            if ik_btn.clicked() {
                if selected_ents.active_robot.is_some() {
                    if !motion_planning.ik_window.open {
                        selected_ents.selected_joints.clear();
                        selected_ents.selection_mode = EntitySelectionMode::SelectRobotJointsLocal;
                    }
                    motion_planning.ik_window.open = true;
                }
                else {
                    // TODO: make a way for this msg to show up in the app
                    bevy::log::error!("Failed IK: no active robot!");
                }
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

pub fn ik_window_function(
    commands: &mut Commands,
    selected_ents: &mut SelectedEntities,
    motion_planning: &mut MotionPlanning,
    robot_q: &Query<(&Robot, &RapierRobotHandles)>,
    robot_part_q: &Query<&RobotPart>,
    joint_q: &Query<(Option<&RapierImpulseJointHandle>, Option<&RapierMultibodyJointHandle>)>,
    rapier_ctx: &RapierContext,
    gizmos: &mut Gizmos<UiGizmoGroup>
) {
    if !motion_planning.ik_window.open { return; }

    let ik_window = &mut motion_planning.ik_window;
    let active_robot = selected_ents.active_robot.unwrap();

    // If "Create IK Chain" button is clicked
    if ik_window.create_ik_chain {
        for ent in selected_ents.selected_joints.iter().copied() {
            if !robot_part_q.contains(ent) { continue; }
            let robot_ent = robot_part_q.get(ent).unwrap().0;
            if robot_ent != active_robot { continue; }
            let joint_handle_index = joint_q.get(ent)
                .map(|(imp, mb)|
                    if let Some(imp) = imp { Some(imp.0.0) }
                    else if let Some(mb) = mb { Some(mb.0.0) }
                    else { None }
                )
                .unwrap();
            let (robot, rapier_handles) = robot_q.get(robot_ent).expect("Robot doesn't have robot/handles component!");

            let joint_idx = rapier_handles.joints
                .iter()
                .position(|h| h.joint.map_or_else(
                    || joint_handle_index.is_none(),
                    |j| joint_handle_index.is_some_and(|v| v == j)
                ));
            if let Some(joint_idx) = joint_idx {
                let urdf_joint = robot.urdf.joints.get(joint_idx)
                    .expect("Joint indices don't match with robot!");
                let node = k::Node::<Real>::new(k::Joint::from(urdf_joint));
            }

            // TODO: add KinematicNode component to ent using robot urdf data
            // commands.entity(ent)
            //     .insert(KinematicNode {
            //         node: NodeBuilder::new()
            //             .joint_type(
            //                 if joint.as_fixed().is_some() { JointType::Fixed }
            //                 else if joint.as_revolute().is_some() {
            //                     JointType::Rotational { axis: joint.local_axis1() }
            //                 }
            //                 else if joint.locked_axes.is_empty() {
            //                     JointType::
            //                 }
            //                 else if joint.as_prismatic().is_some() {
            //                     JointType::Linear { axis: joint.local_axis1() }
            //                 }
            //             )
            //     });
        }
        ik_window.create_ik_chain = false;
    }
}
