use crate::kinematics::ik::{ForwardAscentCyclic, ForwardDescentCyclic};
use crate::math::Real;
use bevy::prelude::Local;
use bevy::ptr::UnsafeCellDeref;
use bevy_egui::egui::ComboBox;
use bevy_egui::{egui, EguiContexts};
use bevy_rapier3d::prelude::WriteDefaultRapierContext;
use k::{InverseKinematicsSolver, SerialChain};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};
use std::cell::UnsafeCell;
use bevy_rapier3d::parry::math::{Isometry, Vector};

pub struct IKSandboxUI {
    pub kinematic_chain: Option<SerialChain<Real>>,
    pub solvers: Vec<Box<dyn InverseKinematicsSolver<Real> + Send>>,
    pub solver_names: Vec<String>,
}

impl Default for IKSandboxUI {
    fn default() -> Self {
        Self {
            kinematic_chain: None,
            solvers: vec![
                Box::new(ForwardAscentCyclic::default()),
                Box::new(ForwardDescentCyclic::default())
            ],
            solver_names: vec![
                "Forward Ascent Cyclic".to_string(),
                "Forward Descent Cyclic".to_string(),
            ],
        }
    }
}

pub struct IKSandboxUIState {
    pub selected_solver_idx: usize,
    pub urdf_loader_options: UrdfLoaderOptions
}

impl Default for IKSandboxUIState {
    fn default() -> Self {
        Self {
            selected_solver_idx: 0,
            urdf_loader_options: UrdfLoaderOptions {
                create_colliders_from_visual_shapes: true,
                create_colliders_from_collision_shapes: false,
                make_roots_fixed: true,
                // Z-up to Y-up.
                shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
                ..Default::default()
            }
        }
    }
}

pub fn ik_sandbox_ui(
    mut ctxs: EguiContexts,
    mut ui_data: Local<IKSandboxUI>,
    mut ui_state: Local<IKSandboxUIState>,
    mut rapier_ctx: WriteDefaultRapierContext,
) {
    egui::Window::new("Robot Sandbox").show(
        ctxs.ctx_mut(), |ui| {

            //Robot importing
            ui.collapsing("Import Robot", |ui| {
                let _checkbox = ui.checkbox(
                    &mut ui_state.urdf_loader_options.make_roots_fixed,
                    "Make roots fixed"
                );
                let button = ui.button("Import URDF file");
                if button.clicked() { //import urdf robot via file dialog
                    let dialog = rfd::FileDialog::new()
                        .add_filter("Robot Description", &["urdf"])
                        .pick_file();
                    if let Some(path) = dialog {
                        let (robot, _) = UrdfRobot::from_file(path, ui_state.urdf_loader_options.clone(), None).unwrap();
                        unsafe {
                            let mut unsafe_ctx = UnsafeCell::new(rapier_ctx);
                            //TODO: use robot handles to make the rapier entities.
                            let _robot_handles = robot.insert_using_multibody_joints(
                                &mut unsafe_ctx.deref_mut().bodies,
                                &mut unsafe_ctx.deref_mut().colliders,
                                &mut unsafe_ctx.deref_mut().multibody_joints,
                                UrdfMultibodyOptions::DISABLE_SELF_CONTACTS
                            );
                        }
                    }
                }
            });

            //Inverse Kinematics
            ui.collapsing("Inverse Kinematics", |ui| {
                let solver_idx = ui_state.selected_solver_idx;
                let selected_solver_name =
                    if let Some(solver) = ui_data.solvers.get(solver_idx) {
                        ui_data.solver_names.get(solver_idx).unwrap()
                    }
                    else { &"-".to_string() };
                ComboBox::from_label("Solver type")
                    .selected_text(selected_solver_name)
                    .show_ui(ui, |ui| {
                        for (i, solver_name) in ui_data.solver_names.iter().enumerate() {
                            ui.selectable_value(&mut ui_state.selected_solver_idx, i, solver_name);
                        }
                    });

                ui.horizontal(|ui| {
                    ui.label("Chain:");
                    let _ = ui.button(
                        if ui_data.kinematic_chain.is_some() {"Chain" }
                        else { "N/A" }
                    );
                });
            });
        }
    );
}
