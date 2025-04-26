use std::path::PathBuf;
use bevy::prelude::Commands;
use bevy_egui::egui;
use bevy_rapier3d::na::{Isometry3, Vector3};
use bevy_rapier3d::prelude::VHACDParameters;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use crate::functionality::import::{ImportEvent, RobotJointType};
use crate::ui::View;

#[derive(Default)]
pub struct ImportUi {
    window: ImportWindow,
}

impl View for ImportUi {
    fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Commands) {
        if ui.button("Import URDF").clicked() {
            self.window.open = true;
        }

        if self.window.open {
            let mut open = self.window.open;
            egui::Window::new("Import URDF")
                .open(&mut open)
                .show(ui.ctx(), |ui| {
                    self.window.ui(ui, commands);
                });
            self.window.open = open;
        }
    }

    fn view_name(&self) -> &'static str {
        "Import"
    }
}

pub struct ImportWindow {
    open: bool,
    urdf_path: String,
    mesh_dir: String,
    urdf_loader_options: UrdfLoaderOptions,
    mb_options: UrdfMultibodyOptions,
    robot_joint_type: RobotJointType,
    approximate_collisions: bool,
    convex_decomp_options: VHACDParameters,
    all_joints_have_motors: bool,
}

impl Default for ImportWindow {
    fn default() -> Self {
        Self {
            open: false,
            urdf_path: Default::default(),
            mesh_dir: Default::default(),
            urdf_loader_options: UrdfLoaderOptions {
                create_colliders_from_visual_shapes: true,
                create_colliders_from_collision_shapes: false,
                make_roots_fixed: true,
                // Z-up to Y-up.
                shift: Isometry3::rotation(Vector3::x() * -std::f32::consts::FRAC_PI_2),
                ..Default::default()
            },
            mb_options: UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
            robot_joint_type: Default::default(),
            approximate_collisions: true,
            convex_decomp_options: VHACDParameters {
                resolution: 32,
                concavity: 0.01,
                max_convex_hulls: 128,
                ..Default::default()
            },
            all_joints_have_motors: false,
        }
    }
}

impl View for ImportWindow {
    fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Commands) {
        egui::Grid::new("import_grid")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("URDF Path:");
                ui.text_edit_singleline(&mut self.urdf_path);
                let browse_urdf = ui.button("Browse").clicked();
                ui.end_row();

                ui.label("Mesh Directory:");
                ui.text_edit_singleline(&mut self.mesh_dir);
                let browse_mesh_dir = ui.button("Browse").clicked();
                ui.end_row();

                if browse_urdf {
                    if let Some(file) = rfd::FileDialog::new()
                        .add_filter("Robot Description", &["URDF", "urdf"])
                        .pick_file()
                    {
                        self.urdf_path = file.display().to_string();
                    };
                }

                if browse_mesh_dir {
                    if let Some(folder) = rfd::FileDialog::new()
                        .pick_folder()
                    {
                        self.mesh_dir = folder.display().to_string();
                    };
                }
            });

        ui.checkbox(&mut self.urdf_loader_options.make_roots_fixed, "Make robot roots fixed");
        egui::ComboBox::from_label("Joint Solver Type")
            .selected_text(self.robot_joint_type.to_string())
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.robot_joint_type,
                    RobotJointType::Multibody(self.mb_options),
                    "Multibody"
                );
                ui.selectable_value(
                    &mut self.robot_joint_type,
                    RobotJointType::Impulse,
                    "Impulse"
                );
            });
        let mut collide_with_self = self.mb_options != UrdfMultibodyOptions::DISABLE_SELF_CONTACTS;
        if ui.checkbox(&mut collide_with_self, "Collide with self").changed() {
            self.mb_options ^= UrdfMultibodyOptions::DISABLE_SELF_CONTACTS;
        }
        ui.checkbox(&mut self.approximate_collisions, "Approximate collisions");

        if self.approximate_collisions {
            ui.indent("coll_approx_params_indent", |ui| {
                ui.collapsing("Collision Approximation Parameters", |ui| {
                    let opts = &mut self.convex_decomp_options;
                    ui.add(egui::DragValue::new(&mut opts.resolution).max_decimals(0));
                    ui.add(egui::DragValue::new(&mut opts.max_convex_hulls).max_decimals(0));
                });
            });
        }

        ui.checkbox(&mut self.all_joints_have_motors, "All joints have motors");

        if ui.button("Import").clicked() {
            if let RobotJointType::Multibody(opt) = &mut self.robot_joint_type {
                *opt = self.mb_options.clone();
            }
            commands.send_event(ImportEvent {
                urdf_path: PathBuf::from(self.urdf_path.clone()),
                mesh_dir: PathBuf::from(self.mesh_dir.clone()),
                urdf_loader_options: self.urdf_loader_options.clone(),
                robot_joint_type: self.robot_joint_type,
                approximate_collisions: self.approximate_collisions,
                convex_decomp_options: self.convex_decomp_options.clone(),
                all_joints_have_motors: self.all_joints_have_motors,
            });
            self.open = false;
        }
    }
}
