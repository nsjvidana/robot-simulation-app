use crate::prelude::*;
use crate::ui::ribbon::finish_ui_section_vertical;
use bevy::core::Name;
use bevy::prelude::{Commands, Resource};
use bevy::utils::default;
use bevy_egui::egui;
use bevy_egui::egui::{ComboBox, Ui};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use std::fmt::Display;
use std::path::PathBuf;
use crate::ui::{UiEvents, UiResources, View};

#[derive(Resource)]
pub struct RobotImporting {
    pub window_is_open: bool,
    pub import_triggered: bool,
    pub mb_loader_options: UrdfMultibodyOptions,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub import_joint_type: ImportJointType,
    pub import_file_path: String,
    pub mesh_dir: String,
}

impl RobotImporting {
    pub fn functionality(&self, commands: &mut Commands) -> Result<()> {
        // Import urdf robot via file dialog
        if self.import_triggered {
            let path = PathBuf::from(&self.import_file_path);
            let robot_name = path.file_stem().unwrap().to_str().unwrap().to_string();
            let robot_urdf = urdf_rs::read_file(&path)
                .map_err(|e| Error::Urdf {
                    error: e.to_string(),
                    robot_name: "<Unknown Robot>".to_string(),
                })?;

            let mesh_dir = PathBuf::from(&self.mesh_dir);
            let mesh_dir =
                if mesh_dir.exists() && mesh_dir.is_dir() { Some(mesh_dir) }
                else { None };
            let robot_cmp = Robot::new(robot_urdf, path.clone(), mesh_dir);
            commands.spawn((
                match self.import_joint_type {
                    ImportJointType::Impulse => robot_cmp.with_impulse_joints(),
                    ImportJointType::Multibody => {
                        robot_cmp.with_multibody_joints(self.mb_loader_options)
                    }
                },
                Name::new(robot_name),
            ));
        }
        Ok(())
    }
}

impl Default for RobotImporting {
    fn default() -> Self {
        Self {
            window_is_open: false,
            import_triggered: false,
            mb_loader_options: default(),
            urdf_loader_options: UrdfLoaderOptions {
                create_colliders_from_visual_shapes: true,
                create_colliders_from_collision_shapes: false,
                make_roots_fixed: true,
                // Z-up to Y-up.
                shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
                ..default()
            },
            import_joint_type: default(),
            import_file_path: default(),
            mesh_dir: default(),
        }
    }
}

impl View for RobotImporting {
    fn ui(&mut self, ui: &mut Ui) {
        ui.vertical(|ui| {
            // File path
            egui::Grid::new("Import IO")
                .num_columns(2)
                .show(ui, |ui| {
                    ui.label("Import Path:");
                    ui.add(egui::TextEdit::singleline(&mut self.import_file_path));
                    let import_browse = ui.button("Browse");
                    if import_browse.clicked() {
                        let dialog = rfd::FileDialog::new()
                            .add_filter("Robot Description", &["urdf", "URDF"])
                            .pick_file();
                        if let Some(path) = dialog {
                            self.import_file_path = path.display().to_string();
                        }
                    }
                    ui.end_row();

                    ui.label("Mesh Directory (optional):");
                    ui.add(egui::TextEdit::singleline(&mut self.mesh_dir));
                    let mesh_dir_browse = ui.button("Browse");
                    if mesh_dir_browse.clicked() {
                        let dialog = rfd::FileDialog::new()
                            .pick_folder();
                        if let Some(path) = dialog {
                            self.mesh_dir = path.display().to_string();
                        }
                    }
                    ui.end_row();
                });

            // Import urdf
            self.import_triggered = ui.button("Import URDF").clicked();

            // "Make roots fixed" checkbox
            let _checkbox = ui.checkbox(
                &mut self.urdf_loader_options.make_roots_fixed,
                "Make roots fixed",
            );

            // Robot joint type
            ComboBox::from_label("Robot joint type")
                .selected_text(match self.import_joint_type {
                    ImportJointType::Impulse => "Impulse",
                    ImportJointType::Multibody => "Multibody",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.import_joint_type,
                        ImportJointType::Impulse,
                        "Impulse",
                    );
                    ui.selectable_value(
                        &mut self.import_joint_type,
                        ImportJointType::Multibody,
                        "Multibody",
                    );
                });
        });
    }
    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> Result<()> {
        todo!()
    }
}

#[derive(Default, PartialEq)]
pub enum ImportJointType {
    #[default]
    Multibody,
    Impulse,
}

pub fn import_ui(
    ui: &mut Ui,
    ui_resources: &mut UiResources,
) -> (egui::Rect, &'static str) {
    ui.vertical(|ui| {
        let button = ui.button("Import URDF");
        if button.clicked() {
            ui_resources.import.window_is_open = true;
        }
        finish_ui_section_vertical!(ui, "Importing")
    }).inner
}

pub fn import_window(
    egui_ctx: &mut egui::Context,
    importing: &mut RobotImporting,
) {
    let mut is_open = importing.window_is_open;
    egui::Window::new("Import URDF")
        .open(&mut is_open)
        .show(egui_ctx, |ui| {
            importing.ui(ui);
        });
    // Avoid borrow checker
    importing.window_is_open = is_open;
}
