use crate::general::{ImportEvent, OtherImportOptions};
use crate::prelude::*;
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI, TOOLTIP_LAYER};
use bevy::prelude::{default, Resource};
use bevy_egui::egui;
use bevy_egui::egui::{ComboBox, Context, Ui};
use bevy_rapier3d::na::{Isometry3, Vector3};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use std::path::PathBuf;

#[derive(Resource, Default)]
pub struct Import {
    pub window: ImportWindow,
}

impl View for Import {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        let button = ui.button("Import URDF");
        if button.clicked() {
            self.window.is_open = true;
        }
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> crate::prelude::Result<()> {
        ImportWindow::functionality(resources, events)?;
        Ok(())
    }
}

pub struct ImportWindow {
    pub is_open: bool,
    pub import_clicked: bool,
    pub mb_loader_options: UrdfMultibodyOptions,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub other_options: OtherImportOptions,
    pub import_joint_type: ImportJointType,
    pub import_file_path: String,
    pub mesh_dir: String,
}

impl ImportWindow {
    const MESH_DIR_TOOLTIP: &'static str = "mesh_dir_tooltip";
}

impl Default for ImportWindow {
    fn default() -> Self {
        Self {
            is_open: default(),
            import_clicked: default(),
            mb_loader_options: UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
            urdf_loader_options: UrdfLoaderOptions {
                create_colliders_from_visual_shapes: true,
                create_colliders_from_collision_shapes: false,
                make_roots_fixed: true,
                // Z-up to Y-up.
                shift: Isometry3::rotation(Vector3::x() * std::f32::consts::FRAC_PI_2),
                ..default()
            },
            other_options: default(),
            import_joint_type: default(),
            import_file_path: default(),
            mesh_dir: default(),
        }
    }
}

impl View for ImportWindow {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
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
                let mesh_dir_hovered = ui.add(egui::TextEdit::singleline(&mut self.mesh_dir)).hovered();
                if mesh_dir_hovered {
                    egui::show_tooltip_at_pointer(
                        ui.ctx(),
                        egui::LayerId::new(egui::Order::Tooltip, egui::Id::new(TOOLTIP_LAYER)),
                        egui::Id::new(Self::MESH_DIR_TOOLTIP),
                        |ui| ui.label(
                            "The directory where robot meshes are found. Mesh file paths in \
                                the URDF file are directly appended to this directory.\n\
                                Example: \n\
                                    mesh directory: /meshes/\n \
                                    mesh file: ./arm.stl\n \
                                    resulting mesh file path: /meshes/arm.stl"
                        )
                    );
                }
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
        self.import_clicked = ui.button("Import URDF").clicked();

        // "Make roots fixed" checkbox
        ui.checkbox(
            &mut self.urdf_loader_options.make_roots_fixed,
            "Make roots fixed",
        );
        ui.checkbox(
            &mut self.other_options.all_joints_have_motors,
            "All joints have motors"
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
    }
    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> Result<()> {
        let window = &mut resources.general_tab.import.window;
        if !window.import_clicked { return Ok(()); }
        // Close window upon clicking "Import URDF"
        window.is_open = false;

        let file_path = PathBuf::from(&window.import_file_path);
        let mesh_dir = PathBuf::from(&window.mesh_dir);
        if !file_path.is_file() {
            return Err(Error::FailedOperation("Importing Failed: file path isn't a file.".to_string()))
        }
        events.import_events.send(ImportEvent {
            urdf_loader_options: window.urdf_loader_options.clone(),
            import_joint_type: window.import_joint_type,
            mb_loader_options: window.mb_loader_options.clone(),
            other_options: window.other_options.clone(),
            file_path,
            mesh_dir,
        });

        Ok(())
    }
}

impl WindowUI for ImportWindow {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        let mut is_open = self.is_open;
        egui::Window::new("Import URDF")
            .open(&mut is_open)
            .show(egui_ctx, |ui| {
                self.ui(ui, ui_assets);
            });
        // Avoid borrow checker error
        self.is_open = is_open;
    }
}

#[derive(Default, PartialEq, Copy, Clone)]
pub enum ImportJointType {
    #[default]
    Multibody,
    Impulse,
}