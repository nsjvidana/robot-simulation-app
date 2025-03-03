use std::fmt::Display;
use bevy::core::Name;
use bevy::prelude::{Commands, Resource};
use bevy::utils::default;
use bevy_egui::egui;
use bevy_egui::egui::{ComboBox, Ui};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use crate::robot::Robot;
use crate::{finish_ui_section_vertical, transparent_button};

#[derive(Resource)]
pub struct RobotImporting {
    pub mb_loader_options: UrdfMultibodyOptions,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub import_joint_type: ImportJointType,
}

impl Default for RobotImporting {
    fn default() -> Self {
        Self {
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
        }
    }
}

#[derive(Default, PartialEq)]
pub enum ImportJointType {
    #[default]
    Impulse,
    Multibody
}

pub fn import_ui(
    commands: &mut Commands,
    ui: &mut Ui,
    importing: &mut RobotImporting,
) -> (egui::Rect, &'static str) {
    let resp = ui.vertical(|ui| {
        // "Make roots fixed" checkbox
        let _checkbox = ui.checkbox(
            &mut importing.urdf_loader_options.make_roots_fixed,
            "Make roots fixed"
        );
        // Robot joint type
        ComboBox::from_label("Robot joint type")
            .selected_text(match importing.import_joint_type {
                ImportJointType::Impulse => "Impulse", ImportJointType::Multibody => "Multibody"
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut importing.import_joint_type, ImportJointType::Impulse, "Impulse");
                ui.selectable_value(&mut importing.import_joint_type, ImportJointType::Multibody, "Multibody");
            });

        // Import urdf robot via file dialog
        let button = ui.button("Import URDF");
        if button.clicked() {
            let dialog = rfd::FileDialog::new()
                .add_filter("Robot Description", &["urdf", "URDF"])
                .pick_file();
            if let Some(path) = dialog {
                let robot_name = path.file_stem().unwrap()
                    .to_str().unwrap().to_string();
                let (robot, _) = rapier3d_urdf::UrdfRobot::from_file(
                    &path,
                    importing.urdf_loader_options.clone(),
                    None
                ).unwrap();
                let robot_cmp = Robot::new(robot, path);
                commands.spawn((
                    match importing.import_joint_type {
                        ImportJointType::Impulse => robot_cmp.with_impulse_joints(),
                        ImportJointType::Multibody => robot_cmp.with_multibody_joints(importing.mb_loader_options),
                    },
                    Name::new(robot_name)
                ));
            }
        }
        finish_ui_section_vertical!(ui, "Importing")
    });
    resp.inner
}
