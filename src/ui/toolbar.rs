use bevy::color::Color;
use bevy::math::Vec3;
use bevy::prelude::{Gizmos, GlobalTransform, Query, Resource};
use bevy_egui::egui::Ui;
use nalgebra::{UnitVector3, Vector3};
use crate::math::Real;
use crate::ui::SelectedEntities;

/// Contains all the data needed for the toolbar UI
#[derive(Default, Resource)]
pub struct Toolbar {
    pub selected_tool: Tool,
    pub pointer_usage_state: PointerUsageState,
    pub global_coords: bool
}

pub enum Tool {
    Grab {
        grabbed_axis: Option<UnitVector3<Real>>,
        plane_normal: Option<UnitVector3<Real>>,
        init_pointer_pos: Option<Vector3<Real>>,
        curr_pointer_pos: Option<Vector3<Real>>,
    },
    Rotate {
        grabbed_disc_normal: Option<UnitVector3<Real>>,
        init_pointer_pos: Option<Vector3<Real>>,
        curr_pointer_pos: Option<Vector3<Real>>,
    },
}

impl Default for Tool {
    fn default() -> Self {
        Self::Grab {
            grabbed_axis: None,
            plane_normal: None,
            init_pointer_pos: None,
            curr_pointer_pos: None,
        }
    }
}

/// An enum that tells how the pointer using the toolbar
#[derive(Default)]
pub enum PointerUsageState {
    #[default]
    UsingTool,
    NotUsed
}

/// The function that draws the toolbar UI and any gizmos associated with the current tool
pub fn toolbar_ui(
    ui: &mut Ui,
    toolbar: &mut Toolbar,
    selected_entities: &mut SelectedEntities,
    transform_q: &Query<&GlobalTransform>,
    gizmos: &mut Gizmos
) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            let mut grab = matches!(toolbar.selected_tool, Tool::Grab);
            let grab_clicked = ui.toggle_value(&mut grab, "Grab").clicked();
            let mut rotate = matches!(toolbar.selected_tool, Tool::Rotate);
            let rotate_clicked = ui.toggle_value(&mut rotate, "Rotate").clicked();

            if grab && grab_clicked { toolbar.selected_tool = Tool::Grab; }
            else if rotate && rotate_clicked { toolbar.selected_tool = Tool::Rotate; }
        });
        ui.label("Coordinate Space");
        ui.horizontal(|ui| {
            let mut global = toolbar.global_coords;
            let global_clicked = ui.toggle_value(&mut global, "Global").clicked();
            let mut local = !toolbar.global_coords;
            let local_clicked = ui.toggle_value(&mut local, "Local").clicked();

            if global && global_clicked { toolbar.global_coords = true; }
            else if local && local_clicked { toolbar.global_coords = false; }
        })
    });

    // Drawing gizmos
    if let Some(robot) = selected_entities.active_robot {
        let robot_transform = transform_q.get(robot).unwrap();
        let pos = robot_transform.translation();
        let rot = robot_transform.rotation();
        match toolbar.selected_tool {
            Tool::Grab => {
                let (x_axis, y_axis, z_axis) =
                    if toolbar.global_coords {
                        (Vec3::X, Vec3::Y, Vec3::Z)
                    }
                    else {
                        (rot * Vec3::X, rot * Vec3::Y, rot * Vec3::Z)
                    };
                gizmos.arrow(pos, pos + x_axis, Color::linear_rgb(1., 0., 0.));
                gizmos.arrow(pos, pos + y_axis, Color::linear_rgb(0., 1., 0.));
                gizmos.arrow(pos, pos + z_axis, Color::linear_rgb(0., 0., 1.));
            },
            Tool::Rotate => {}
        }
    }
}
