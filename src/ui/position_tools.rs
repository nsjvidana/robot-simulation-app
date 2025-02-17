use bevy::color::Color;
use bevy::math::Vec3;
use bevy::prelude::{Gizmos, GlobalTransform, Query, Resource};
use bevy::utils::default;
use bevy_egui::egui::Ui;
use bevy_rapier3d::parry::shape::{Cuboid, Cylinder, };
use bevy_rapier3d::parry::query::RayCast;
use bevy_rapier3d::rapier::prelude::Ray;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use crate::math::Real;
use crate::ui::{EntitySelectionMode, PointerUsageState, SceneWindowData, SelectedEntities, UiGizmoGroup};

/// Contains all the data needed for the Posiion section of the Ribbon
#[derive(Resource)]
pub struct PositionTools {
    pub selected_tool: PositionTool,
    pub gizmos_origin: Option<Vec3>,
    pub local_coords: bool,

    grab_shape: Cuboid,
    rotate_shape_outer: Cylinder,
    rotate_shape_inner: Cylinder,
}

impl Default for PositionTools {
    fn default() -> Self {
        Self {
            selected_tool: default(),
            gizmos_origin: None,
            local_coords: default(),

            grab_shape: Cuboid::new(Vector3::new(0.6, 0.05, 0.05)),
            rotate_shape_outer: Cylinder::new(0.05, 1.),
            rotate_shape_inner: Cylinder::new(0.05, 0.8),
        }
    }
}

pub enum PositionTool {
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

impl Default for PositionTool {
    fn default() -> Self {
        Self::Grab {
            grabbed_axis: None,
            plane_normal: None,
            init_pointer_pos: None,
            curr_pointer_pos: None,
        }
    }
}

/// The function that draws the Position UI and any gizmos associated with the current tool
pub fn position_tools_ui(
    ui: &mut Ui,
    position_tools: &mut PositionTools,
    selected_entities: &mut SelectedEntities,
    transform_q: &Query<&mut GlobalTransform>,
    scene_window_data: &SceneWindowData,
    gizmos: &mut Gizmos<UiGizmoGroup>
) {
    // Ribbon UI
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            let mut grab = matches!(position_tools.selected_tool, PositionTool::Grab { .. });
            let grab_clicked = ui.toggle_value(&mut grab, "Grab").clicked();
            let mut rotate = matches!(position_tools.selected_tool, PositionTool::Rotate { .. });
            let rotate_clicked = ui.toggle_value(&mut rotate, "Rotate").clicked();

            if grab && grab_clicked {
                position_tools.selected_tool = PositionTool::Grab {
                    grabbed_axis: None,
                    plane_normal: None,
                    init_pointer_pos: None,
                    curr_pointer_pos: None,
                }
            }
            else if rotate && rotate_clicked {
                position_tools.selected_tool = PositionTool::Rotate {
                    grabbed_disc_normal: None,
                    init_pointer_pos: None,
                    curr_pointer_pos: None,
                }
            }
        });
        ui.label("Coordinate Space");
        ui.horizontal(|ui| {
            let mut global = !position_tools.local_coords;
            let global_clicked = ui.toggle_value(&mut global, "Global").clicked();
            let mut local = position_tools.local_coords;
            let local_clicked = ui.toggle_value(&mut local, "Local").clicked();

            if global && global_clicked { position_tools.local_coords = false; }
            else if local && local_clicked { position_tools.local_coords = true; }
        })
    });

    // Gizmos UI
    if let Some(robot) = selected_entities.active_robot {
        let robot_transform = transform_q.get(robot).unwrap();
        let robot_pos = robot_transform.translation();
        let robot_rot = robot_transform.rotation();
        let cam_pos = scene_window_data.camera_transform.translation();
        match position_tools.selected_tool {
            PositionTool::Grab { .. } => {
                let (x_axis, y_axis, z_axis) =
                    if position_tools.local_coords {
                        (robot_rot * Vec3::X, robot_rot * Vec3::Y, robot_rot * Vec3::Z)
                    }
                    else {
                        (Vec3::X, Vec3::Y, Vec3::Z)
                    };
                let cam_to_robot = robot_pos - cam_pos;
                let orig = cam_pos + (cam_to_robot.normalize() * 10.);
                position_tools.gizmos_origin = Some(orig);
                gizmos.arrow(orig, orig + x_axis, Color::linear_rgb(1., 0., 0.));
                gizmos.arrow(orig, orig + y_axis, Color::linear_rgb(0., 1., 0.));
                gizmos.arrow(orig, orig + z_axis, Color::linear_rgb(0., 0., 1.));

                if let Some(ray) = scene_window_data.viewport_to_world_ray {
                    let (x_axis, y_axis, z_axis): (Vector3<_>, Vector3<_>, Vector3<_>) =
                        (x_axis.into(), y_axis.into(), z_axis.into());
                    let orig: Vector3<_> = orig.into();
                    let mut iso = Isometry3 {
                        translation: (orig + x_axis/2.).into(),
                        rotation: UnitQuaternion::identity(),
                    };
                    let ray = Ray {
                        origin: ray.origin.into(),
                        dir: ray.direction.as_vec3().into(),
                    };
                    let shape = position_tools.grab_shape;
                    let mut clicked_axes = [false, false ,false];
                        clicked_axes[0] = shape.intersects_ray(&iso, &ray, Real::MAX);
                        iso.translation = (orig + y_axis/2.).into();
                        iso.rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
                        clicked_axes[1] = shape.intersects_ray(&iso, &ray, Real::MAX);
                        iso.translation = (orig + z_axis/2.).into();
                        iso.rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);
                        clicked_axes[2] = shape.intersects_ray(&iso, &ray, Real::MAX);
                    if clicked_axes.contains(&true) {
                        selected_entities.pointer_usage_state = PointerUsageState::UsingTool;
                    }
                }
            },
            PositionTool::Rotate { .. } => {}
        }
    }
}
