use std::ops::DerefMut;
use bevy::prelude::*;
use bevy_rapier3d::na::{Isometry3, Rotation3, UnitQuaternion, UnitVector3, Vector3};
use bevy_egui::egui::Ui;
use bevy_rapier3d::parry::query::RayCast;
use bevy_rapier3d::parry::shape::{Cuboid, Cylinder};
use bevy_rapier3d::rapier::geometry::Ray;
use crate::math::compute_intersection_pos;
use crate::prelude::*;
use crate::prelude::Real as Real;
use crate::ui::{GizmosUi, GizmosUiParameters, RobotLabUiAssets, UiEvents, UiResources, View};
use crate::ui::entity_selection::PointerUsageState;

#[derive(Resource)]
pub struct PositionTools {
    pub selected_tool: PositionTool,
    pub gizmos_origin: Option<Isometry3<Real>>,
    pub gizmos_axes: Option<[UnitVector3<Real>; 3]>,
    pub local_coords: bool,
    pub init_robot_transform: Option<GlobalTransform>,

    grab_shape: Cuboid,
    rotate_shape_outer: Cylinder,
    rotate_shape_inner: Cylinder,
}

impl Default for PositionTools {
    fn default() -> Self {
        Self {
            selected_tool: default(),
            gizmos_origin: default(),
            gizmos_axes: default(),
            local_coords: default(),
            init_robot_transform: default(),

            grab_shape: Cuboid::new(Vector3::new(0.05, 0.5, 0.05)),
            rotate_shape_outer: Cylinder::new(0.05, 1.1),
            rotate_shape_inner: Cylinder::new(0.05, 0.9),
        }
    }
}

impl View for PositionTools {
    fn ui(&mut self, ui: &mut Ui, _ui_assets: &RobotLabUiAssets) {
        ui.horizontal(|ui| {
            let mut grab = matches!(self.selected_tool, PositionTool::Grab { .. });
            let grab_clicked = ui.toggle_value(&mut grab, "Grab").clicked();
            let mut rotate = matches!(self.selected_tool, PositionTool::Rotate { .. });
            let rotate_clicked = ui.toggle_value(&mut rotate, "Rotate").clicked();

            if grab && grab_clicked {
                self.selected_tool = PositionTool::Grab {
                    grabbed_axis: None,
                    plane_normal: None,
                    init_pointer_pos: None,
                    curr_pointer_pos: None,
                }
            } else if rotate && rotate_clicked {
                self.selected_tool = PositionTool::Rotate {
                    grabbed_disc_normal: None,
                    init_pointer_pos: None,
                    curr_pointer_pos: None,
                }
            }
        });
        ui.label("Coordinate Space");
        ui.horizontal(|ui| {
            let mut global = !self.local_coords;
            let global_clicked = ui.toggle_value(&mut global, "Global").clicked();
            let mut local = self.local_coords;
            let local_clicked = ui.toggle_value(&mut local, "Local").clicked();

            if global && global_clicked {
                self.local_coords = false;
            } else if local && local_clicked {
                self.local_coords = true;
            }
        });
    }

    fn functionality(_resources: &mut UiResources, _events: &mut UiEvents) -> Result<()> {
        Ok(())
    }
}

impl GizmosUi for PositionTools {
    fn gizmos_ui(ui_resources: &mut UiResources, gizmos_resources: &mut GizmosUiParameters) {
        let UiResources {
            scene_window_data,
            selected_entities,
            ..
        } = ui_resources;
        let physics_sim = ui_resources.general_tab.simulation.deref_mut();
        let position_tools = ui_resources.general_tab.position_tools.deref_mut();
        let GizmosUiParameters {
            gizmos,
            global_transform_q,
            ..
        } = gizmos_resources;

        // Don't draw gizmos when sim is active.
        if physics_sim.physics_active {
            return;
        }

        if let Some(robot) = selected_entities.active_robot {
            let robot_transform = global_transform_q.get(robot).unwrap();
            let robot_pos = robot_transform.translation();
            let robot_rot = robot_transform.rotation();
            let cam_pos = scene_window_data.camera_transform.translation();
            match position_tools.selected_tool {
                PositionTool::Grab { .. } => {
                    let [x_axis, y_axis, z_axis] = if position_tools.local_coords {
                        [
                            robot_rot * Vec3::X,
                            robot_rot * Vec3::Y,
                            robot_rot * Vec3::Z,
                        ]
                    } else {
                        [Vec3::X, Vec3::Y, Vec3::Z]
                    };
                    position_tools.gizmos_axes = Some([
                        UnitVector3::new_unchecked(x_axis.into()),
                        UnitVector3::new_unchecked(y_axis.into()),
                        UnitVector3::new_unchecked(z_axis.into()),
                    ]);
                    let cam_to_robot = robot_pos - cam_pos;
                    let orig = cam_pos + (cam_to_robot.normalize() * 10.);
                    position_tools.gizmos_origin = Some(Isometry3 {
                        translation: orig.into(),
                        rotation: robot_rot.into(),
                    });
                    gizmos.arrow(orig, orig + x_axis, Color::linear_rgb(1., 0., 0.));
                    gizmos.arrow(orig, orig + y_axis, Color::linear_rgb(0., 1., 0.));
                    gizmos.arrow(orig, orig + z_axis, Color::linear_rgb(0., 0., 1.));
                }
                PositionTool::Rotate { .. } => {
                    let [x_axis, y_axis, z_axis] = if position_tools.local_coords {
                        [
                            robot_rot * Vec3::X,
                            robot_rot * Vec3::Y,
                            robot_rot * Vec3::Z,
                        ]
                    } else {
                        [Vec3::X, Vec3::Y, Vec3::Z]
                    };
                    position_tools.gizmos_axes = Some([
                        UnitVector3::new_unchecked(x_axis.into()),
                        UnitVector3::new_unchecked(y_axis.into()),
                        UnitVector3::new_unchecked(z_axis.into()),
                    ]);
                    let cam_to_robot = robot_pos - cam_pos;
                    let orig = cam_pos + (cam_to_robot.normalize() * 10.);
                    position_tools.gizmos_origin = Some(Isometry3 {
                        translation: orig.into(),
                        rotation: robot_rot.into(),
                    });
                    let mut iso = Isometry3d {
                        translation: orig.into(),
                        rotation: robot_rot,
                    };
                    iso.rotation = Quat::from_mat3(&Mat3 {
                        x_axis: z_axis,
                        y_axis,
                        z_axis: -x_axis,
                    });
                    gizmos.circle(iso, 1., Color::linear_rgb(1., 0., 0.));
                    iso.rotation = Quat::from_mat3(&Mat3 {
                        x_axis,
                        y_axis: z_axis,
                        z_axis: -y_axis,
                    });
                    gizmos.circle(iso, 1., Color::linear_rgb(0., 1., 0.));
                    iso.rotation = Quat::from_mat3(&Mat3 {
                        x_axis,
                        y_axis,
                        z_axis,
                    });
                    gizmos.circle(iso, 1., Color::linear_rgb(0., 0., 1.));
                }
            }
        }
    }

    fn gizmos_functionality(ui_resources: &mut UiResources, gizmos_resources: &mut GizmosUiParameters, events: &mut UiEvents) -> Result<()> {
        let UiResources {
            scene_window_data,
            selected_entities,
            ..
        } = ui_resources;
        let physics_sim = ui_resources.general_tab.simulation.deref_mut();
        let position_tools = ui_resources.general_tab.position_tools.deref_mut();
        let global_transform_q = &mut gizmos_resources.global_transform_q;
        let (mouse_pressed, mouse_just_released) = (selected_entities.mouse_pressed, selected_entities.mouse_just_released);

        if scene_window_data.viewport_to_world_ray.is_none()
            || position_tools.gizmos_origin.is_none()
            || position_tools.gizmos_axes.is_none()
            || selected_entities.active_robot.is_none()
            || physics_sim.physics_active
        { return Ok(()); }

        let PositionTools {
            selected_tool,
            gizmos_origin,
            gizmos_axes,
            init_robot_transform,
            grab_shape,
            rotate_shape_outer,
            rotate_shape_inner,
            ..
        } = position_tools;

        let ray = scene_window_data.viewport_to_world_ray.unwrap();
        let gizmos_origin = gizmos_origin.unwrap();
        let axes = gizmos_axes.unwrap();
        let ray = Ray {
            origin: ray.origin.into(),
            dir: ray.direction.as_vec3().into(),
        };
        let robot = selected_entities.active_robot.unwrap();

        match selected_tool {
            PositionTool::Grab {
                ref mut grabbed_axis,
                ref mut plane_normal,
                ref mut init_pointer_pos,
                ref mut curr_pointer_pos,
            } => {
                // When mouse was just pressed
                if selected_entities.viewport_clicked {
                    let shape = grab_shape;
                    let mut clicked_axes = [false, false, false];
                    let [x_axis, y_axis, z_axis] = [
                        axes[0].into_inner(),
                        axes[1].into_inner(),
                        axes[2].into_inner(),
                    ];
                    let mut iso = Isometry3 {
                        translation: (gizmos_origin.translation.vector + x_axis * 0.5).into(),
                        rotation: UnitQuaternion::from_basis_unchecked(&[-y_axis, x_axis, z_axis]),
                    };
                    clicked_axes[0] = shape.intersects_ray(&iso, &ray, Real::MAX);
                    iso.translation = (gizmos_origin.translation.vector + y_axis * 0.5).into();
                    iso.rotation = UnitQuaternion::from_basis_unchecked(&[x_axis, y_axis, z_axis]);
                    clicked_axes[1] = shape.intersects_ray(&iso, &ray, Real::MAX);
                    iso.translation = (gizmos_origin.translation.vector + z_axis * 0.5).into();
                    iso.rotation = UnitQuaternion::from_basis_unchecked(&[x_axis, z_axis, -y_axis]);
                    clicked_axes[2] = shape.intersects_ray(&iso, &ray, Real::MAX);
                    if let Some(idx) = clicked_axes.iter().position(|b| *b) {
                        selected_entities.pointer_usage_state = PointerUsageState::UsingTool;
                        *grabbed_axis = Some(axes[idx]);
                        *plane_normal = if idx == 0 {
                            Some(axes[2])
                        } else if idx == 1 || idx == 2 {
                            Some(axes[0])
                        } else {
                            unreachable!()
                        };

                        // Save robot transform
                        *init_robot_transform = global_transform_q.get(robot).ok().copied();
                        // Computing plane intersection point
                        *init_pointer_pos = compute_intersection_pos(
                            &ray,
                            &init_robot_transform
                                .unwrap()
                                .translation()
                                .into(),
                            &plane_normal.unwrap(),
                        );
                    } else {
                        *grabbed_axis = None;
                        *plane_normal = None;
                    }
                }
                // When mouse is held down
                else if mouse_pressed {
                    if let Some(normal) = plane_normal {
                        selected_entities.pointer_usage_state = PointerUsageState::UsingTool;
                        if let Some(robot_transform) = init_robot_transform {
                            *curr_pointer_pos = compute_intersection_pos(
                                &ray,
                                &robot_transform.translation().into(),
                                &normal,
                            );
                        }
                    }
                    if let (Some(grabbed_axis), Some(init_pos), Some(curr_pos), Some(init_transform)) = (
                        grabbed_axis,
                        init_pointer_pos,
                        curr_pointer_pos,
                        init_robot_transform,
                    ) {
                        if let Ok(mut robot_transform) = global_transform_q.get_mut(robot) {
                            let disp = (*curr_pos - *init_pos).dot(grabbed_axis);
                            let axis: Vec3 = grabbed_axis.into_inner().into();
                            *robot_transform = Transform {
                                translation: init_transform.translation() + axis * disp,
                                rotation: init_transform.rotation(),
                                scale: Vec3::ONE,
                            }
                                .into();
                        }
                    }
                }
                // When mouse gets released
                else if mouse_just_released {
                    // Reset the tool's data after moving the robot.
                    *init_robot_transform = None;
                    *grabbed_axis = None;
                    *plane_normal = None;
                    *init_pointer_pos = None;
                    *curr_pointer_pos = None;
                    // TODO: register Undo action
                }
            },
            PositionTool::Rotate {
                ref mut grabbed_disc_normal,
                ref mut init_pointer_pos,
                ref mut curr_pointer_pos,
            } => {
                if selected_entities.viewport_clicked {
                    let [x_axis, y_axis, z_axis] = [
                        axes[0].into_inner(),
                        axes[1].into_inner(),
                        axes[2].into_inner(),
                    ];
                    let mut clicked_axes = [false, false, false];
                    let mut iso = gizmos_origin;
                    let inner = rotate_shape_inner;
                    let outer = rotate_shape_outer;
                    iso.rotation = Rotation3::from_basis_unchecked(&[-y_axis, x_axis, z_axis]).into();
                    clicked_axes[0] = !inner.intersects_ray(&iso, &ray, crate::math::Real::MAX)
                        && outer.intersects_ray(&iso, &ray, crate::math::Real::MAX);
                    iso.rotation = Rotation3::from_basis_unchecked(&[x_axis, y_axis, z_axis]).into();
                    clicked_axes[1] = !inner.intersects_ray(&iso, &ray, crate::math::Real::MAX)
                        && outer.intersects_ray(&iso, &ray, crate::math::Real::MAX);
                    iso.rotation = Rotation3::from_basis_unchecked(&[x_axis, z_axis, -y_axis]).into();
                    clicked_axes[2] = !inner.intersects_ray(&iso, &ray, crate::math::Real::MAX)
                        && outer.intersects_ray(&iso, &ray, crate::math::Real::MAX);

                    if let Some(idx) = clicked_axes.iter().position(|b| *b) {
                        selected_entities.pointer_usage_state = PointerUsageState::UsingTool;
                        *grabbed_disc_normal = Some(axes[idx]);

                        // Save robot transform
                        *init_robot_transform = global_transform_q.get(robot).ok().copied();
                        // Computing plane intersection point
                        *init_pointer_pos = compute_intersection_pos(
                            &ray,
                            &init_robot_transform
                                .unwrap()
                                .translation()
                                .into(),
                            &axes[idx],
                        );
                    } else {
                        *grabbed_disc_normal = None;
                    }
                } else if mouse_pressed {
                    if let Some(normal) = grabbed_disc_normal {
                        selected_entities.pointer_usage_state = PointerUsageState::UsingTool;
                        if let Some(init_transform) = init_robot_transform {
                            *curr_pointer_pos = compute_intersection_pos(
                                &ray,
                                &init_transform.translation().into(),
                                &normal,
                            );
                            if let (Some(init), Some(curr)) = (init_pointer_pos, curr_pointer_pos) {
                                if let Ok(mut robot_transform) = global_transform_q.get_mut(robot) {
                                    let robot_translation = robot_transform.translation();
                                    let (init, curr): (Vec3, Vec3) = ((*init).into(), (*curr).into());
                                    let (init_loc, curr_loc) =
                                        ((init - robot_translation), (curr - robot_translation));
                                    let angle = init_loc.angle_between(curr_loc);
                                    let angle_dir = init_loc
                                        .cross(curr_loc)
                                        .dot(normal.into_inner().into())
                                        .signum();
                                    *robot_transform = Transform {
                                        translation: init_transform.translation(),
                                        rotation: Quat::from_axis_angle(
                                            (*normal).into(),
                                            angle * angle_dir,
                                        ) * init_transform.rotation(),
                                        scale: Vec3::ONE,
                                    }
                                        .into();
                                }
                            }
                        }
                    }
                } else if mouse_just_released {
                    *init_robot_transform = None;
                    *grabbed_disc_normal = None;
                    *init_pointer_pos = None;
                    *curr_pointer_pos = None;
                }
            },
            _ => {}
        }

        Ok(())
    }
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