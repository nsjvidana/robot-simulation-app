use crate::math::compute_plane_intersection_pos;
use crate::prelude::Real;
use crate::prelude::*;
use crate::ui::selecting::{PickingRequest, PickingRequestCommandExt, PickingResponse};
use crate::ui::{FunctionalUiResources, GizmosUi, GizmosUiResources, RobotLabUiSet, UiSet, View, GIZMO_DISTANCE};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_rapier3d::na::{Isometry3, Rotation3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use bevy_rapier3d::parry::shape::{Compound, Cuboid, SharedShape};
use bevy_rapier3d::rapier::prelude::{Ray, RayCast, DIM};
use std::ops::Mul;

pub fn build_app(app: &mut App) {
    app.init_resource::<ToolbarWindow>();

    app.add_event::<MovableEntityPickedEvent>();

    app.add_systems(
        Update,
        (
            toolbar_ui_system
                .in_set(RobotLabUiSet::Ui)
                .after(UiSet::RibbonUi),
            toolbar_functionality
                .in_set(RobotLabUiSet::Functionality),
        )
    );
}

#[derive(Resource, Default)]
pub struct ToolbarWindow {
    global_coords: bool,
    pub active_movable_entity: Option<Entity>,
    position_tool: PositionTool,
}

impl ToolbarWindow {
    pub fn panel_ui(&mut self, ctx: &mut egui::Context, commands: &mut Commands) {
        egui::TopBottomPanel::top("toolbar_panel")
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    self.ui(ui, commands);
                });
            });
    }
}

impl View for ToolbarWindow {
    fn ui(&mut self, ui: &mut egui::Ui, _commands: &mut Commands) {
        let mut selected = matches!(self.position_tool, PositionTool::Translate {..});
        let toggle_val = ui.toggle_value(&mut selected, "Translate");
        if toggle_val.changed() && selected {
            self.position_tool = PositionTool::Translate(Default::default());
        }
        let mut selected = matches!(self.position_tool, PositionTool::Rotate {..});
        let toggle_val = ui.toggle_value(&mut selected, "Rotate");
        if toggle_val.changed() && selected {
            self.position_tool = PositionTool::Rotate(Default::default());
        }

        ui.toggle_value(&mut self.global_coords, "Global");
        let mut selected = !self.global_coords;
        ui.toggle_value(&mut selected, "Local");
        self.global_coords = !selected;
    }
}

impl GizmosUi for ToolbarWindow {
    fn gizmos_ui(&mut self, resources: &mut GizmosUiResources) {
        for event in resources.movable_entity_picked.drain() {
            self.active_movable_entity = Some(event.0);
        }
        self.position_tool.set_active_movable_entity(self.active_movable_entity.clone());
        match &mut self.position_tool {
            PositionTool::Translate(tool) => {
                tool.operate(self.global_coords, resources);
                tool.draw(resources);
            }
            PositionTool::Rotate(tool) => {
                tool.operate(self.global_coords, resources);
                tool.draw(resources);
            }
        }
    }
}

#[derive(Clone)]
pub enum PositionTool {
    Translate(TranslateTool),
    Rotate(RotateTool),
}

impl PositionTool {
    pub fn set_active_movable_entity(&mut self, active_movable_entity: Option<Entity>) {
        match self {
            PositionTool::Translate(tool) => tool.active_movable_entity = active_movable_entity,
            PositionTool::Rotate(tool) => tool.active_movable_entity = active_movable_entity,
        }
    }
}

impl Default for PositionTool {
    fn default() -> Self {
        Self::Translate(TranslateTool::default())
    }
}

#[derive(Clone)]
pub struct TranslateTool {
    active_movable_entity: Option<Entity>,
    init_transform: Option<GlobalTransform>,
    init_click: Option<Vector3<Real>>,
    worldspace_axes: [Vec3; DIM],
    grabbed_axis_local_idx: Option<usize>,
    grabbed_axis_normal_world_idx: Option<usize>,
    selection_blocking_resp: Option<PickingResponse>,
    shapes: Vec<Cuboid>,
}

impl TranslateTool {
    pub fn operate(&mut self, global_coords: bool, res: &mut GizmosUiResources) {
        let GizmosUiResources {
            functional_ui_resources,
            camera: camera_query_data,
            ..
        } = &mut *res;
        let FunctionalUiResources {
            transforms,
            selections,
            mouse,
            scene_window_data,
            commands,
            ..
        } = functional_ui_resources;
        
        let cam_trans = camera_query_data.transform;
        let Some(active_movable_entity) = self.active_movable_entity else { return; };
            if !transforms.contains(active_movable_entity) { return; }
        let (gizmo_origin, mut entity_transform) = {
            let entity_trans = transforms.get_mut(active_movable_entity).unwrap();
            let cam_pos = cam_trans.translation();
            let cam_to_entity = (entity_trans.translation() - cam_pos)
                .normalize()
                .mul(GIZMO_DISTANCE);
            (Vector3::from(cam_pos + cam_to_entity), entity_trans)
        };
        let entity_iso = Isometry3 {
            translation: entity_transform.translation().into(),
            rotation: entity_transform.rotation().into(),
        };

        self.worldspace_axes =
            if global_coords { [Vec3::X, Vec3::Y, Vec3::Z] }
            else {[
                entity_transform.rotation() * Vec3::X,
                entity_transform.rotation() * Vec3::Y,
                entity_transform.rotation() * Vec3::Z,
            ]};
        let axes_local = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];

        if mouse.just_pressed(MouseButton::Left) {
            let Some(ray) = scene_window_data.viewport_to_world_ray else {
                return;
            };
            let ray: Ray = W(ray).into();
            let (mut min_idx, mut min_toi) = (None, None);
            for (idx, axis_shape) in self.shapes.iter().enumerate() {
                let hit = axis_shape
                    .cast_ray_and_get_normal(
                        &Isometry3 {
                            translation: Translation3::from(Vec3::from(gizmo_origin) + self.worldspace_axes[idx] * 0.5),
                            rotation: entity_iso.rotation,
                        },
                        &ray,
                        Real::MAX,
                        true
                    );
                if hit.is_some_and(|hit| min_toi.is_some_and(|min| hit.time_of_impact < min)) ||
                    (hit.is_some() && (min_idx.is_none() || min_toi.is_none()))
                {
                    min_toi = Some(hit.unwrap().time_of_impact);
                    min_idx = Some(idx);
                }
            }
            if let (Some(local_axis_idx), Some(_)) = (min_idx, min_toi) {
                let worldspace_normal_idx =
                    if local_axis_idx == 0 { 2 }
                    else if local_axis_idx == 1 || local_axis_idx == 2 { 0 }
                    else { unreachable!() };
                let worldspace_normal =
                    UnitVector3::new_unchecked(self.worldspace_axes[worldspace_normal_idx].into());
                self.init_transform = Some(*entity_transform);
                self.grabbed_axis_local_idx = Some(local_axis_idx);
                self.grabbed_axis_normal_world_idx = Some(worldspace_normal_idx);
                self.init_click = compute_plane_intersection_pos(&ray, &gizmo_origin, &worldspace_normal);
                self.selection_blocking_resp = Some(
                    commands.send_picking_request(PickingRequest::new(|_, _| true).continuous(true))
                );
            }
        }
        if mouse.pressed(MouseButton::Left) {
            let Some(ray) = scene_window_data.viewport_to_world_ray else {
                return;
            };
            let ray: Ray = W(ray).into();
            if let (
                Some(init_transform),
                Some(init_click),
                Some(axis),
                Some(worldspace_normal_idx)
            ) = (self.init_transform, self.init_click, self.grabbed_axis_local_idx, self.grabbed_axis_normal_world_idx)
            {
                let worldspace_normal =
                    UnitVector3::new_unchecked(self.worldspace_axes[worldspace_normal_idx].into());
                if let Some(curr_pos) = compute_plane_intersection_pos(&ray, &gizmo_origin, &worldspace_normal) {
                    let init = init_click - gizmo_origin;
                    let curr = curr_pos - gizmo_origin;
                    let grabbed_axis = axes_local[axis];
                    let disp = curr.dot(&grabbed_axis) - init.dot(&grabbed_axis);
                    let translation = GlobalTransform::from_translation(
                        Vec3::from(grabbed_axis) * disp
                    );
                    *entity_transform =
                        if global_coords {
                            translation * init_transform
                        }
                        else {
                            init_transform * translation
                        };
                }
            }
        }
        if mouse.just_released(MouseButton::Left) {
            self.reset_data();
        }
    }

    pub fn reset_data(&mut self) {
        self.init_transform = None;
        self.init_click = None;
        self.grabbed_axis_local_idx = None;
        self.grabbed_axis_normal_world_idx = None;
        if let Some(resp) = self.selection_blocking_resp.take() {
            resp.cancel_request();
        }
    }

    pub fn draw(&self, res: &mut GizmosUiResources) {
        let Some(entity_e) = self.active_movable_entity else { return; };
        let cam_trans = *res.camera.transform;
        let Ok(entity_trans) = res.transforms.get(entity_e).cloned() else { return; };
        let gizmos = &mut res.gizmos;
        let [x, y, z] = self.worldspace_axes;

        // Draw axes
        {
            let x: Vec3 = x.into();
            let y: Vec3 = y.into();
            let z: Vec3 = z.into();
            let cam_pos = cam_trans.translation();
            let cam_to_entity = (entity_trans.translation() - cam_pos)
                .normalize()
                .mul(GIZMO_DISTANCE);
            let origin = cam_pos + cam_to_entity;
            gizmos.arrow(
                origin,
                origin + x,
                Color::linear_rgb(1., 0., 0.)
            );
            gizmos.arrow(
                origin,
                origin + y,
                Color::linear_rgb(0., 1., 0.)
            );
            gizmos.arrow(
                origin,
                origin + z,
                Color::linear_rgb(0., 0., 1.)
            );
        }

        if let Some(pos) = self.init_click {
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                0.005,
                Color::WHITE
            );
        }
    }
}

impl Drop for TranslateTool {
    fn drop(&mut self) {
        if let Some(resp) = &mut self.selection_blocking_resp {
            resp.cancel_request();
        }
    }
}

impl Default for TranslateTool {
    fn default() -> Self {
        Self {
            active_movable_entity: None,
            init_transform: None,
            init_click: None,
            worldspace_axes: [Vec3::X, Vec3::Y, Vec3::Z],
            grabbed_axis_local_idx: None,
            grabbed_axis_normal_world_idx: None,
            selection_blocking_resp: None,
            shapes: vec![
                Cuboid::new(Vector3::new(0.5, 0.05, 0.05)),
                Cuboid::new(Vector3::new(0.05, 0.5, 0.05)),
                Cuboid::new(Vector3::new(0.05, 0.05, 0.5)),
            ]
        }
    }
}

#[derive(Clone)]
pub struct RotateTool {
    active_movable_entity: Option<Entity>,
    init_transform: Option<GlobalTransform>,
    init_click: Option<Vector3<Real>>,
    worldspace_axes: [Vec3; DIM],
    ring_normal_idx: Option<usize>,
    selection_blocking_resp: Option<PickingResponse>,
    // (outer ring, inner ring)
    rings: Vec<(Compound, Compound)>,
}

impl RotateTool {
    pub fn operate(&mut self, global_coords: bool, res: &mut GizmosUiResources) {
        let GizmosUiResources {
            functional_ui_resources,
            camera: camera_query_data,
            ..
        } = &mut *res;
        let FunctionalUiResources {
            transforms: entity_transforms,
            selections,
            mouse,
            scene_window_data,
            commands,
            ..
        } = functional_ui_resources;
        
        let cam_trans = camera_query_data.transform;
        let Some((gizmo_origin, mut entity_transform)) = self.active_movable_entity
            .map(|entity_e| {
                let entity_trans = entity_transforms.get_mut(entity_e).unwrap();
                let cam_pos = cam_trans.translation();
                let cam_to_entity = (entity_trans.translation() - cam_pos)
                    .normalize()
                    .mul(GIZMO_DISTANCE);
                (Vector3::from(cam_pos + cam_to_entity), entity_trans)
            })
        else { return; };
        let entity_iso = Isometry3 {
            translation: entity_transform.translation().into(),
            rotation: entity_transform.rotation().into(),
        };

        let axes = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
        self.worldspace_axes = if global_coords { [Vec3::X, Vec3::Y, Vec3::Z] }
            else {[
                entity_transform.rotation() * Vec3::X,
                entity_transform.rotation() * Vec3::Y,
                entity_transform.rotation() * Vec3::Z,
            ]};

        if mouse.just_pressed(MouseButton::Left) {
            let Some(ray) = scene_window_data.viewport_to_world_ray else {
                return;
            };
            let ray: Ray = W(ray).into();
            let iso = Isometry3 {
                translation: gizmo_origin.into(),
                rotation:
                    if global_coords { UnitQuaternion::identity() }
                    else { entity_iso.rotation },
            };
            let (mut min_idx, mut min_toi) = (None, None);
            for (idx, (outer, inner)) in self.rings.iter().enumerate() {
                let outer_hit = outer
                    .cast_ray_and_get_normal(
                        &iso,
                        &ray,
                        Real::MAX,
                        true
                    );
                let inner_hit = inner
                    .cast_ray_and_get_normal(
                        &iso,
                        &ray,
                        Real::MAX,
                        true
                    );
                if outer_hit.is_some() && inner_hit.is_none() {
                    let outer_hit = outer_hit.unwrap();
                    if min_toi.is_some_and(|min_toi| outer_hit.time_of_impact < min_toi) ||
                        (min_idx.is_none() || min_toi.is_none())
                    {
                        min_idx = Some(idx);
                        min_toi = Some(outer_hit.time_of_impact);
                    }
                }
            }
            if let (Some(min_idx), Some(_)) = (min_idx, min_toi) {
                let worldspace_normal = UnitVector3::new_unchecked(self.worldspace_axes[min_idx].into());
                self.init_transform = Some(*entity_transform);
                self.init_click = compute_plane_intersection_pos(
                    &ray,
                    &gizmo_origin,
                    &worldspace_normal
                );
                self.ring_normal_idx = Some(min_idx);
                self.selection_blocking_resp = Some(commands.send_picking_request(PickingRequest::new(|_,_| true).continuous(true)));
            }
        }
        else if mouse.pressed(MouseButton::Left) {
            let (
                Some(init_transform),
                Some(init_click),
                Some(normal_idx),
                Some(ray),
            ) = (self.init_transform, self.init_click, self.ring_normal_idx, scene_window_data.viewport_to_world_ray) else {
                return;
            };
            let ray: Ray = W(ray).into();
            let worldspace_normal = UnitVector3::new_unchecked(self.worldspace_axes[normal_idx].into());
            let normal = axes[normal_idx];
            if let Some(curr_pos) = compute_plane_intersection_pos(&ray, &gizmo_origin, &worldspace_normal) {
                let init = Vec3::from(init_click - gizmo_origin);
                let curr = Vec3::from(curr_pos - gizmo_origin);
                let angle = init.angle_between(curr);
                let angle_dir = init
                    .cross(curr)
                    .dot(normal.into_inner().into())
                    .signum();
                let rot = GlobalTransform::from_rotation(
                    Quat::from_scaled_axis(Vec3::from(normal) * angle * angle_dir)
                );
                *entity_transform =
                    if global_coords {
                        let mut trans = Transform::from(rot * init_transform);
                            trans.translation = init_transform.translation();
                        trans.into()
                    }
                    else {
                        init_transform * rot
                    }
            }
        }
        else if mouse.just_released(MouseButton::Left) {
            self.reset_data();
        }
    }

    pub fn reset_data(&mut self) {
        self.init_transform = None;
        self.init_click = None;
        self.ring_normal_idx = None;
        if let Some(resp) = self.selection_blocking_resp.take() {
            resp.cancel_request();
        }
    }

    pub fn draw(&self, res: &mut GizmosUiResources) {
        let Some(entity_e) = self.active_movable_entity else { return; };
        let cam_trans = *res.camera.transform;
        let entity_trans = *res.transforms.get(entity_e).unwrap();
        let gizmos = &mut res.gizmos;

        let cam_pos = cam_trans.translation();
        let cam_to_entity = (entity_trans.translation() - cam_pos)
            .normalize()
            .mul(GIZMO_DISTANCE);
        let origin = cam_pos + cam_to_entity;

        // Draw rings
        {
            let [x_axis, y_axis, z_axis] = self.worldspace_axes;
            let mut iso = Isometry3d {
                translation: origin.into(),
                rotation: entity_trans.rotation(),
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

        if let Some(pos) = self.init_click {
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                0.005,
                Color::WHITE
            );
        }
    }
}

impl Drop for RotateTool {
    fn drop(&mut self) {
        if let Some(resp) = &mut self.selection_blocking_resp {
            resp.cancel_request();
        }
    }
}

impl Default for RotateTool {
    fn default() -> Self {
        let outer_ring = SharedShape::cylinder(0.05, 1.1);
        let inner_ring = SharedShape::cylinder(0.05, 0.9);
        let x_axis = Vector3::x();
        let y_axis = Vector3::y();
        let z_axis = Vector3::z();
        let mut iso = Isometry3::identity();
        iso.rotation = Rotation3::from_basis_unchecked(&[-y_axis, x_axis, z_axis]).into();
            let iso1 = iso;
        iso.rotation = Rotation3::from_basis_unchecked(&[x_axis, y_axis, z_axis]).into();
            let iso2 = iso;
        iso.rotation = Rotation3::from_basis_unchecked(&[x_axis, z_axis, -y_axis]).into();
            let iso3 = iso;
        Self {
            active_movable_entity: None,
            init_transform: None,
            init_click: None,
            worldspace_axes: [Vec3::X, Vec3::Y, Vec3::Z],
            ring_normal_idx: None,
            selection_blocking_resp: None,
            rings: vec![
                (Compound::new(vec![(iso1, outer_ring.clone())]) , Compound::new(vec![(iso1, inner_ring.clone())])),
                (Compound::new(vec![(iso2, outer_ring.clone())]) , Compound::new(vec![(iso2, inner_ring.clone())])),
                (Compound::new(vec![(iso3, outer_ring)]) , Compound::new(vec![(iso3, inner_ring)])),
            ]
        }
    }
}

#[derive(Event)]
pub struct MovableEntityPickedEvent(pub Entity);

pub fn toolbar_ui_system(
    mut toolbar_window: ResMut<ToolbarWindow>,
    mut ctxs: EguiContexts,
    mut commands: Commands,
) {
    toolbar_window.panel_ui(ctxs.ctx_mut(), &mut commands);
}

pub fn toolbar_functionality(
    mut toolbar_window: ResMut<ToolbarWindow>,
    mut gizmos_resources: GizmosUiResources,
) {
    toolbar_window.gizmos_ui(&mut gizmos_resources)
}
