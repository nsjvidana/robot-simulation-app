pub mod position_tools;
mod import;
mod simulation;
mod ribbon;
mod motion_planning;

use crate::kinematics::ik::{ForwardAscentCyclic, ForwardDescentCyclic};
use crate::math::{ray_scale_for_plane_intersect_local, Real};
use crate::robot::{Robot, RobotPart, RobotSet};
use bevy::app::App;
use bevy::gizmos::AppGizmoBuilder;
use bevy::asset::Handle;
use bevy::ecs::intern::Interned;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::ecs::system::SystemState;
use bevy::image::Image;
use bevy::math::Vec3;
use bevy::prelude::{AssetServer, ButtonInput, Camera, Color, Commands, Component, DetectChanges, Entity, FromWorld, GizmoConfigGroup, GizmoConfigStore, Gizmos, GlobalTransform, IntoSystemConfigs, Isometry3d, Local, Mat3, MouseButton, Name, Plugin, Quat, Query, Ray3d, Reflect, Res, ResMut, Resource, Single, Transform, Window, With, World};
use bevy_egui::egui::{Align, ComboBox, Label, Layout, TextureId, Ui, UiBuilder};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use bevy_rapier3d::parry::query::RayCast;
use bevy_rapier3d::plugin::TimestepMode;
use bevy_rapier3d::prelude::{DefaultRapierContext, PhysicsSet, QueryFilter, RapierConfiguration, RapierContext, ReadDefaultRapierContext};
use bevy_rapier3d::rapier::prelude::{Cuboid, Ray};
use k::{InverseKinematicsSolver, SerialChain};
use nalgebra::{Isometry3, UnitQuaternion, UnitVector3, Vector3};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use std::cmp::Ordering;
use std::ops::DerefMut;
use bevy::gizmos::GizmoPlugin;

pub struct RobotLabUiPlugin {
    schedule: Interned<dyn ScheduleLabel>,
    physics_schedule: Interned<dyn ScheduleLabel>
}

impl RobotLabUiPlugin {
    pub fn new(schedule: impl ScheduleLabel, physics_schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            physics_schedule: physics_schedule.intern()
        }
    }
}

impl Plugin for RobotLabUiPlugin {
    fn build(&self, app: &mut App) {

        // Ensure that the plugins this plugin depends on are added to the app.
        if app.get_added_plugins::<EguiPlugin>().is_empty() {
            app.add_plugins(EguiPlugin);
        }
        if app.get_added_plugins::<GizmoPlugin>().is_empty() {
            app.add_plugins(GizmoPlugin);
        }

        app
            .init_resource::<SceneWindowData>()
            .init_resource::<SelectedEntities>()
            .init_resource::<RobotLabUiAssets>()
            .init_resource::<ribbon::Ribbon>()
            .init_resource::<import::RobotImporting>()
            .init_resource::<position_tools::PositionTools>()
            .init_resource::<simulation::PhysicsSimulation>();

        app.init_non_send_resource::<motion_planning::MotionPlanning>();

        // Make all UI gizmos draw on top of everything
        app.init_gizmo_group::<UiGizmoGroup>();
        let mut gizmo_config_store = app.world_mut().get_resource_mut::<GizmoConfigStore>().unwrap();
        let (ui_gizmo_config, _) = gizmo_config_store.config_mut::<UiGizmoGroup>();
        ui_gizmo_config.depth_bias = -1.;

        app.add_systems(
            self.schedule,
            (
                update_scene_window_data,
                ribbon::ribbon_ui,
                update_clicked_entities,
                ribbon::ribbon_functionality,
                select_entities,
            ).chain()
        );
        app.add_systems(
            self.physics_schedule,
            simulation::control_simulation
                .before(PhysicsSet::StepSimulation)
                .after(PhysicsSet::SyncBackend)
        );
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct UiGizmoGroup;

#[derive(Resource, Default)]
pub struct SceneWindowData {
    viewport_to_world_ray: Option<Ray3d>,
    camera_transform: GlobalTransform
}

#[derive(Resource, Default)]
pub struct SelectedEntities {
    /// The entities that were clicked this frame.
    ///
    /// This has all the entities under the pointer when the user clicked
    /// including those that are behind others.
    pub clicked_entities: Vec<Entity>,
    /// If the simulation viewport was clicked
    pub viewport_clicked: bool,
    pub pointer_usage_state: PointerUsageState,
    pub selected_entities: Vec<Entity>,
    pub selected_robots: Vec<Entity>,
    pub active_robot: Option<Entity>,
    pub selection_mode: EntitySelectionMode,
}

/// An enum that tells how the pointer using the Positioning tools
#[derive(Default, Debug)]
pub enum PointerUsageState {
    UsingTool,
    UiUsingPointer,
    #[default]
    NotUsed
}

#[derive(Default)]
pub enum EntitySelectionMode {
    /// Only one robot can be selected at a time.
    #[default]
    SelectOneRobot,
    /// Only select robots as a whole.
    SelectRobots,
    /// Select robot parts only from the current `active_robot`.
    SelectRobotPartsLocal,
    /// Select robot parts from multiple robots
    SelectRobotPartsGlobal,
}

#[derive(Resource)]
pub struct RobotLabUiAssets {
    play_img_handle: Handle<Image>,
    new_window_img_handle: Handle<Image>,
    play_img: TextureId,
    new_window_img: TextureId,
}
impl FromWorld for RobotLabUiAssets {
    fn from_world(world: &mut World) -> Self {
        let mut sys_state: SystemState<EguiContexts> = SystemState::new(world);

        let asset_server = world.get_resource::<AssetServer>().unwrap();

        let new_window_img_handle: Handle<Image> = asset_server.load("../../assets/new_window.png");
        let play_img_handle: Handle<Image> = asset_server.load("../../assets/play.png");

        let mut egui_ctxs = sys_state.get_mut(world);
        Self {
            play_img: egui_ctxs.add_image(new_window_img_handle.clone_weak()),
            new_window_img: egui_ctxs.add_image(play_img_handle.clone_weak()),
            play_img_handle,
            new_window_img_handle,
        }
    }
}

#[macro_export]
macro_rules! transparent_button {
    ($txt:expr) => {
        egui::Button::new($txt).fill(egui::Color32::TRANSPARENT)
    };
}

pub fn update_scene_window_data(
    camera: Single<(&Camera, &GlobalTransform)>,
    mut scene_window_data: ResMut<SceneWindowData>,
    window: Single<&Window>
) {
    let (camera, camera_transform) = *camera;
    scene_window_data.camera_transform = camera_transform.clone();
    scene_window_data.viewport_to_world_ray = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok());
}

pub fn update_clicked_entities(
    mut egui_ctxs: EguiContexts,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    rapier_context: ReadDefaultRapierContext,
    mut selected_entities: ResMut<SelectedEntities>,
    scene_window_data: Res<SceneWindowData>,
) {
    selected_entities.clicked_entities.clear();
    selected_entities.viewport_clicked =
        !egui_ctxs.ctx_mut().is_pointer_over_area() && mouse_button_input.just_pressed(MouseButton::Left);
    if !selected_entities.viewport_clicked {
        return;
    }
    if let Some(ray) = scene_window_data.viewport_to_world_ray {
        let mut intersections = Vec::new();
        rapier_context.intersections_with_ray(
            ray.origin,
            ray.direction.as_vec3(),
            Real::MAX,
            true,
            QueryFilter::default(),
            |ent, intersection| {
                intersections.push((ent, intersection.time_of_impact));
                true
            }
        );

        intersections.sort_by(|(_, a), (_, b)| {
            if let Some(ord) = a.partial_cmp(b) { ord }
            else { Ordering::Equal }
        });

        let mut entities_behind_pointer: Vec<_> = intersections
            .iter()
            .map(|v| v.0)
            .collect();
        selected_entities.clicked_entities.append(&mut entities_behind_pointer);
    }
}

pub fn select_entities(
    mut selected_entities: ResMut<SelectedEntities>,
    robot_part_q: Query<&RobotPart>,
) {
    if !selected_entities.viewport_clicked
        || matches!(selected_entities.pointer_usage_state, PointerUsageState::UsingTool)
    {
        return;
    }
    match selected_entities.selection_mode {
        EntitySelectionMode::SelectOneRobot => {
            selected_entities.selected_robots.clear();
            selected_entities.active_robot = None;
            if let Some(ent) = selected_entities.clicked_entities.first() {
                if let Ok(robot) = robot_part_q.get(*ent).map(|v| v.0) {
                    selected_entities.selected_robots.push(robot);
                    selected_entities.active_robot = Some(robot);
                }
            }
        },
        _ => todo!()
    }
}
