pub mod general;
pub mod motion_planning;
pub mod ribbon;
pub mod entity_selection;

use crate::kinematics::ik::{ForwardAscentCyclic, ForwardDescentCyclic};
use crate::math::ray_scale_for_plane_intersect_local;
use crate::prelude::*;
use bevy::app::App;
use bevy::asset::Handle;
use bevy::ecs::intern::Interned;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::ecs::system::{SystemParam, SystemState};
use bevy::gizmos::AppGizmoBuilder;
use bevy::gizmos::GizmoPlugin;
use bevy::image::Image;
use bevy::math::Vec3;
use bevy::prelude::{AssetServer, ButtonInput, Camera, Color, Commands, Component, DetectChanges, Entity, Event, EventReader, EventWriter, Events, FromWorld, GizmoConfigGroup, GizmoConfigStore, Gizmos, GlobalTransform, IntoSystemConfigs, Isometry3d, KeyCode, Local, Mat3, MouseButton, Name, NonSendMut, Or, Parent, Plugin, Quat, Query, Ray3d, Reflect, Res, ResMut, Resource, Single, Transform, Window, With, World};
use bevy_egui::egui::{Align, ComboBox, Label, Layout, TextureId, Ui, UiBuilder};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use bevy_rapier3d::parry::query::RayCast;
use bevy_rapier3d::plugin::TimestepMode;
use bevy_rapier3d::prelude::{
    DefaultRapierContext, ImpulseJoint, MultibodyJoint, PhysicsSet, QueryFilter,
    RapierConfiguration, RapierContext, RapierImpulseJointHandle, RapierMultibodyJointHandle,
    RayIntersection, ReadRapierContext,
};
use bevy_rapier3d::rapier::crossbeam::channel::internal::select;
use bevy_rapier3d::rapier::prelude::{Cuboid, Ray};
use k::nalgebra::{Isometry3, UnitQuaternion, UnitVector3, Vector3};
use k::{InverseKinematicsSolver, SerialChain};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use std::cmp::Ordering;
use std::ops::DerefMut;
use entity_selection::{SceneWindowData, SelectedEntities};
use general::import::Import;
use general::simulation::Simulation;
use crate::general::{ImportEvent, SimulationEvent};
use crate::motion_planning::{AllInstructions, Plan, PlanEvent};
use crate::ui::entity_selection::EntitySelectionServer;
use crate::ui::general::GeneralTab;
use crate::ui::motion_planning::{IkWindowUiEvent, MotionPlanning};

pub const TOOLTIP_LAYER: &'static str = "egui tooltips";

pub struct RobotLabUiPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl RobotLabUiPlugin {
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Plugin for RobotLabUiPlugin {
    fn build(&self, app: &mut App) {
        // Ensure that plugins this plugin depends on are already added
        if !app.is_plugin_added::<EguiPlugin>() {
            app.add_plugins(EguiPlugin);
        }
        if !app.is_plugin_added::<GizmoPlugin>() {
            app.add_plugins(GizmoPlugin);
        }

        app.add_event::<IkWindowUiEvent>();

        app.init_resource::<SceneWindowData>()
            .init_resource::<SelectedEntities>()
            .init_resource::<RobotLabUiAssets>()
            .init_resource::<Import>()
            .init_resource::<general::position_tools::PositionTools>()
            .init_resource::<Simulation>()

            .init_resource::<ribbon::Ribbon>();

        app
            .init_non_send_resource::<MotionPlanning>()
            .init_non_send_resource::<EntitySelectionServer>();

        // Make all UI gizmos draw on top of everything
        app.init_gizmo_group::<UiGizmoGroup>();
        let mut gizmo_config_store = app
            .world_mut()
            .get_resource_mut::<GizmoConfigStore>()
            .unwrap();
        let (ui_gizmo_config, _) = gizmo_config_store.config_mut::<UiGizmoGroup>();
        ui_gizmo_config.depth_bias = -1.;

        app.add_systems(
            self.schedule,
            (
                entity_selection::update_scene_window_data,
                ribbon::ribbon_ui,
                entity_selection::update_hovered_entities,
                ribbon::ribbon_functionality,
                entity_selection::select_entities,
            )
                .chain(),
        );
    }
}

#[derive(SystemParam)]
pub struct UiResources<'w, 's> {
    general_tab: GeneralTab<'w>,
    motion_planning_tab: NonSendMut<'w, MotionPlanning>,

    // Robot resources
    plan_q: Query<'w, 's, &'static Plan>,
    robot_q: Query<'w, 's, (&'static Robot, &'static RobotKinematics)>,

    instructions: Res<'w, AllInstructions>,

    selected_entities: ResMut<'w, SelectedEntities>,
    scene_window_data: Res<'w, SceneWindowData>,
}

#[derive(SystemParam)]
pub struct GizmosUiParameters<'w, 's> {
    gizmos: Gizmos<'w, 's, UiGizmoGroup>,
    transform_q: Query<'w, 's, &'static mut Transform>,
    global_transform_q: Query<'w, 's, &'static mut GlobalTransform>
}

#[derive(SystemParam)]
pub struct UiEvents<'w> {
    plan_events: ResMut<'w, Events<PlanEvent>>,
    import_events: EventWriter<'w, ImportEvent>,
    simulation_events: EventWriter<'w, SimulationEvent>,
    ik_window_events: ResMut<'w, Events<IkWindowUiEvent>>
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

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct UiGizmoGroup;

pub trait View {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets);

    fn functionality(
        resources: &mut UiResources,
        events: &mut UiEvents,
    ) -> Result<()>;
}

pub trait GizmosUi {
    fn gizmos_ui(
        ui_resources: &mut UiResources,
        gizmos_resources: &mut GizmosUiParameters,
    );

    fn gizmos_functionality(
        ui_resources: &mut UiResources,
        gizmos_resources: &mut GizmosUiParameters,
        events: &mut UiEvents,
    ) -> Result<()>;
}

pub trait WindowUI {
    fn window_ui(&mut self, egui_ctx: &mut egui::Context, ui_assets: &RobotLabUiAssets);
}

#[macro_export]
macro_rules! transparent_button {
    ($txt:expr) => {
        egui::Button::new($txt).fill(egui::Color32::TRANSPARENT)
    };
}

