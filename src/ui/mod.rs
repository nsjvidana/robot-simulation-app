pub mod general;
pub mod motion_planning;
pub mod ribbon;

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
use bevy::prelude::{AssetServer, ButtonInput, Camera, Color, Commands, Component, DetectChanges, Entity, EventWriter, Events, FromWorld, GizmoConfigGroup, GizmoConfigStore, Gizmos, GlobalTransform, IntoSystemConfigs, Isometry3d, KeyCode, Local, Mat3, MouseButton, Name, NonSendMut, Or, Parent, Plugin, Quat, Query, Ray3d, Reflect, Res, ResMut, Resource, Single, Transform, Window, With, World};
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
use general::import::Import;
use general::simulation::Simulation;
use crate::general::{ImportEvent, SimulationEvent};
use crate::motion_planning::{AllInstructions, Plan, PlanEvent};
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

        app.init_non_send_resource::<MotionPlanning>();

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
                update_scene_window_data,
                ribbon::ribbon_ui,
                update_hovered_entities,
                ribbon::ribbon_functionality,
                select_entities,
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
    plan_events: EventWriter<'w, PlanEvent>,
    import_events: EventWriter<'w, ImportEvent>,
    simulation_events: EventWriter<'w, SimulationEvent>,
    ik_window_events: ResMut<'w, Events<IkWindowUiEvent>>
}

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

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct UiGizmoGroup;

#[derive(Resource, Default)]
pub struct SceneWindowData {
    viewport_to_world_ray: Option<Ray3d>,
    camera_transform: GlobalTransform,
}

#[derive(Resource, Default)]
pub struct SelectedEntities {
    /// The entities that were clicked this frame.
    ///
    /// This has all the entities under the pointer when the user clicked
    /// including those that are behind others.
    pub hovered_entities: Vec<Entity>,
    /// If the simulation viewport was clicked
    pub viewport_clicked: bool,
    pub mouse_just_released: bool,
    pub mouse_pressed: bool,
    pub pointer_usage_state: PointerUsageState,

    selection_root: Option<Entity>,
    selected_joints: Vec<Entity>,
    pub selected_robots: Vec<Entity>,

    pub active_robot: Option<Entity>,
    pub selection_mode: EntitySelectionMode,
}

impl SelectedEntities {
    pub fn set_selection_mode(&mut self, selection_mode: EntitySelectionMode) -> &mut Self {
        match selection_mode {
            EntitySelectionMode::SelectOneRobot => {
                self.selected_robots.clear();
                if let Some(active_robot) = self.active_robot {
                    self.selected_robots.push(active_robot);
                }
            }
            EntitySelectionMode::SelectRobotJointsLocal => {
                self.selected_joints.clear();
            }
            _ => todo!(),
        }
        self.selection_mode = selection_mode;
        self
    }
}

/// An enum that tells how the pointer using the Positioning tools
#[derive(Default, Debug)]
pub enum PointerUsageState {
    UsingTool,
    UiUsingPointer,
    #[default]
    NotUsed,
}

#[derive(Debug, Default)]
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
    /// Select robot joints only from the current `active_robot`.
    SelectRobotJointsLocal,
    /// Only select one serial chain at a time
    SelectSerialChainLocal {
        selection_root: Option<Entity>,
        selected_joints: Vec<Entity>,
        /// List of entities that will have a UI overlay when attempting to select
        selection_preview_joints: Vec<Entity>,
        hovered_joint: Option<Entity>,
    },
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
    window: Single<&Window>,
) {
    let (camera, camera_transform) = *camera;
    scene_window_data.camera_transform = camera_transform.clone();
    scene_window_data.viewport_to_world_ray = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok());
}

pub fn update_hovered_entities(
    mut egui_ctxs: EguiContexts,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    rapier_context: ReadRapierContext,
    mut selected_entities: ResMut<SelectedEntities>,
    scene_window_data: Res<SceneWindowData>,
) {
    selected_entities.hovered_entities.clear();
    selected_entities.viewport_clicked = !egui_ctxs.ctx_mut().is_pointer_over_area()
        && mouse_button_input.just_pressed(MouseButton::Left);
    // Wipe hovered entity data when mouse ray can't be found
    if scene_window_data.viewport_to_world_ray.is_none() {
        selected_entities.hovered_entities.clear();
        return;
    }
    let ray = scene_window_data.viewport_to_world_ray.unwrap();
    // Get hovered entities
    let mut intersections = vec![];
    get_intersections(&mut intersections, &rapier_context.single(), &ray);
    let mut ents = intersections.iter().map(|v| v.0).collect();
    selected_entities.hovered_entities.append(&mut ents);

    fn get_intersections(
        intersections: &mut Vec<(Entity, Real)>,
        rapier_context: &RapierContext,
        ray: &Ray3d,
    ) {
        rapier_context.intersections_with_ray(
            ray.origin,
            ray.direction.as_vec3(),
            Real::MAX,
            true,
            QueryFilter::default(),
            |ent, intersection| {
                intersections.push((ent, intersection.time_of_impact));
                true
            },
        );
        intersections.sort_by(|(_, a), (_, b)| {
            if let Some(ord) = a.partial_cmp(b) {
                ord
            } else {
                Ordering::Equal
            }
        });
    }
}

pub fn select_entities(
    mut selected_entities: ResMut<SelectedEntities>,
    scene_window_data: Res<SceneWindowData>,
    transform_q: Query<&GlobalTransform>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    robot_q: Query<&Robot>,
    robot_part_q: Query<(&RobotPart, Option<&Parent>)>,
    robot_joint_q: Query<
        (Option<&ImpulseJoint>, Option<&MultibodyJoint>),
        Or<(
            With<RapierImpulseJointHandle>,
            With<RapierMultibodyJointHandle>,
        )>,
    >,
    mut gizmos: Gizmos<UiGizmoGroup>,
) {
    let alt_pressed = keyboard.pressed(KeyCode::AltLeft) || keyboard.pressed(KeyCode::AltRight);
    let shift_pressed =
        keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight);
    selected_entities.mouse_just_released = mouse.just_released(MouseButton::Left);
    selected_entities.mouse_pressed = mouse.pressed(MouseButton::Left);

    // Selection visuals + getting selection entities
    let cam_pos = scene_window_data.camera_transform.translation();
    let hovered = selected_entities.hovered_entities.first().copied();
    let active_robot = selected_entities.active_robot;
    match selected_entities.selection_mode {
        EntitySelectionMode::SelectSerialChainLocal {
            ref selection_root,
            ref mut selected_joints,
            ref mut selection_preview_joints,
            ref mut hovered_joint,
        } => 'serial_chain_local: {
            let mut draw_chain = |root: Entity,
                                  joints: &Vec<Entity>,
                                  gizmos: &mut Gizmos<UiGizmoGroup>,
                                  alpha: f32| {
                let root_pos = transform_q.get(root).unwrap().translation();
                gizmos.sphere(
                    cam_pos + (root_pos - cam_pos).normalize() * 10.,
                    0.1,
                    Color::linear_rgba(1., 1., 1., alpha),
                );

                let prev_pos = root_pos;
                for joint in joints.iter() {
                    let joint_pos = transform_q.get(*joint).unwrap().translation();
                    gizmos.line(prev_pos, joint_pos, Color::linear_rgba(0., 1., 0., alpha))
                }
            };
            // Drawing selected root/chain
            if let Some(root) = *selection_root {
                draw_chain(root, selected_joints, &mut gizmos, 1.);
            }

            let prev_hovered = hovered_joint.take();
            // If no active robot, don't draw UI.
            if active_robot.is_none() || hovered.is_none() {
                break 'serial_chain_local;
            }
            let active_robot = active_robot.unwrap();
            let hovered = hovered.unwrap();
            // If entity not part of robot, don't draw UI
            let part = robot_part_q.get(hovered);
            if part.is_ok_and(|part| part.0 .0 != active_robot) || part.is_err() {
                break 'serial_chain_local;
            }
            let (_, parent) = part.unwrap();
            // Get joint entity
            let joint_ent = if robot_joint_q.contains(hovered) {
                hovered
            } else if parent.is_some_and(|par| robot_joint_q.contains(par.get())) {
                parent.unwrap().get()
            } else {
                break 'serial_chain_local;
            };
            *hovered_joint = Some(joint_ent);

            // If hovered joint changed, update selection joint chain and display it
            if alt_pressed && prev_hovered.is_none_or(|e| e != joint_ent) {
                selection_preview_joints.clear();
                if let Some(root) = *selection_root {
                    // Chain selection
                    let robot = robot_q
                        .get(active_robot)
                        .expect("Active robot missing Robot component!");
                    let mut curr_joint = robot_joint_q.get(joint_ent).ok();
                    let mut curr_joint_ent = joint_ent;
                    while curr_joint.is_some() {
                        selection_preview_joints.push(curr_joint_ent);
                        if curr_joint_ent == root {
                            break;
                        }
                        let (imp_j, mb_j) = curr_joint.unwrap();
                        let joint_parent = match robot.robot_joint_type {
                            RobotJointType::ImpulseJoints => imp_j.unwrap().parent,
                            RobotJointType::MultibodyJoints(..) => mb_j.unwrap().parent,
                        };
                        curr_joint = robot_joint_q.get(joint_parent).ok();
                        curr_joint_ent = joint_parent;
                    }
                }
            }
            // Draw root selection
            else if !alt_pressed {
                let ent_pos = transform_q.get(hovered).unwrap().translation();
                gizmos.sphere(
                    cam_pos + (ent_pos - cam_pos).normalize() * 10.,
                    0.1,
                    Color::linear_rgba(1., 1., 1., 0.3),
                );
            }

            if alt_pressed {
                if let Some(root) = *selection_root {
                    draw_chain(root, selection_preview_joints, &mut gizmos, 0.3);
                }
            }
        }
        _ => {}
    }

    // Handling clicks
    if !selected_entities.viewport_clicked
        || matches!(
            selected_entities.pointer_usage_state,
            PointerUsageState::UsingTool
        )
    {
        return;
    }

    match selected_entities.selection_mode {
        EntitySelectionMode::SelectOneRobot => {
            selected_entities.selected_robots.clear();
            selected_entities.active_robot = None;
            if let Some(ent) = selected_entities.hovered_entities.first() {
                if let Ok(robot) = robot_part_q.get(*ent).map(|(part, _)| part.0) {
                    selected_entities.selected_robots.push(robot);
                    selected_entities.active_robot = Some(robot);
                }
            }
        }
        EntitySelectionMode::SelectRobotJointsLocal => 'joints_local: {
            if selected_entities.active_robot.is_none() {
                break 'joints_local;
            }
            if !mouse.just_pressed(MouseButton::Left) {
                break 'joints_local;
            }
            let shiftclick =
                keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight);

            let robot = selected_entities.active_robot.unwrap();
            if let Some(ent) = selected_entities.hovered_entities.first().copied() {
                let part = robot_part_q.get(ent);
                // If the selected entity isn't a part of the active robot, don't select it.
                if !part.map_or_else(|_| false, |(part, _)| part.0 == robot) {
                    break 'joints_local;
                }
                let (part, parent) = part.unwrap();

                // If clicked robot part isn't a joint, don't select it.
                let joint_ent = if robot_joint_q.contains(ent) {
                    ent
                } else if parent.is_some_and(|par| robot_joint_q.contains(par.get())) {
                    parent.unwrap().get()
                } else {
                    break 'joints_local;
                };

                if shiftclick {
                    if let Some(idx) = selected_entities
                        .selected_joints
                        .iter()
                        .position(|e| *e == joint_ent)
                    {
                        println!("deselect");
                        selected_entities.selected_joints.remove(idx);
                    } else {
                        println!("select");
                        selected_entities.selected_joints.push(joint_ent);
                    }
                } else {
                    println!("select only one");
                    selected_entities.selected_joints.clear();
                    selected_entities.selected_joints.push(joint_ent);
                }
            } else if !shiftclick {
                println!("deselect all");
                selected_entities.selected_joints.clear();
            }
        }
        EntitySelectionMode::SelectSerialChainLocal {
            ref mut selection_root,
            ref mut selected_joints,
            ref mut selection_preview_joints,
            ref mut hovered_joint,
        } => {
            // Selecting root
            if !alt_pressed && hovered_joint.is_some() {
                *selection_root = Some(hovered_joint.unwrap());
                selected_joints.clear();
            }
            // Selecting joint chain
            else if selection_root.is_some() && alt_pressed {
                selected_joints.clear();
                selected_joints.append(&mut selection_preview_joints.clone());
            }
            // If nothing was under pointer when clicking
            else {
                selected_joints.clear();
                *selection_root = None;
            }
        }
        _ => todo!(),
    }
}
