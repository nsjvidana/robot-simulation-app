use crate::math::Real;
use crate::prelude::{Robot, RobotJointType, RobotPart};
use crate::ui::UiGizmoGroup;
use bevy::color::Color;
use bevy::ecs::system::SystemParam;
use bevy::hierarchy::Parent;
use bevy::input::ButtonInput;
use bevy::math::Ray3d;
use bevy::prelude::{Camera, Entity, Gizmos, GlobalTransform, KeyCode, MouseButton, NonSendMut, Or, Query, Res, ResMut, Resource, Single, Window, With};
use bevy_egui::EguiContexts;
use bevy_rapier3d::dynamics::{ImpulseJoint, MultibodyJoint, RapierImpulseJointHandle, RapierMultibodyJointHandle};
use bevy_rapier3d::pipeline::QueryFilter;
use bevy_rapier3d::plugin::{RapierContext, ReadRapierContext};
use std::cmp::Ordering;
use std::sync::Arc;
use parking_lot::Mutex;
use crate::prelude::ik::KinematicNode;

#[derive(Resource, Default)]
pub struct SceneWindowData {
    pub viewport_to_world_ray: Option<Ray3d>,
    pub camera_transform: GlobalTransform,
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

#[derive(SystemParam)]
pub struct EntitySelectionResources<'w, 's> {
    pub joint_q: Query<'w, 's,
        (Option<&'static ImpulseJoint>, Option<&'static MultibodyJoint>),
        Or<(
            With<RapierImpulseJointHandle>,
            With<RapierMultibodyJointHandle>,
        )>,
    >,
    pub robot_q: Query<'w, 's, &'static Robot>,
    pub robot_part_q: Query<'w, 's, (&'static RobotPart, Option<&'static Parent>)>,
    pub kinematic_nodes: Query<'w, 's, &'static KinematicNode>,
    pub selected_entities: ResMut<'w, SelectedEntities>,
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

#[derive(Default)]
pub struct EntitySelectionServer {
    pub requests: Vec<SelectionRequest>,
}

pub struct SelectionRequest {
    pub selection_filter: Box<dyn Fn(Entity, &EntitySelectionResources) -> bool>,
    response: Arc<Mutex<SelectionResponse>>,
}

impl SelectionRequest {
    pub fn new(selection_filter: impl Fn(Entity, &EntitySelectionResources) -> bool + 'static) -> Self {
        Self {
            selection_filter: Box::new(selection_filter),
            response: Arc::new(Mutex::new(SelectionResponse::default())),
        }
    }

    pub fn send(self, request_writer: &mut EntitySelectionServer) -> Arc<Mutex<SelectionResponse>> {
        let resp = self.response.clone();
        request_writer.requests.push(self);
        resp
    }
}

#[derive(Default)]
pub struct SelectionResponse {
    /// Will be a Some when an entity is selected
    entity: Option<Entity>,
    shift_click: bool,
    alt_click: bool,
    canceled: bool,
}

impl SelectionResponse {
    pub fn cancel_response(&mut self) {
        self.canceled = true;
    }

    pub fn get_selection(&self) -> Option<Entity> {
        self.entity
    }

    pub fn is_canceled(&self) -> bool {
        self.canceled
    }

    pub fn shift_selected(&mut self) -> bool {
        self.entity.is_some() && self.shift_click
    }
    pub fn alt_selected(&mut self) -> bool {
        self.entity.is_some() && self.alt_click
    }
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
    scene_window_data: Res<SceneWindowData>,
    transform_q: Query<&GlobalTransform>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut entity_selection_resources: EntitySelectionResources,
    mut gizmos: Gizmos<UiGizmoGroup>,
) {
    let robot_q = &entity_selection_resources.robot_q;
    let robot_part_q = &entity_selection_resources.robot_part_q;
    let robot_joint_q = &entity_selection_resources.joint_q;
    let selected_entities = &mut entity_selection_resources.selected_entities;

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

pub fn handle_selection_requests(
    mut entity_selection_resources: EntitySelectionResources,
    mut selection_server: NonSendMut<EntitySelectionServer>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if !entity_selection_resources.selected_entities.viewport_clicked
        || matches!(
            entity_selection_resources.selected_entities.pointer_usage_state,
            PointerUsageState::UsingTool
        )
    {
        return;
    }

    let alt_pressed = keyboard.pressed(KeyCode::AltLeft) || keyboard.pressed(KeyCode::AltRight);
    let shift_pressed =
        keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight);

    let mut requests = Vec::new();
    requests.append(&mut selection_server.requests);
    for request in requests.into_iter() {
        let mut resp = request.response.lock();
        // Don't add this request back if it was canceled
        if resp.is_canceled() {
            continue;
        }

        let Some(clicked_entity) = entity_selection_resources
            .selected_entities
            .hovered_entities
            .first()
            .cloned()
        else {
            // If no entity was clicked, add this request back and leave it alone.
            std::mem::drop(resp);
            selection_server.requests.push(request);
            continue;
        };

        // If the clicked entity meets the selection critera for this request, finish the request
        // and don't add it back since it has been dealt with.
        if (request.selection_filter)(clicked_entity, &entity_selection_resources) {
            resp.entity = Some(clicked_entity);
            resp.shift_click = shift_pressed;
            resp.alt_click = alt_pressed;
        }
        // If criteria not met, add the request back to the list for it to be completed later on.
        else {
            std::mem::drop(resp);
            selection_server.requests.push(request);
        }
    }
}
