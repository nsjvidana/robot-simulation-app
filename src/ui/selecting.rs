use crate::functionality::robot::{KinematicNode, RobotPart};
use crate::ui::generic_object::GenericObject;
use crate::ui::robot_selection::RobotSelection;
use crate::ui::{generic_object, robot_selection};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_egui::EguiContexts;
use bevy_rapier3d::prelude::{ImpulseJoint, MultibodyJoint, RapierPickable};
use parking_lot::Mutex;
use std::sync::Arc;
use bevy_salva3d::fluid::FluidPositions;

pub fn build_app(app: &mut App) {
    app.init_non_send_resource::<PickingServer>();

    app
        .init_resource::<SceneWindowData>()
        .init_resource::<Events<PickingEvent>>()
        .init_resource::<Events<UnpickEvent>>();

    app.configure_sets(
        Update,
        (SelectingSet, MakeSelectionsSet).chain()
    );

    app.add_systems(
        Update,
        (
            update_scene_window_data,
            selections_system,
        )
            .chain()
            .in_set(SelectingSet)
    );

    robot_selection::build_app(app);
    generic_object::build_app(app);
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct SelectingSet;
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct MakeSelectionsSet;

#[derive(Resource, Default)]
pub struct SceneWindowData {
    pub viewport_to_world_ray: Option<Ray3d>,
    pub viewport_clicked: bool,
    pub viewport_contains_pointer: bool,
}

#[derive(SystemParam)]
pub struct PickingFilterResources<'w, 's> {
    pub multibody_joints: Query<'w, 's, &'static MultibodyJoint>,
    pub impulse_joint: Query<'w, 's, &'static ImpulseJoint>,
    pub kinematic_nodes: Query<'w, 's, &'static KinematicNode>,
    pub robot_parts: Query<'w, 's, &'static RobotPart>,
    pub generic_objects: Query<'w, 's, (), With<GenericObject>>,
    pub selections: Res<'w, RobotSelection>,
    pub fluids: Query<'w, 's, &'static FluidPositions>,
}

#[derive(Event, Debug, Clone)]
pub struct PickingEvent {
    pub pointer_event: Pointer<Click>,
    pub entity: Entity,
}

#[derive(Event, Debug, Clone)]
pub struct UnpickEvent;

#[derive(Default)]
pub struct PickingServer {
    pub requests: Vec<PickingRequest>,
}

pub struct PickingRequest {
    picking_filter: Box<dyn Fn(&PickingFilterResources, &PickingEvent) -> bool + Send + Sync>,
    pub continuous: bool,
    prevent_blocking: bool,
    priority: Option<usize>,
    resp: PickingResponse,
}

impl PickingRequest {
    pub fn new(picking_filter: impl Fn(&PickingFilterResources, &PickingEvent) -> bool + Send + Sync + 'static) -> Self {
        Self {
            picking_filter: Box::new(picking_filter),
            resp: PickingResponse::default(),
            continuous: false,
            prevent_blocking: false,
            priority: None,
        }
    }

    /// Sets whether the request will constantly produce outputs.
    pub fn continuous(mut self, continuous: bool) -> Self {
        self.continuous = continuous;
        self
    }

    /// If the user clicks on nothing, [`PickingResponse::unpicked()`] will be true
    pub fn detect_unpicking(mut self, detect_unpicking: bool) -> Self {
        if detect_unpicking {
            self.resp.unpicked = Some(Default::default());
        }
        else {
            self.resp.unpicked = None;
        }
        self
    }
    
    /// Prevent blocking requests of certain priorities.
    pub fn prevent_blocking(mut self, prevent_blocking: bool) -> Self {
        self.prevent_blocking = prevent_blocking;
        self
    }

    pub fn at_priority(mut self, priority: usize) -> Self {
        self.priority = Some(priority);
        self
    }

    pub fn response(&self) -> PickingResponse {
        self.resp.clone()
    }
}

#[derive(Default, Debug, Clone)]
pub struct PickingResponse {
    picking_response: Arc<Mutex<Option<EntityPickingData>>>,
    unpicked: Option<Arc<Mutex<bool>>>,
    canceled: Arc<Mutex<bool>>,
}

impl PickingResponse {
    pub fn cancel_request(&self) {
        *self.canceled.lock() = true;
    }

    pub fn take_response(&self) -> Option<EntityPickingData> {
        self.picking_response.lock().take()
    }

    /// True if this response supports unpicking and if the user clicked on nothing
    pub fn unpicked(&self) -> bool {
        self.unpicked.as_ref().is_some_and(|v| *v.lock())
    }

    pub fn reset_unpicked(&self) {
        if let Some(unpicked) = self.unpicked.as_ref() {
            *unpicked.lock() = false;
        }
    }
}

#[derive(Debug)]
pub struct EntityPickingData {
    pub event: PickingEvent,
    pub shift_click: bool,
    pub alt_click: bool,
}

impl EntityPickingData {
    pub fn entity(&self) -> Entity {
        self.event.entity
    }
}

pub struct PickingRequestCommand {
    request: PickingRequest
}

impl Command for PickingRequestCommand {
    fn apply(self, world: &mut World) {
        let mut s = world.non_send_resource_mut::<PickingServer>();
        if let Some(priority) = self.request.priority {
            s.requests.insert(priority, self.request);
        }
        else {
            s.requests.push(self.request);
        }
    }
}

impl<'w, 's> PickingRequestCommandExt for Commands<'w, 's> {
    fn send_picking_request(
        &mut self,
        request: PickingRequest
    ) -> PickingResponse {
        let resp = request.response();
        self.queue(PickingRequestCommand {
            request,
        });
        resp
    }
}

impl PickingRequestCommandExt for World {
    fn send_picking_request(
        &mut self,
        request: PickingRequest
    ) -> PickingResponse {
        let resp =  request.response();
        let mut s = self.non_send_resource_mut::<PickingServer>();
        if let Some(priority) = request.priority {
            if priority <= s.requests.len() {
                s.requests.insert(priority, request);
            }
            else  { s.requests.push(request); }
        }
        else {
            s.requests.push(request);
        }
        resp
    }
}

pub trait PickingRequestCommandExt {
    fn send_picking_request(
        &mut self,
        request: PickingRequest
    ) -> PickingResponse;
}

impl<'a> PickingExt for EntityCommands<'a> {
    fn make_entity_pickable(&mut self) -> &mut Self {
        self
            .insert((Visibility::default(), RapierPickable))
            .observe(
                |
                     trigger: Trigger<Pointer<Click>>,
                     mut picking_events: EventWriter<PickingEvent>,
                | {
                    picking_events.send(PickingEvent {
                        pointer_event: trigger.event().clone(),
                        entity: trigger.entity(),
                    });
                }
            )
        ;
        self
    }
}

pub trait PickingExt {
    fn make_entity_pickable(&mut self) -> &mut Self;
}

pub fn selections_system(
    mut picking_server: NonSendMut<PickingServer>,
    mut picking_events: ResMut<Events<PickingEvent>>,
    mut unpick_events: ResMut<Events<UnpickEvent>>,
    picking_filter_resources: PickingFilterResources,
    scene_window_data: Res<SceneWindowData>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
) {
    let mut entity_was_picked = false;

    let mut requests = Vec::with_capacity(picking_server.requests.len());
    loop {
        if let Some(request) = picking_server.requests.pop() {
            let prevent_blocking = request.prevent_blocking;
            let mut cursor = picking_events.get_cursor();
            if let Some(event) = cursor
                .read(&picking_events)
                .find(|event| (&request.picking_filter)(&picking_filter_resources, &event))
                .cloned()
            {
                *request.resp.picking_response.lock() = Some(EntityPickingData {
                    event,
                    shift_click: keyboard.pressed(KeyCode::ShiftLeft),
                    alt_click: keyboard.pressed(KeyCode::AltLeft),
                });
                if request.continuous {
                    requests.push(request);
                }
                entity_was_picked = true;
            }
            else { requests.push(request); }
            if !prevent_blocking {
                break;
            }
        }
        else {
            break;
        }
    }
    requests.reverse();
    picking_server.requests.append(&mut requests);

    let mut requests = Vec::new();
    requests.append(&mut picking_server.requests);
    for request in requests {
        if !*request.resp.canceled.lock() {
            picking_server.requests.push(request);
        }
    }

    // Reset unpicked state if the previous frame had unpick events.
    if !unpick_events.is_empty() {
        for request in &mut picking_server.requests {
            request.resp.reset_unpicked();
        }
    }
    unpick_events.clear();
    unpick_events.update();

    if scene_window_data.viewport_contains_pointer &&
        mouse.just_released(MouseButton::Left) &&
        !entity_was_picked
    {
        unpick_events.send(UnpickEvent);
        let mut requests = Vec::with_capacity(picking_server.requests.len());
        loop {
            if let Some(request) = picking_server.requests.pop() {
                let prevent_blocking = request.prevent_blocking;
                if let Some(unpicked) = &request.resp.unpicked {
                    *unpicked.lock() = true;
                }
                requests.push(request);
                if !prevent_blocking {
                    break;
                }
            }
            else {
                break;
            }
        }
        requests.reverse();
        picking_server.requests.append(&mut requests);
    }

    //Wipe events for next frame
    picking_events.update();
    picking_events.clear();

    // TODO: implement deleting robots.
}

pub fn update_scene_window_data(
    mut scene_window_data: ResMut<SceneWindowData>,
    mut egui_ctxs: EguiContexts,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<(&Camera, &GlobalTransform)>,
    window: Single<&Window>,
) {
    let (camera, camera_transform) = *camera;
    scene_window_data.viewport_to_world_ray = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok());
    scene_window_data.viewport_contains_pointer = !egui_ctxs.ctx_mut().is_pointer_over_area();
    scene_window_data.viewport_clicked = mouse.just_released(MouseButton::Left) &&
        scene_window_data.viewport_contains_pointer;
}