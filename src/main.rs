use crate::error::ErrorEvent;
use crate::general::GeneralTabPlugin;
use crate::motion_planning::MotionPlanningPlugin;
use crate::prelude::*;
use bevy::app::PostUpdate;
use bevy::ecs::system::SystemParam;
use bevy::prelude::{ButtonInput, EventReader, FixedUpdate, IntoSystemConfigs, KeyCode, Local, NonSendMut, Query, Res, Startup, Time, Update, With};
use bevy::{
    app::App,
    math::Vec3,
    prelude::{Camera3d, Commands, Component, Transform},
    DefaultPlugins,
};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::prelude::{DefaultRapierContext, ImpulseJoint, MultibodyJoint, RapierConfiguration, RapierDebugRenderPlugin, TimestepMode, WriteRapierContext};
use bevy_rapier3d::{
    plugin::RapierPhysicsPlugin,
    prelude::{Collider, RigidBody},
};
use std::ops::DerefMut;
use std::sync::Arc;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use parking_lot::Mutex;
use crate::ui::entity_selection::{EntitySelectionServer, SelectionRequest, SelectionResponse};

pub mod convert;
pub mod error;
pub mod kinematics;
pub mod math;
pub mod prelude;
pub mod robot;
#[allow(clippy::too_many_arguments)]
pub mod ui;
pub mod motion_planning;
mod general;
mod impls;

fn main() {
    let mut app = App::new();

    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });
    app.add_event::<ErrorEvent>();

    app.add_plugins((
        DefaultPlugins,
        InfiniteGridPlugin,
    ));
    app.add_plugins((
        GeneralTabPlugin::new(FixedUpdate),
        MotionPlanningPlugin::new(FixedUpdate),
        RobotLabUiPlugin::new(Update),
        RobotPlugin,
        NoCameraPlayerPlugin,
    ));
    app.add_plugins((
        RapierPhysicsPlugin::<()>::default().in_schedule(FixedUpdate),
        RapierDebugRenderPlugin::default(),
        // SalvaPhysicsPlugin::new(),
        WorldInspectorPlugin::default(),
    ));

    app.add_systems(Startup, startup);
    app.add_systems(Update, update);

    app.add_systems(PostUpdate, |mut events: EventReader<ErrorEvent>| {
        for event in events.read() {
            println!("{:?}", event.error);
        }
    });

    app.run();
}

#[derive(SystemParam)]
pub struct PhysicsData<'w, 's> {
    ctx: WriteRapierContext<'w, 's, ()>,
    ctx_config: Query<'w, 's, &'static mut RapierConfiguration, With<DefaultRapierContext>>,
    time: Res<'w, Time>
}

#[derive(Default)]
struct State {
    resp: Option<Arc<Mutex<SelectionResponse>>>,
}

pub fn update(
    mut state: Local<State>,
    keys: Res<ButtonInput<KeyCode>>,
    mut entity_selection_server: NonSendMut<EntitySelectionServer>
) {
    if keys.just_pressed(KeyCode::KeyP) {
        state.resp = Some(SelectionRequest::new(|entity, resources| {
            resources.joint_q.contains(entity)
        })
            .send(&mut entity_selection_server));
    }
    if let Some(resp) = state.resp.clone() {
        if let Some(entity) = resp.lock().get_selection() {
            println!("{}", entity);
            state.resp = None;
        }
    }
}

pub fn startup(mut commands: Commands) {
    // Camera
    commands.spawn((
        Transform::from_xyz(-10., 10., 10.).looking_at(Vec3::ZERO, Vec3::Y),
        Camera3d::default(),
        FlyCam,
    ));

    // Infinite grid
    commands.spawn(InfiniteGridBundle::default());

    //ground
    // commands.spawn((
    //     RigidBody::Fixed,
    //     Collider::cuboid(10., 0.1, 10.),
    //     Transform::from_xyz(0., -0.1, 0.),
    // ));
}
