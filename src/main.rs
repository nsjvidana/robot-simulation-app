use crate::error::ErrorEvent;
use crate::prelude::*;
use bevy::prelude::{ButtonInput, EventReader, FixedUpdate, IntoSystemConfigs, KeyCode, Query, Res, Startup, Time, Update, With};
use bevy::{
    app::App,
    math::Vec3,
    prelude::{Camera3d, Commands, Component, Transform},
    DefaultPlugins,
};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::{RapierDebugRenderPlugin, TimestepMode, WriteDefaultRapierContext};
use bevy_rapier3d::{
    plugin::RapierPhysicsPlugin,
    prelude::{Collider, RigidBody},
};
use k::SerialChain;
use std::ops::DerefMut;
use std::sync::Arc;
use bevy::app::PostUpdate;
use bevy::ecs::system::SystemParam;
use bevy::gizmos::GizmoPlugin;
use bevy_egui::EguiPlugin;
use parking_lot::Mutex;
use crate::general::GeneralTabPlugin;
use crate::motion_planning::MotionPlanningPlugin;

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

fn main() {
    let mut app = App::new();

    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });

    app.add_event::<ErrorEvent>();


    app.add_plugins(DefaultPlugins);
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
        // WorldInspectorPlugin::default(),
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
    ctx: WriteDefaultRapierContext<'w, 's>,
    time: Res<'w, Time>
}

pub fn update(mut robot_q: Query<&mut Transform, With<Robot>>, keys: Res<ButtonInput<KeyCode>>) {
    if keys.just_pressed(KeyCode::KeyP) {
        for mut robot_transform in robot_q.iter_mut() {
            robot_transform.translation.x += 1.;
        }
    }
    // else {
    //     rapier_ctx_q.get_single_mut().unwrap().physics_pipeline_active = false;
    // }
}

pub fn startup(mut commands: Commands) {
    //camera
    commands.spawn((
        Transform::from_xyz(-2., 2., 2.).looking_at(Vec3::ZERO, Vec3::Y),
        Camera3d::default(),
        FlyCam,
    ));

    //ground
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
        Transform::from_xyz(0., -5., 0.),
    ));
}
