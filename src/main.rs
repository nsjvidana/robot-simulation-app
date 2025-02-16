use crate::robot::systems::sync_robot_changes;
use crate::robot::{Robot, RobotPlugin};
use crate::ui::{edit_timestep_mode, RobotLabUiPlugin};
use bevy::prelude::{ButtonInput, FixedUpdate, IntoSystemConfigs, KeyCode, Query, Res, Startup, Update, With};
use bevy::{app::App, math::Vec3, prelude::{Camera3d, Commands, Component, Transform}, DefaultPlugins};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::plugin::systems::{init_colliders, init_joints, init_rigid_bodies};
use bevy_rapier3d::plugin::PhysicsSet;
use bevy_rapier3d::prelude::{RapierDebugRenderPlugin, TimestepMode};
use bevy_rapier3d::{plugin::RapierPhysicsPlugin, prelude::{Collider, RigidBody}};
use k::SerialChain;
use math::Real;
use std::ops::DerefMut;

mod kinematics;
mod math;
#[allow(clippy::too_many_arguments)]
mod ui;
mod convert;
mod robot;

fn main() {
    let mut app = App::new();

    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1
    });

    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default()
            .in_schedule(FixedUpdate),
        RapierDebugRenderPlugin::default(),
        // SalvaPhysicsPlugin::new(),
        WorldInspectorPlugin::default(),
        // EguiPlugin,
        RobotLabUiPlugin::new(Update, FixedUpdate),
        RobotPlugin,
        NoCameraPlayerPlugin,
    ));

    app.add_systems(Startup, startup);
    app.add_systems(Update, update);

    app.add_systems(FixedUpdate, (
        edit_timestep_mode.before(PhysicsSet::SyncBackend),
        sync_robot_changes.before(PhysicsSet::SyncBackend),
        robot::systems::init_robots
            .in_set(PhysicsSet::SyncBackend)
            .after(init_rigid_bodies)
            .after(init_colliders)
            .after(init_joints),
    ));

    app.run();
}

#[derive(Component)]
pub struct TestComponent {
    pub chain: SerialChain<Real>,
}

pub fn update(
    mut robot_q: Query<&mut Transform, With<Robot>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        for mut robot_transform in robot_q.iter_mut() {
            robot_transform.translation.x += 1.;
        }
    }
    // else {
    //     rapier_ctx_q.get_single_mut().unwrap().physics_pipeline_active = false;
    // }
}

pub fn startup(
    mut commands: Commands,
) {
    //camera
    commands.spawn((
        Transform::from_xyz(-2., 2., 2.)
                .looking_at(Vec3::ZERO, Vec3::Y),
        Camera3d::default(),
        FlyCam,
        Collider::ball(0.1)
    ));

    //ground
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
        Transform::from_xyz(0., -5., 0.),
    ));
}
