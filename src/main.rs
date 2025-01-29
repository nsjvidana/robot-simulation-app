use std::path::Path;
use bevy::{app::App, math::Vec3, prelude::{Camera3d, Commands, Component, Transform}, DefaultPlugins};
use bevy::prelude::{ButtonInput, FixedUpdate, IntoSystemConfigs, KeyCode, Query, Res, Startup, Update, With};
use bevy_egui::EguiPlugin;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_rapier3d::{plugin::RapierPhysicsPlugin, prelude::{Collider, RigidBody}};
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use k::SerialChain;
use math::Real;
use std::fs;
use bevy_rapier3d::plugin::PhysicsSet;
use bevy_rapier3d::prelude::{DefaultRapierContext, RapierConfiguration, RapierDebugRenderPlugin, WriteDefaultRapierContext};

mod kinematics;
mod math;
mod ui;
mod urdf;
mod convert;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default().in_fixed_schedule(),
        RapierDebugRenderPlugin::default(),
        SalvaPhysicsPlugin::new(),
        EguiPlugin,
        NoCameraPlayerPlugin,
    ));

    app.add_systems(Startup, startup);
    app.add_systems(Update, (
        ui::ik_sandbox_ui,
        // update
    ));

    app.add_systems(FixedUpdate, urdf::init_robots.before(PhysicsSet::SyncBackend));

    app.run();
}

#[derive(Component)]
pub struct TestComponent {
    pub chain: SerialChain<Real>,
}

pub fn update(
    mut rapier_ctx_q: Query<&mut RapierConfiguration, With<DefaultRapierContext>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        rapier_ctx_q.get_single_mut().unwrap().physics_pipeline_active = true;
    }
    else {
        rapier_ctx_q.get_single_mut().unwrap().physics_pipeline_active = false;
    }
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
