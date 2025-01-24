use bevy::{app::App, math::Vec3, prelude::{Camera3d, Commands, Component, Transform}, DefaultPlugins};
use bevy_flycam::FlyCam;
use bevy_rapier3d::{plugin::RapierPhysicsPlugin, prelude::{Collider, RigidBody}};
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use k::SerialChain;
use math::Real;

// mod error;
mod kinematics;
mod math;

fn main() {
    let mut app = App::new()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<()>::default(),
            SalvaPhysicsPlugin::new()
        ));
}

#[derive(Component)]
pub struct TestComponent {
    pub chain: SerialChain<Real>,
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
