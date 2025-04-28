use crate::error::ErrorEvent;
use crate::functionality::RobotLabPlugin;
use crate::ui::selecting::PickingExt;
use crate::ui::RobotLabUiPlugin;
use bevy::prelude::*;
use bevy::DefaultPlugins;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings};
use bevy_rapier3d::prelude::RapierPickable;
use bevy_salva3d::fluid::FluidPositions;
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use bevy_salva3d::utils::cube_particle_positions;

pub mod ui;
pub mod functionality;
pub mod error;
pub mod prelude;
mod convert;
mod math;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        // WorldInspectorPlugin::default(),
        RobotLabPlugin,
        RobotLabUiPlugin,
        InfiniteGridPlugin,
        NoCameraPlayerPlugin,
    ));

    app.add_event::<ErrorEvent>();

    app.add_systems(Startup, startup);
    app.add_systems(Update, |fluids: Query<&FluidPositions>, mut gizmos: Gizmos| {
        for positions in fluids.iter() {
            for pos in positions.iter().copied() {
                gizmos.sphere(
                    pos,
                    SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS,
                    Color::linear_rgb(1., 1., 0.)
                );
            }
        }
    });

    app.add_systems(Update, |mut errs: EventReader<ErrorEvent>| {
        for err in errs.read() {
            println!("{}", err.error)
        }
    });

    app.run();
}

pub fn startup(
    mut commands: Commands,
) {
    // Grid
    commands.spawn(InfiniteGridBundle {
        settings: InfiniteGridSettings {
            x_axis_color: Color::linear_rgb(1., 0., 0.),
            z_axis_color: Color::linear_rgb(0., 0., 1.),
            ..Default::default()
        },
        ..Default::default()
    });

    // Main directional light
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(3.0, 3.0, 3.0)
            .looking_at(Vec3::ZERO, Vec3::Y)
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, -3.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        FlyCam,
        RapierPickable,
    ));

    commands.spawn(FluidPositions(cube_particle_positions(10, 10, 10, SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS)));
}
