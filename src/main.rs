use crate::error::ErrorEvent;
use crate::functionality::RobotLabPlugin;
use crate::ui::selecting::PickingExt;
use crate::ui::RobotLabUiPlugin;
use bevy::prelude::*;
use bevy::DefaultPlugins;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings};
use bevy_rapier3d::prelude::{MultibodyJoint, RapierPickable, TypedJoint};
use bevy_rapier3d::rapier::prelude::SPATIAL_DIM;
use rand::Rng;
use crate::functionality::simulation::PhysicsSchedule;

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

    app.add_systems(PhysicsSchedule, |mut mbjs: Query<&mut MultibodyJoint>| {
        for mut mbj in mbjs.iter_mut() {
            let j = &mut mbj.data.as_mut().raw;
            for motor_idx in 0..SPATIAL_DIM {
                if (j.motor_axes.bits() & 1 << motor_idx) != 0 {
                    let limit = j.limits[motor_idx];
                    j.motors[motor_idx].target_pos = rand::thread_rng().gen_range(limit.min..limit.max);
                }
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
}
