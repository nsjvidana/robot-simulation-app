use crate::error::ErrorEvent;
use crate::functionality::RobotLabPlugin;
use crate::ui::selecting::PickingExt;
use crate::ui::{RobotLabUiPlugin, VisualEntity};
use bevy::prelude::*;
use bevy::DefaultPlugins;
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::geometry::Collider;
use bevy_rapier3d::prelude::RapierPickable;
use bevy_salva3d::fluid::{FluidNonPressureForces, FluidPositions};
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use bevy_salva3d::rapier_integration::{ColliderSamplingMethod, RapierColliderSampling};
use bevy_salva3d::salva::solver::{Akinci2013SurfaceTension, ArtificialViscosity};
use bevy_salva3d::utils::cube_particle_positions;
use crate::ui::generic_object::GenericObject;

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
        WorldInspectorPlugin::default(),
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
                    // Color::linear_rgba(0.83984375, 0.8203125, 0.79296875, 0.1)
                    Color::linear_rgba(0.1, 0.1, 1., 0.5)
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
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
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

    // let mut positions = cube_particle_positions(50, 20, 50, SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS);
    // commands.spawn(FluidPositions(positions));

    //     positions.iter_mut()
    //         .for_each(|v| *v += Vec3::Y*2.);
    // let surface_tension = Akinci2013SurfaceTension::new(1.0, 0.0);
    // let viscosity = ArtificialViscosity::new(0.01, 0.01);
    // commands.spawn((
    //     FluidPositions(positions),
    //     FluidNonPressureForces(vec![
    //         Box::new(surface_tension),
    //         Box::new(viscosity)
    //     ]),
    // ));
    //
    // let white_mat = materials.add(Color::WHITE);
    //
    // //ground
    // commands
    //     .spawn((
    //         Name::new("Ground"),
    //         RigidBody::Fixed,
    //         Collider::cuboid(5., 0.5, 5.),
    //         Transform::from_translation(Vec3::new(0., -0.5, 0.)),
    //         InheritedVisibility::default(),
    //         VisualEntity,
    //         Mesh3d(meshes.add(Cuboid::new(10., 1., 10.))),
    //         MeshMaterial3d(white_mat.clone()),
    //         GenericObject,
    //         RapierColliderSampling {
    //             sampling_method: ColliderSamplingMethod::DynamicContact
    //         }
    //     ))
    //     .make_entity_pickable();
}
