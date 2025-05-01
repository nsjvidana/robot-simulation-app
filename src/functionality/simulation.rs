use bevy::app::App;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::ecs::system::SystemState;
use bevy::prelude::*;
use bevy_rapier3d::plugin::systems::{writeback_rigid_bodies, RigidBodyWritebackComponents};
use bevy_rapier3d::prelude::*;
use bevy_salva3d::fluid::{FluidAccelerations, FluidPositions, FluidVelocities};
use bevy_salva3d::plugin::{DefaultSalvaContext, SalvaConfiguration, SalvaPhysicsPlugin};
use derivative::Derivative;

pub fn build_plugin(app: &mut App) {
    app.insert_resource(Time::<Fixed>::from_hz(60.));

    app.init_schedule(PhysicsSchedule);

    if !app.is_plugin_added::<RapierPhysicsPlugin<NoUserData>>() {
        app.insert_resource(TimestepMode::Fixed {
            dt: 1./60.,
            substeps: 1
        });
        app.add_plugins(
            RapierPhysicsPlugin::<NoUserData>::default()
                .in_schedule(PhysicsSchedule)
        );
    }
    if !app.is_plugin_added::<SalvaPhysicsPlugin>() {
        app.insert_resource(bevy_salva3d::plugin::TimestepMode::Fixed {
            dt: 1./60.,
            substeps: 1
        });
        app.add_plugins(
            SalvaPhysicsPlugin::default()
                .in_schedule(PhysicsSchedule)
        );
    }

    app.init_state::<SimulationState>();

    app
        .init_resource::<ResetSimulationSnapshot>()
        .init_resource::<SimulationRunnerParameters>();

    app.add_systems(
        PostStartup,
        |mut config: Single<&mut RapierConfiguration, With<DefaultRapierContext>>|
            config.physics_pipeline_active = false
    );

    app.add_systems(
        PostUpdate,
        (
            update_fluid_entity_positions,
            simulation_runner
        )
            .chain()
    );
}

#[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PhysicsSchedule;

#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SimulationState {
    Playing,
    Paused,
    Stepped,
    #[default]
    Reset,
}

#[derive(Resource, Default, Clone)]
pub struct ResetSimulationSnapshot {
    pub rapier: Vec<u8>,
    /// Contains Vec<(Entity, FluidPositions, FluidVelocities, FluidAccelerations)>.
    pub salva: Vec<u8>,
}

#[derive(Resource, Derivative)]
#[derivative(Default)]
pub struct SimulationRunnerParameters {
    #[derivative(Default(value = "60."))]
    pub frames_per_second: f64,
    prev_state: SimulationState,
    sim_to_render_diff: f64,
}

/// Emulate a FixedUpdate for the physics schedule to prevent synchronization issues with UI changes.
///
/// [`TimestepMode`] resource MUST be [`TimestepMode::Fixed`].
pub fn simulation_runner(world: &mut World) {
    let (fixed_timestep_dt, prev_sim_state) = {
        let res = world.resource::<SimulationRunnerParameters>();
        let dt =
            if res.frames_per_second != 0. { 1./res.frames_per_second }
            else { 0. };
        (dt,  res.prev_state)
    };
    let sim_state = **world.resource::<State<SimulationState>>();
    let sim_state_changed = world.resource_mut::<State<SimulationState>>().is_changed();

    // Upon exiting Reset state
    if sim_state_changed {
        match sim_state {
            _ =>
                if prev_sim_state == SimulationState::Reset {
                    on_exit_reset(world);
                }
        }
    }

    let mut rapier_config = world.query_filtered::<&mut RapierConfiguration, With<DefaultRapierContext>>();

    let mut run_physics_schedule = |world: &mut World| {
        let mut sim_to_render_diff = world.resource::<SimulationRunnerParameters>().sim_to_render_diff;
            sim_to_render_diff += world.resource::<Time>().delta_secs_f64();
        while sim_to_render_diff > 0. {
            world.run_schedule(PhysicsSchedule);
            sim_to_render_diff -= fixed_timestep_dt;
        }
        world.resource_mut::<SimulationRunnerParameters>().sim_to_render_diff = sim_to_render_diff;
    };

    match sim_state {
        SimulationState::Playing => {
            rapier_config.single_mut(world).physics_pipeline_active = true;
            run_physics_schedule(world);
        },
        SimulationState::Stepped => {
            rapier_config.single_mut(world).physics_pipeline_active = true;
            run_physics_schedule(world);
            world.commands().set_state(SimulationState::Paused);
        },
        SimulationState::Paused | SimulationState::Reset => {
            rapier_config.single_mut(world).physics_pipeline_active = false;
            run_physics_schedule(world);
            // Force rigid body writeback
            {
                rapier_config.single_mut(world).physics_pipeline_active = true;
                let mut sys_state: SystemState<(
                    Query<&mut RapierRigidBodySet>,
                    Res<TimestepMode>,
                    Query<&RapierConfiguration>,
                    Query<&SimulationToRenderTime>,
                    Query<&GlobalTransform>,
                    Query<
                        RigidBodyWritebackComponents,
                        (With<RigidBody>, Without<RigidBodyDisabled>),
                    >
                )> = SystemState::new(world);
                let (
                    rigid_body_sets,
                    timestep_mode,
                    config,
                    sim_to_render_time,
                    global_transforms,
                    writeback,
                ) = sys_state.get_mut(world);
                writeback_rigid_bodies(
                    rigid_body_sets,
                    timestep_mode,
                    config,
                    sim_to_render_time,
                    global_transforms,
                    writeback,
                );
                rapier_config.single_mut(world).physics_pipeline_active = false;
            }
        },
    }

    // Upon entering Reset state
    if sim_state_changed {
        match sim_state {
            SimulationState::Reset =>
                if prev_sim_state != SimulationState::Reset {
                    on_enter_reset(world);
                    rapier_config.single_mut(world).physics_pipeline_active = true;
                    let mut sys_state: SystemState<(
                        Query<&mut RapierRigidBodySet>,
                        Res<TimestepMode>,
                        Query<&RapierConfiguration>,
                        Query<&SimulationToRenderTime>,
                        Query<&GlobalTransform>,
                        Query<
                            RigidBodyWritebackComponents,
                            (With<RigidBody>, Without<RigidBodyDisabled>),
                        >
                    )> = SystemState::new(world);
                    let (
                        rigid_body_sets,
                        timestep_mode,
                        config,
                        sim_to_render_time,
                        global_transforms,
                        writeback,
                    ) = sys_state.get_mut(world);
                    writeback_rigid_bodies(
                        rigid_body_sets,
                       timestep_mode,
                       config,
                       sim_to_render_time,
                       global_transforms,
                       writeback,
                    );
                    rapier_config.single_mut(world).physics_pipeline_active = false;
                },
            _ => {}
        }
    }

    world.resource_mut::<SimulationRunnerParameters>().prev_state = sim_state;
}

fn on_enter_reset(world: &mut World) {
    if !world.resource::<ResetSimulationSnapshot>().rapier.is_empty() {
        let (
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        ): (
            RapierContextSimulation,
            RapierContextColliders,
            RapierContextJoints,
            RapierQueryPipeline,
            RapierRigidBodySet,
        ) = bincode::deserialize(&world.resource::<ResetSimulationSnapshot>().rapier).unwrap();

        let mut ctx_q = world.query_filtered::<
            (
                &'static mut RapierContextSimulation,
                &'static mut RapierContextColliders,
                &'static mut RapierContextJoints,
                &'static mut RapierQueryPipeline,
                &'static mut RapierRigidBodySet,
            ),
            With<DefaultRapierContext>
        >();
        let (
            mut ctx_simulation,
            mut ctx_colliders,
            mut ctx_joints,
            mut ctx_query_pipeline,
            mut ctx_rigidbody_set,
        ) = ctx_q.single_mut(world);

        *ctx_simulation = simulation;
        *ctx_colliders = colliders;
        *ctx_joints = joints;
        *ctx_query_pipeline = query_pipeline;
        *ctx_rigidbody_set = rigidbody_set;
    }

    if !world.resource::<ResetSimulationSnapshot>().salva.is_empty() {
        let salva_ser_data: Vec<(Entity, FluidPositions, FluidVelocities, FluidAccelerations)> =
            bincode::deserialize(&world.resource::<ResetSimulationSnapshot>().salva)
                .unwrap();
        let mut fluids_q = world.query::<(
            &mut FluidPositions,
            &mut FluidVelocities,
            &mut FluidAccelerations,
        )>();
        for (entity, positions, vels, accs) in salva_ser_data {
            let (
                mut curr_positions,
                mut curr_vels,
                mut curr_accs
            ) = fluids_q.get_mut(world, entity)
                .unwrap();
            *curr_positions = positions;
            *curr_vels = vels;
            *curr_accs = accs;
        }
    }
}

fn on_exit_reset(world: &mut World) {
    let mut rapier_q = world.query_filtered::<RapierContext, With<DefaultRapierContext>>();
    let RapierContextItem {
        simulation,
        colliders,
        joints,
        query_pipeline,
        rigidbody_set,
    } = rapier_q.single(world);
    world.resource_mut::<ResetSimulationSnapshot>().rapier = bincode::serialize(&(
        simulation,
        colliders,
        joints,
        query_pipeline,
        rigidbody_set,
    )).unwrap();

    let mut fluids_q = world.query::<(
        Entity,
        &FluidPositions,
        &FluidVelocities,
        &FluidAccelerations,
    )>();
    let salva_ser_data = fluids_q
        .iter(world)
        .map(|(entity, pos, vel, acc)| (entity, pos.clone(), vel.clone(), acc.clone()))
        .collect::<Vec<_>>();
    world.resource_mut::<ResetSimulationSnapshot>().salva = bincode::serialize(&salva_ser_data)
        .unwrap();
}

pub fn update_fluid_entity_positions(
    mut fluid_positions: Query<(&mut FluidPositions, &mut GlobalTransform)>,
) {
    // for (positions, mut transform) in fluid_positions.iter_mut() {
    //     if positions.is_empty() { continue; };
    //     let center = Aabb3d::from_point_cloud(
    //         Isometry3d::IDENTITY,
    //         positions
    //             .iter()
    //             .copied()
    //     )
    //         .center();
    //     *transform = GlobalTransform::from_translation(center.into());
    // }
}
