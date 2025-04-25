use bevy::app::App;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::ecs::system::SystemState;
use bevy::prelude::*;
use bevy_rapier3d::plugin::systems::{writeback_rigid_bodies, RigidBodyWritebackComponents};
use bevy_rapier3d::prelude::*;
use crate::functionality::robot::RobotLinks;

pub fn build_plugin(app: &mut App) {
    app.insert_resource(TimestepMode::Fixed {
        dt: 1./60.,
        substeps: 1
    });
    app.insert_resource(Time::<Fixed>::from_hz(60.));

    app.init_schedule(PhysicsSchedule);

    if !app.is_plugin_added::<RapierPhysicsPlugin<NoUserData>>() {
        app.add_plugins(
            RapierPhysicsPlugin::<NoUserData>::default()
                .in_schedule(PhysicsSchedule)
        );
    }

    app.init_state::<SimulationState>();

    app
        .init_resource::<ResetSimulationSnapshot>()
        .init_resource::<PreviousSimulationState>();

    app.add_systems(
        PostStartup,
        |mut config: Single<&mut RapierConfiguration, With<DefaultRapierContext>>|
            config.physics_pipeline_active = false
    );

    app.add_systems(
        FixedUpdate,
        simulation_runner
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
pub struct ResetSimulationSnapshot(pub Vec<u8>);

#[derive(Resource, Default)]
struct PreviousSimulationState(SimulationState);

pub fn simulation_runner(world: &mut World) {
    let prev_sim_state = world.resource::<PreviousSimulationState>().0.clone();
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

    let mut config_q = world.query_filtered::<&mut RapierConfiguration, With<DefaultRapierContext>>();
    match sim_state {
        SimulationState::Playing => {
            config_q.single_mut(world).physics_pipeline_active = true;
            world.run_schedule(PhysicsSchedule)
        },
        SimulationState::Stepped => {
            config_q.single_mut(world).physics_pipeline_active = true;
            world.run_schedule(PhysicsSchedule);
            world.commands().set_state(SimulationState::Paused);
        },
        SimulationState::Paused | SimulationState::Reset => {
            config_q.single_mut(world).physics_pipeline_active = false;
            world.run_schedule(PhysicsSchedule);
            config_q.single_mut(world).physics_pipeline_active = true;
            // let prev_dt =
            //     match &mut *world.resource_mut::<TimestepMode>() {
            //         TimestepMode::Fixed { dt, .. } => { let prev_dt = *dt; *dt = 0.; prev_dt },
            //         TimestepMode::Variable { max_dt: dt, .. } => { let prev_dt = *dt; *dt = 0.; prev_dt },
            //         TimestepMode::Interpolated { dt, .. } => { let prev_dt = *dt; *dt = 0.; prev_dt },
            //     };
            // world.run_schedule(PhysicsSchedule);
            // match &mut *world.resource_mut::<TimestepMode>() {
            //     TimestepMode::Fixed { dt, .. } => { *dt = prev_dt },
            //     TimestepMode::Variable { max_dt: dt, .. } => { *dt = prev_dt },
            //     TimestepMode::Interpolated { dt, .. } => { *dt = prev_dt },
            // };
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
            config_q.single_mut(world).physics_pipeline_active = false;
        },
    }

    // Upon entering Reset state
    if sim_state_changed {
        match sim_state {
            SimulationState::Reset =>
                if prev_sim_state != SimulationState::Reset {
                    on_enter_reset(world);
                    config_q.single_mut(world).physics_pipeline_active = true;
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
                    config_q.single_mut(world).physics_pipeline_active = false;
                },
            _ => {}
        }
    }

    world.resource_mut::<PreviousSimulationState>().0 = sim_state;
}

fn on_enter_reset(world: &mut World) {
    let snapshot = world.resource_mut::<ResetSimulationSnapshot>();
    if !snapshot.0.is_empty() {
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
        ) = bincode::deserialize(&snapshot.0).unwrap();

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
}

fn on_exit_reset(world: &mut World) {
    let mut ctx_q = world.query_filtered::<RapierContext, With<DefaultRapierContext>>();
    let RapierContextItem {
        simulation,
        colliders,
        joints,
        query_pipeline,
        rigidbody_set,
    } = ctx_q.single(world);
    world.resource_mut::<ResetSimulationSnapshot>().0 = bincode::serialize(&(
        simulation,
        colliders,
        joints,
        query_pipeline,
        rigidbody_set,
    )).unwrap();
}
