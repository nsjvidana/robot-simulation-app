use std::ops::DerefMut;
use bevy::app::{App, Update};
use bevy::ecs::intern::Interned;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::math::Vec3;
use crate::kinematics::ik::{ForwardAscentCyclic, ForwardDescentCyclic};
use crate::math::Real;
use crate::robot::{Robot, RobotPart};
use bevy::prelude::{ButtonInput, Camera, Color, Commands, Entity, Gizmos, GlobalTransform, IntoSystemConfigs, Local, MouseButton, Name, Plugin, Query, Res, ResMut, Resource, Single, Window, With, Without};
use bevy_egui::egui::{ComboBox, Ui};
use bevy_egui::{egui, EguiContexts};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use bevy_rapier3d::plugin::TimestepMode;
use bevy_rapier3d::prelude::{DefaultRapierContext, PhysicsSet, QueryFilter, RapierConfiguration, RapierContext, ReadDefaultRapierContext};
use k::{InverseKinematicsSolver, SerialChain};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};

pub struct RobotLabUiPlugin {
    schedule: Interned<dyn ScheduleLabel>,
    physics_schedule: Interned<dyn ScheduleLabel>
}

impl RobotLabUiPlugin {
    pub fn new(schedule: impl ScheduleLabel, physics_schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
            physics_schedule: physics_schedule.intern()
        }
    }
}

impl Plugin for RobotLabUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsSimUi>();

        app.add_systems(self.schedule, robot_sandbox_ui)
            .add_systems(self.physics_schedule, control_physics_sim
                .before(PhysicsSet::StepSimulation)
                .after(PhysicsSet::SyncBackend)
            );
    }
}

//TODO: separate ik, sim, and import stuff into separate structs
pub struct IKSandboxUI {
    pub kinematic_chain: Option<SerialChain<Real>>,
    pub solvers: Vec<Box<dyn InverseKinematicsSolver<Real> + Send>>,
    pub solver_names: Vec<String>,

    pub mb_loader_options: UrdfMultibodyOptions,

    pub selected_robot: Option<Entity>,
    pub selected_entity: Option<Entity>,
}

impl Default for IKSandboxUI {
    fn default() -> Self {
        Self {
            kinematic_chain: None,
            solvers: vec![
                Box::new(ForwardAscentCyclic::default()),
                Box::new(ForwardDescentCyclic::default())
            ],
            solver_names: vec![
                "Forward Ascent Cyclic".to_string(),
                "Forward Descent Cyclic".to_string(),
            ],
            mb_loader_options: UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
            selected_robot: None,
            selected_entity: None,
        }
    }
}

pub struct IKSandboxUIState {
    pub selected_solver_idx: usize,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub selected_joint_type: usize,
}

impl Default for IKSandboxUIState {
    fn default() -> Self {
        Self {
            selected_solver_idx: 0,
            urdf_loader_options: UrdfLoaderOptions {
                create_colliders_from_visual_shapes: true,
                create_colliders_from_collision_shapes: false,
                make_roots_fixed: true,
                // Z-up to Y-up.
                shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
                ..Default::default()
            },
            selected_joint_type: 0
        }
    }
}

#[derive(Resource)]
pub struct PhysicsSimUi {
    pub physics_enabled: bool,
    pub sim_step_pressed: bool,
    pub sim_resetted: bool,
    pub resetted_snapshot: Option<Vec<u8>>,
    pub resetted_snapshot_state: ResettedSnapshotState
}

impl Default for PhysicsSimUi {
    fn default() -> Self {
        Self {
            physics_enabled: false,
            sim_step_pressed: false,
            sim_resetted: true,
            resetted_snapshot: None,
            resetted_snapshot_state: ResettedSnapshotState::Idle,
        }
    }
}

pub enum ResettedSnapshotState {
    LoadSnapshot,
    SaveSnapshot,
    Idle
}

pub fn robot_sandbox_ui(
    mut ctxs: EguiContexts,
    mut ui_data: Local<IKSandboxUI>,
    mut physics_sim_ui_data: ResMut<PhysicsSimUi>,
    mut ui_state: Local<IKSandboxUIState>,
    mut commands: Commands,
    robot_part_q: Query<&RobotPart>,
    transform_q: Query<&GlobalTransform>,
    name_q: Query<&Name>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    window: Single<&Window>,
    default_rapier_context_q: Query<&RapierContext, With<DefaultRapierContext>>,
    mut gizmos: Gizmos,
) {
    let rapier_context = default_rapier_context_q.single();

    if mouse_button_input.just_pressed(MouseButton::Left) {
        let clicked_entity = get_clicked_entity(
            camera,
            window,
            rapier_context,
        );
        if let Some(clicked_entity) = clicked_entity {
            ui_data.selected_entity = Some(clicked_entity);

            if let Ok(robot_entity) = robot_part_q.get(clicked_entity).map(|v| v.0) {
                ui_data.selected_robot = Some(robot_entity);
            }
            else {
                ui_data.selected_robot = None;
            }
        }
        else {
            ui_data.selected_entity = None;
            ui_data.selected_robot = None;
        }
    }

    if let Some(robot_entity) = ui_data.selected_robot {
        //print robot name to console
        if let Ok(name) = name_q.get(robot_entity) {
            println!("{}", name);
        }
        //draw move gizmos
        if let Ok(robot_transform) = transform_q.get(robot_entity) {
            let robot_pos = robot_transform.translation(); 
            gizmos.arrow(robot_pos, robot_pos + Vec3::X, Color::linear_rgba(1., 0., 0., 1.));
            gizmos.arrow(robot_pos, robot_pos + Vec3::Y, Color::linear_rgba(0., 1., 0., 1.));
            gizmos.arrow(robot_pos, robot_pos + Vec3::Z, Color::linear_rgba(1., 0., 1., 1.));
        }
    }

    egui::Window::new("Robot Sandbox").show(
        ctxs.ctx_mut(), |ui| {

            //Robot importing
            robot_import_ui(
                &mut commands,
                ui,
                &mut ui_data,
                &mut ui_state,
            );

            //Inverse Kinematics
            robot_ik_ui(
                ui,
                &mut ui_data,
                &mut ui_state,
            );

            physics_sim_ui(
                ui,
                &mut physics_sim_ui_data,
            );
        }
    );
}

// Gets the entity clicked in the current frame. Returns None if no entity was clicked.
fn get_clicked_entity(
    camera: Single<(&Camera, &GlobalTransform)>,
    window: Single<&Window>,
    rapier_context: &RapierContext
) -> Option<Entity> {
    let (camera, camera_transform) = *camera;
    if let Some(pos) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok())
    {
        if let Some((rb_ent, _)) = rapier_context.cast_ray(
            pos.origin,
            pos.direction.as_vec3(),
            1000.0,
            true,
            QueryFilter::default()
        ) {
            return Some(rb_ent);
        }
    }
    None
}

fn robot_import_ui(
    commands: &mut Commands,
    ui: &mut Ui,
    ui_data: &mut IKSandboxUI,
    ui_state: &mut IKSandboxUIState,
) {
    ui.collapsing("Import Robot", |ui| {
        let _checkbox = ui.checkbox(
            &mut ui_state.urdf_loader_options.make_roots_fixed,
            "Make roots fixed"
        );
        ComboBox::from_label("Robot joint type")
            .selected_text(match ui_state.selected_joint_type {
                0 => "Impulse", 1 => "Multibody", _ => unreachable!()
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut ui_state.selected_joint_type, 0, "Impulse");
                ui.selectable_value(&mut ui_state.selected_joint_type, 1, "Multibody");
            });
        let button = ui.button("Import URDF file");
        if button.clicked() { //import urdf robot via file dialog
            let dialog = rfd::FileDialog::new()
                .add_filter("Robot Description", &["urdf", "URDF"])
                .pick_file();
            if let Some(path) = dialog {
                let robot_name = path.file_stem().unwrap()
                    .to_str().unwrap().to_string();
                let (robot, _) = rapier3d_urdf::UrdfRobot::from_file(
                    path,
                    ui_state.urdf_loader_options.clone(),
                    None
                ).unwrap();
                let robot_cmp = Robot::new(robot);
                commands.spawn((
                    match ui_state.selected_joint_type {
                        0 => robot_cmp.with_impulse_joints(),
                        1 => robot_cmp.with_multibody_joints(ui_data.mb_loader_options),
                        _ => unreachable!()
                    },
                    Name::new(robot_name)
                ));
            }
        }
    });
}

fn robot_ik_ui(
    ui: &mut Ui,
    ui_data: &mut IKSandboxUI,
    ui_state: &mut IKSandboxUIState,
) {
    ui.collapsing("Inverse Kinematics", |ui| {
        let solver_idx = ui_state.selected_solver_idx;
        let selected_solver_name =
            if ui_data.solvers.get(solver_idx).is_some() {
                ui_data.solver_names.get(solver_idx).unwrap()
            }
            else { &"-".to_string() };
        ComboBox::from_label("Solver type")
            .selected_text(selected_solver_name)
            .show_ui(ui, |ui| {
                for (i, solver_name) in ui_data.solver_names.iter().enumerate() {
                    ui.selectable_value(&mut ui_state.selected_solver_idx, i, solver_name);
                }
            });

        ui.horizontal(|ui| {

            ui.label("Chain:");
            let _ = ui.button(
                if ui_data.kinematic_chain.is_some() {"Chain" }
                else { "N/A" }
            );

        });
    });
}

fn physics_sim_ui(
    ui: &mut Ui,
    physics_sim_ui_data: &mut PhysicsSimUi,
) {
    ui.collapsing("Physics", |ui| {
        let pause_toggle = ui.button(
        if physics_sim_ui_data.physics_enabled { "Pause" }
            else { "Play" }
        );
        let step = ui.button("Step Once");
        let reset = ui.button("Reset");

        if pause_toggle.clicked() {
            physics_sim_ui_data.physics_enabled = !physics_sim_ui_data.physics_enabled;
        }
        physics_sim_ui_data.sim_step_pressed = step.clicked();

        if reset.clicked() {
            physics_sim_ui_data.resetted_snapshot_state = ResettedSnapshotState::LoadSnapshot;
            physics_sim_ui_data.sim_resetted = true;
            physics_sim_ui_data.physics_enabled = false;
            physics_sim_ui_data.sim_step_pressed = false;
        }
        else if physics_sim_ui_data.physics_enabled || physics_sim_ui_data.sim_step_pressed { //saving snapshot / handling resetted state
            if physics_sim_ui_data.sim_resetted {
                physics_sim_ui_data.resetted_snapshot_state = ResettedSnapshotState::SaveSnapshot;
            }
            physics_sim_ui_data.sim_resetted = false;
        }

        ui.label(if physics_sim_ui_data.sim_resetted { "Resetted" }
        else { "Unresetted" });
    });
}

fn control_physics_sim(
    mut rapier_config_q: Query<
        (&mut RapierConfiguration, &mut RapierContext),
        With<DefaultRapierContext>
    >,
    mut physics_sim_ui_data: ResMut<PhysicsSimUi>
) {
    let (mut config, mut rapier_context) = rapier_config_q.single_mut();

    match physics_sim_ui_data.resetted_snapshot_state {
        ResettedSnapshotState::LoadSnapshot => {
            physics_sim_ui_data.physics_enabled = false;
            if let Some(serialized_ctx) = &physics_sim_ui_data.resetted_snapshot {
                let resetted_ctx = bincode::deserialize::<RapierContext>(serialized_ctx).unwrap();
                let _ = std::mem::replace(rapier_context.deref_mut(), resetted_ctx);
            }
        },
        ResettedSnapshotState::SaveSnapshot => {
            physics_sim_ui_data.resetted_snapshot = Some(bincode::serialize(&*rapier_context).unwrap());
        },
        ResettedSnapshotState::Idle => {}
    }
    physics_sim_ui_data.resetted_snapshot_state = ResettedSnapshotState::Idle;

    if physics_sim_ui_data.physics_enabled {
        //pause sim if step btn was pressed while sim was enabled
        if physics_sim_ui_data.sim_step_pressed {
            physics_sim_ui_data.physics_enabled = false;
            config.physics_pipeline_active = false;
        }
        else { config.physics_pipeline_active = true; } //if step not pressed, continue sim as usual.
    }
    else {
        //run sim for this frame only if the step btn was pressed while sim was disabled
        if physics_sim_ui_data.sim_step_pressed { config.physics_pipeline_active = true; }
        else { config.physics_pipeline_active = false; } //stop sim if step wasn't pressed.
    }
}

pub fn edit_timestep_mode(
    mut rapier_timestep_mode: ResMut<TimestepMode>
) {
    //TODO: let user edit dt
}
