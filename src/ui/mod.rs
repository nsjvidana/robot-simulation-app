pub mod toolbar;
mod import;

use std::cmp::Ordering;
use std::ops::DerefMut;
use bevy::app::{App, Update};
use bevy::ecs::intern::Interned;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::math::Vec3;
use crate::kinematics::ik::{ForwardAscentCyclic, ForwardDescentCyclic};
use crate::math::{project_onto_plane, ray_scale_for_plane_intersect_local, Real};
use crate::robot::{Robot, RobotSet, RobotPart};
use bevy::prelude::{ButtonInput, Camera, Color, Commands, Component, DetectChanges, Entity, Gizmos, GlobalTransform, InfinitePlane3d, IntoSystemConfigs, Isometry3d, Local, Mat3, MouseButton, Name, Plugin, Quat, Query, Ray3d, Res, ResMut, Resource, Single, Time, Transform, Vec2, Window, With, Without};
use bevy_egui::egui::{ComboBox, Ui};
use bevy_egui::{egui, EguiContexts};
use bevy_rapier3d::parry::math::{Isometry, Vector};
use bevy_rapier3d::parry::query::RayCast;
use bevy_rapier3d::plugin::TimestepMode;
use bevy_rapier3d::prelude::{DefaultRapierContext, PhysicsSet, QueryFilter, RapierConfiguration, RapierContext, ReadDefaultRapierContext};
use bevy_rapier3d::rapier::prelude::{Cuboid, Ray};
use k::{InverseKinematicsSolver, SerialChain};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use crate::ui::import::{import_ui, RobotImporting};
use crate::ui::toolbar::{toolbar_ui, Toolbar};

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
        app.init_resource::<PhysicsSimUi>()
            .init_resource::<SceneWindowData>()
            .init_resource::<SelectedEntities>()
            .init_resource::<RobotImporting>()
            .init_resource::<Toolbar>();

        app.add_systems(
            self.schedule,
            (
                update_scene_window_data,
                update_clicked_entities,
                robot_lab_ui,
                // robot_sandbox_ui
            ).chain()
        );
        // .add_systems(self.physics_schedule, control_physics_sim
        //     .before(PhysicsSet::StepSimulation)
        //     .after(PhysicsSet::SyncBackend)
        // );
    }
}

#[derive(Resource, Default)]
pub struct SelectedEntities {
    /// The entities that were clicked this frame.
    /// This has all the entities under the pointer when the user clicked.
    pub clicked_entities: Option<Entity>,
    pub selected_entities: Vec<Entity>,
    pub selected_robots: Vec<Entity>,
    pub active_robot: Option<Entity>,
}

macro_rules! finish_ui_section_vertical {
    ($ui:expr, $label_name:expr) => {{
        $ui.label("");
        $ui.set_width($ui.min_size().x);
        $ui.vertical(|ui|
            ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                ui.label($label_name);
            })
        );
    }};
}

pub fn robot_lab_ui(
    mut commands: Commands,
    mut ctxs: EguiContexts,
    mut selected_entities: ResMut<SelectedEntities>,
    mut toolbar: ResMut<Toolbar>,
    mut transform_q: Query<&mut GlobalTransform>,
    mut robot_importing: ResMut<RobotImporting>,
    mut gizmos: Gizmos,
) {
    egui::TopBottomPanel::top("Toolbar").show(ctxs.ctx_mut(), |ui| {
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                import_ui(
                    &mut commands,
                    ui,
                    &mut robot_importing,
                );
                finish_ui_section_vertical!(ui, "Import")
            });
            ui.separator();
            ui.vertical(|ui| {
                toolbar_ui(
                    ui,
                    &mut toolbar,
                    &mut selected_entities,
                    &transform_q,
                    &mut gizmos,
                );
                finish_ui_section_vertical!(ui, "Move")
            });
            ui.separator();
        });
    });
}

pub fn update_scene_window_data(
    camera: Single<(&Camera, &GlobalTransform)>,
    mut scene_window_data: ResMut<SceneWindowData>,
    window: Single<&Window>
) {
    let (camera, camera_transform) = *camera;
    scene_window_data.viewport_to_world_ray = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok());
}

pub fn update_clicked_entities(
    rapier_context: ReadDefaultRapierContext,
    mut selected_entities: ResMut<SelectedEntities>,
    scene_window_data: Res<SceneWindowData>,
) {
    if let Some(ray) = scene_window_data.viewport_to_world_ray {
        let mut intersections = Vec::new();
        rapier_context.intersections_with_ray(
            ray.origin,
            ray.direction.as_vec3(),
            Real::MAX,
            true,
            QueryFilter::default(),
            |ent, intersection| {
                intersections.push((ent, intersection.time_of_impact));
                true
            }
        );

        intersections.sort_by(|(_, a), (_, b)| {
            if let Some(ord) = a.partial_cmp(b) { ord }
            else { Ordering::Equal }
        });
    }
}

pub struct IKSandboxUI {
    pub kinematic_chain: Option<SerialChain<Real>>,
    pub solvers: Vec<Box<dyn InverseKinematicsSolver<Real> + Send>>,
    pub solver_names: Vec<String>,
    pub selected_solver_idx: usize,
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
            selected_solver_idx: 0,
        }
    }
}

pub struct RobotImportUI {
    pub mb_loader_options: UrdfMultibodyOptions,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub selected_joint_type: usize,
}

impl Default for RobotImportUI {
    fn default() -> Self {
        Self {
            mb_loader_options: UrdfMultibodyOptions::default(),
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

#[derive(Default)]
pub struct ToolbarUI {
    pub toolbar_state: ToolbarState,
    pub selected_robot: Option<Entity>,
    /// The entity that was clicked this frame
    pub clicked_entity: Option<Entity>,
    pub simulation_was_clicked: bool,
    pub selected_entity: Option<Entity>,

    pub is_interacting_with_object: bool,
    pub selected_axis: Option<UnitVector3<Real>>,
    pub selected_axis_normal: Option<UnitVector3<Real>>,
    pub init_interaction_pos: Option<Vector3<Real>>,
    pub final_interaction_pos: Option<Vector3<Real>>,
}

impl ToolbarUI {
    pub fn remove_interaction_data(&mut self) {
        self.selected_axis = None;
        self.selected_axis_normal = None;
        self.init_interaction_pos = None;
        self.final_interaction_pos = None;
        self.is_interacting_with_object = false;
    }
}

pub enum ToolbarState {
    Grab,
    Rotate,
}

impl Default for ToolbarState {
    fn default() -> Self {
        Self::Grab
    }
}

#[derive(Resource)]
pub struct PhysicsSimUi {
    pub physics_enabled: bool,
    pub sim_step_pressed: bool,
    pub sim_resetted: bool,
    pub resetted_snapshot: Option<RobotSimSnapshot>,
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

pub struct RobotSimSnapshot {
    pub rapier_context: Vec<u8>,
    pub robots: Vec<u8>,
}

#[derive(Resource, Default)]
pub struct SceneWindowData {
    viewport_to_world_ray: Option<Ray3d>
}

pub fn robot_sandbox_ui(
    mut ctxs: EguiContexts,
    mut ik_ui: Local<IKSandboxUI>,
    mut physics_sim: ResMut<PhysicsSimUi>,
    mut importing_ui: Local<RobotImportUI>,
    mut toolbar_ui: Local<ToolbarUI>,
    mut commands: Commands,
    robot_part_q: Query<&RobotPart>,
    mut transform_q: Query<&mut GlobalTransform>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    scene_window_data: Res<SceneWindowData>,
    default_rapier_context_q: Query<&RapierContext, With<DefaultRapierContext>>,
    mut gizmos: Gizmos,
) {
    let rapier_context = default_rapier_context_q.single();

    egui::Window::new("Robot Sandbox").show(
        ctxs.ctx_mut(), |ui| {

            //Robot importing
            robot_import_ui(
                &mut commands,
                ui,
                &mut importing_ui,
            );

            //Inverse Kinematics
            robot_ik_ui(
                ui,
                &mut ik_ui,
            );

            physics_sim_ui(
                ui,
                &mut physics_sim,
            );
        }
    );


    if mouse_button_input.just_pressed(MouseButton::Left) {
        if !ctxs.ctx_mut().is_using_pointer() { // If the click wasn't on the UI
            toolbar_ui.simulation_was_clicked = true;
            // Get the entity that was clicked in the current frame (if any)
            if let Some(ray) = scene_window_data.viewport_to_world_ray {
                if let Some((rb_ent, _)) = rapier_context.cast_ray(
                    ray.origin,
                    ray.direction.as_vec3(),
                    Real::MAX,
                    true,
                    QueryFilter::default()
                ) { toolbar_ui.clicked_entity = Some(rb_ent); } else { toolbar_ui.clicked_entity = None; }
            }
        }
        else {
            toolbar_ui.simulation_was_clicked = false;
        }
    }
    else {
        toolbar_ui.simulation_was_clicked = false;
    }

    //Toolbar window
    egui::Window::new("Toolbar").show(
        ctxs.ctx_mut(), |ui| {
            make_toolbar_ui(
                ui,
                &mut toolbar_ui,
                &robot_part_q,
                &mut gizmos,
                &mut transform_q,
                &mouse_button_input,
                scene_window_data.viewport_to_world_ray.as_ref()
            );
        }
    );

}

fn robot_import_ui(
    commands: &mut Commands,
    ui: &mut Ui,
    importing_ui: &mut RobotImportUI,
) {
    ui.collapsing("Import Robot", |ui| {
        let _checkbox = ui.checkbox(
            &mut importing_ui.urdf_loader_options.make_roots_fixed,
            "Make roots fixed"
        );
        ComboBox::from_label("Robot joint type")
            .selected_text(match importing_ui.selected_joint_type {
                0 => "Impulse", 1 => "Multibody", _ => unreachable!()
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut importing_ui.selected_joint_type, 0, "Impulse");
                ui.selectable_value(&mut importing_ui.selected_joint_type, 1, "Multibody");
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
                    importing_ui.urdf_loader_options.clone(),
                    None
                ).unwrap();
                let robot_cmp = Robot::new(robot);
                commands.spawn((
                    match importing_ui.selected_joint_type {
                        0 => robot_cmp.with_impulse_joints(),
                        1 => robot_cmp.with_multibody_joints(importing_ui.mb_loader_options),
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
    ik_ui: &mut IKSandboxUI,
) {
    ui.collapsing("Inverse Kinematics", |ui| {
        let solver_idx = ik_ui.selected_solver_idx;
        let selected_solver_name =
            if ik_ui.solvers.get(solver_idx).is_some() {
                ik_ui.solver_names.get(solver_idx).unwrap()
            }
            else { &"-".to_string() };
        ComboBox::from_label("Solver type")
            .selected_text(selected_solver_name)
            .show_ui(ui, |ui| {
                for (i, solver_name) in ik_ui.solver_names.iter().enumerate() {
                    ui.selectable_value(&mut ik_ui.selected_solver_idx, i, solver_name);
                }
            });

        ui.horizontal(|ui| {

            ui.label("Chain:");
            let _ = ui.button(
                if ik_ui.kinematic_chain.is_some() {"Chain" }
                else { "N/A" }
            );

        });
    });
}

fn physics_sim_ui(
    ui: &mut Ui,
    physics_sim: &mut PhysicsSimUi,
) {
    ui.collapsing("Physics", |ui| {
        let pause_toggle = ui.button(
        if physics_sim.physics_enabled { "Pause" }
            else { "Play" }
        );
        let step = ui.button("Step Once");
        let reset = ui.button("Reset");

        if pause_toggle.clicked() {
            physics_sim.physics_enabled = !physics_sim.physics_enabled;
        }
        physics_sim.sim_step_pressed = step.clicked();

        if reset.clicked() {
            physics_sim.resetted_snapshot_state = ResettedSnapshotState::LoadSnapshot;
            physics_sim.sim_resetted = true;
            physics_sim.physics_enabled = false;
            physics_sim.sim_step_pressed = false;
        }
        else if physics_sim.physics_enabled || physics_sim.sim_step_pressed { //saving snapshot / handling resetted state
            if physics_sim.sim_resetted {
                physics_sim.resetted_snapshot_state = ResettedSnapshotState::SaveSnapshot;
            }
            physics_sim.sim_resetted = false;
        }

        ui.label(if physics_sim.sim_resetted { "Resetted" }
        else { "Unresetted" });
    });
}

fn control_physics_sim(
    mut commands: Commands,
    mut rapier_config_q: Query<
        (&mut RapierConfiguration, &mut RapierContext),
        With<DefaultRapierContext>
    >,
    mut physics_sim_ui_data: ResMut<PhysicsSimUi>,
    mut robot_set: ResMut<RobotSet>
) {
    let (mut config, mut rapier_context) = rapier_config_q.single_mut();

    match physics_sim_ui_data.resetted_snapshot_state {
        ResettedSnapshotState::LoadSnapshot => {
            physics_sim_ui_data.physics_enabled = false;
            if let Some(snapshot) = &physics_sim_ui_data.resetted_snapshot {
                let resetted_ctx = bincode::deserialize::<RapierContext>(&snapshot.rapier_context).unwrap();
                let _ = std::mem::replace(rapier_context.deref_mut(), resetted_ctx);
                let resetted_robot_set = bincode::deserialize::<RobotSet>(&snapshot.rapier_context).unwrap();
                for robot_data in resetted_robot_set.robots.iter().map(|v| v.1) {
                    commands.entity(robot_data.robot_entity).insert(robot_data.transform);
                }
            }
        },
        ResettedSnapshotState::SaveSnapshot => {
            physics_sim_ui_data.resetted_snapshot = Some(RobotSimSnapshot {
                rapier_context: bincode::serialize(&*rapier_context).unwrap(),
                robots: bincode::serialize(&*robot_set).unwrap(),
            });
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

fn make_toolbar_ui(
    ui: &mut Ui,
    toolbar_ui: &mut ToolbarUI,
    robot_part_q: &Query<&RobotPart>,
    gizmos: &mut Gizmos,
    mut transform_q: &mut Query<&mut GlobalTransform>,
    mouse_button_input: &ButtonInput<MouseButton>,
    ray: Option<&Ray3d>,
) {
    ui.horizontal(|ui| {
        let mut grab = matches!(toolbar_ui.toolbar_state, ToolbarState::Grab);
        let grap_resp = ui.toggle_value(&mut grab, "Grab");
        let mut rotate = matches!(toolbar_ui.toolbar_state, ToolbarState::Rotate);
        let rotate_resp = ui.toggle_value(&mut rotate, "Rotate");

        if grab && grap_resp.clicked() { toolbar_ui.toolbar_state = ToolbarState::Grab; }
        else if rotate && rotate_resp.clicked() { toolbar_ui.toolbar_state = ToolbarState::Rotate; }
    });

    if let Some(robot_entity) = toolbar_ui.selected_robot {
        //drawing toolbar gizmos (grab & rotate)
        if let Ok(mut robot_transform) = transform_q.get_mut(robot_entity) {
            let robot_pos = robot_transform.translation();
            match toolbar_ui.toolbar_state {
                ToolbarState::Grab => { // for now, it only moves in global coords
                    // draw axes
                    gizmos.arrow(robot_pos, robot_pos + Vec3::X, Color::linear_rgb(1., 0., 0.));
                    gizmos.arrow(robot_pos, robot_pos + Vec3::Y, Color::linear_rgb(0., 1., 0.));
                    gizmos.arrow(robot_pos, robot_pos + Vec3::Z, Color::linear_rgb(0., 0., 1.));

                    let just_clicked = mouse_button_input.just_pressed(MouseButton::Left);
                    let just_released = mouse_button_input.just_released(MouseButton::Left);
                    // Finding selected axis.
                    let mut clicked_axes = [false, false, false];
                    if just_clicked && ray.is_some() {
                        let ray = ray.unwrap();
                        let robot_translation: Vector3<Real> = robot_pos.into();
                        let ray = Ray {
                            origin: ray.origin.into(),
                            dir: ray.direction.as_vec3().into(),
                        };
                        let cuboid = Cuboid::new(Vector3::new(0.5, 0.05, 0.05));
                        let mut iso = Isometry3 {
                            translation: (robot_translation + Vector3::new(0.5f32, 0., 0.)).into(),
                            rotation: UnitQuaternion::identity(),
                        };
                        clicked_axes[0] = cuboid.intersects_ray(&iso, &ray, 1000.0);
                        iso.translation = (robot_translation + Vector3::new(0., 0.5, 0.)).into();
                        iso.rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
                        clicked_axes[1] = cuboid.intersects_ray(&iso, &ray, 1000.0);
                        iso.translation = (robot_translation + Vector3::new(0., 0., 0.5)).into();
                        iso.rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);
                        clicked_axes[2] = cuboid.intersects_ray(&iso, &ray, 1000.0);

                        toolbar_ui.is_interacting_with_object = clicked_axes.contains(&true);
                        if toolbar_ui.is_interacting_with_object {
                            toolbar_ui.selected_axis =
                                if clicked_axes[0] { Some(Vector3::x_axis()) }
                                else if clicked_axes[1] { Some(Vector3::y_axis()) }
                                else if clicked_axes[2] { Some(Vector3::z_axis()) }
                                else { unreachable!() };
                            toolbar_ui.selected_axis_normal =
                                if clicked_axes[0] { Some(Vector3::z_axis()) }
                                else if clicked_axes[1] { Some(Vector3::x_axis()) }
                                else if clicked_axes[2] { Some(Vector3::x_axis()) }
                                else { unreachable!() };
                        }
                        else {
                            toolbar_ui.selected_axis = None;
                            toolbar_ui.init_interaction_pos = None;
                        }
                    }

                    // Calculate data needed for when the interacting is done.
                    if toolbar_ui.is_interacting_with_object && ray.is_some() {
                        let ray = ray.unwrap();
                        let plane_normal = toolbar_ui.selected_axis_normal.unwrap();

                        let ray_len = ray_scale_for_plane_intersect_local(
                            &plane_normal,
                            &(ray.origin - robot_pos).into(),
                            &ray.direction.as_vec3().into(),
                        );
                        if let Some(len) = ray_len {
                            if len >= 0. {// ensure that the intersection isn't behind the camera
                                let intersection_pos = (ray.origin + ray.direction * len).into();
                                if just_clicked {
                                    toolbar_ui.init_interaction_pos = Some(intersection_pos);
                                }
                                else if just_released {
                                    toolbar_ui.final_interaction_pos = Some(intersection_pos);
                                }
                                else {
                                    toolbar_ui.final_interaction_pos = None;
                                }
                                gizmos.sphere(
                                    Isometry3d::from_translation(intersection_pos),
                                    0.1,
                                    Color::linear_rgb(0., 1., 1.),
                                );
                            }
                        }
                    }

                    // debug draw initial click pos
                    if let Some(pos) = toolbar_ui.init_interaction_pos {
                        gizmos.sphere(Isometry3d::from_translation(pos), 0.1, Color::linear_rgb(1., 1., 1.));
                    }

                    if just_released && toolbar_ui.is_interacting_with_object && ray.is_some() {
                        if let Some(final_pos) = toolbar_ui.final_interaction_pos {
                            let init_pos = toolbar_ui.init_interaction_pos.unwrap();
                            let moved_dist = (final_pos - init_pos).dot(&toolbar_ui.selected_axis.unwrap());
                            *robot_transform = robot_transform.mul_transform(
                                Transform::from_translation(toolbar_ui.selected_axis.unwrap().scale(moved_dist).into())
                            );
                        }

                        // register that the interaction is now done
                        toolbar_ui.remove_interaction_data();
                    }
                },
                //draw rotate gizmos
                ToolbarState::Rotate => { //for now, it only rotates locally
                    let mut iso = robot_transform.to_isometry();
                    let rot_x = iso.rotation * Vec3::X;
                    let rot_y = iso.rotation * Vec3::Y;
                    let rot_z = iso.rotation * Vec3::Z;

                    iso.rotation = Quat::from_mat3(&Mat3 { x_axis: -rot_z, y_axis: rot_y, z_axis: rot_x, });
                    gizmos.circle(iso, 1., Color::linear_rgb(1., 0., 0.));
                    iso.rotation = Quat::from_mat3(&Mat3 { x_axis: -rot_x, y_axis: rot_z, z_axis: rot_y, });
                    gizmos.circle(iso, 1., Color::linear_rgb(0., 1., 0.));
                    iso.rotation = Quat::from_mat3(&Mat3 { x_axis: rot_x, y_axis: rot_y, z_axis: rot_z, });
                    gizmos.circle(iso, 1., Color::linear_rgb(0., 0., 1.));
                }
            }


        }
    }

    //get selected robot if one was selected
    if let Some(clicked_entity) = toolbar_ui.clicked_entity {
        if !toolbar_ui.is_interacting_with_object {
            toolbar_ui.selected_entity = Some(clicked_entity);
            if let Ok(robot_entity) = robot_part_q.get(clicked_entity).map(|v| v.0) {
                toolbar_ui.selected_robot = Some(robot_entity);
            }
        }
    }
    else if toolbar_ui.simulation_was_clicked && !toolbar_ui.is_interacting_with_object {
        toolbar_ui.selected_entity = None;
        toolbar_ui.selected_robot = None;
    }
}

pub fn edit_timestep_mode(
    mut rapier_timestep_mode: ResMut<TimestepMode>,
) {
    //TODO: let user edit dt
}
