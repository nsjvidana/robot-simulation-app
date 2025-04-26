use crate::error::error_handling_system;
use crate::functionality::motion_planning::{AllInstructions, Plan};
use crate::functionality::robot::{KinematicNode, RobotEntity, RobotMaterials, RobotQueryData, RobotSet};
use crate::functionality::simulation::SimulationState;
use crate::prelude::*;
use crate::ui::ribbon::motion_planning::ik::IkWindow;
use crate::ui::ribbon::Ribbon;
use crate::ui::selecting::SceneWindowData;
use bevy::app::App;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_rapier3d::prelude::{ColliderDebug, DefaultRapierContext, ImpulseJoint, MultibodyJoint, RapierConfiguration, RapierDebugRenderPlugin, RapierPickingPlugin, RapierPickingSettings, TimestepMode, WriteRapierContext};
use std::ops::{Deref, DerefMut};
use bevy::ecs::query::QueryData;

pub mod toolbar;
pub mod ribbon;
pub mod selecting;
pub mod generic_object;
pub mod properties;
pub mod robot_selection;

/// The distance between the camera and gizmos. Used for to make the visual effect
/// where gizmos stay at a reasonable size on the screen no matter where the camera is.
pub const GIZMO_DISTANCE: f32 = 5.;

pub struct RobotLabUiPlugin;

impl Plugin for RobotLabUiPlugin {
    fn build(&self, app: &mut App) {
        if !app.is_plugin_added::<EguiPlugin>() {
            app.add_plugins(EguiPlugin);
        }
        if !app.is_plugin_added::<RapierDebugRenderPlugin>() {
            app.add_plugins(RapierDebugRenderPlugin {
                default_collider_debug: ColliderDebug::NeverRender,
                ..Default::default()
            });
        }
        if !app.is_plugin_added::<RapierPickingPlugin>() {
            app.add_plugins(RapierPickingPlugin);
        }

        app.insert_resource(RapierPickingSettings {
            require_markers: true,
            ..Default::default()
        });

        app.init_gizmo_group::<UiGizmoGroup>();
        // Make all UI gizmos draw on top of everything
        app.init_gizmo_group::<UiGizmoGroup>();
        let mut gizmo_config_store = app
            .world_mut()
            .get_resource_mut::<GizmoConfigStore>()
            .unwrap();
        let (ui_gizmo_config, _) = gizmo_config_store.config_mut::<UiGizmoGroup>();
        ui_gizmo_config.depth_bias = -0.999;

        app.init_non_send_resource::<Ribbon>();

        app.init_utility_window_gizmos(IkWindow::default());

        let if_sim_is_paused = |s: Option<Res<State<SimulationState>>>| {
            s.is_some_and(|s| *s == SimulationState::Paused || *s == SimulationState::Reset)
        };

        app.configure_sets(
            Update,
            (
                (
                    RobotLabUiSet::Prepare
                        .run_if(if_sim_is_paused),
                    RobotLabUiSet::Ui,
                )
                    .chain()
                    .before(selecting::SelectingSet),
                RobotLabUiSet::Functionality
                    .after(selecting::SelectingSet)
                    .run_if(if_sim_is_paused),
            )
        );

        app.configure_sets(
            Update,
            (
                UiSet::RibbonUi,
                UiSet::UtilWindowEvents,
                UiSet::UtilityWindows,
            )
                .chain()
                .in_set(RobotLabUiSet::Ui)
        );

        app.add_systems(
            Update,
            (
                robot_lab_ui
                    .in_set(UiSet::RibbonUi),
                robot_lab_functionality
                    .pipe(error_handling_system)
                    .after(selecting::SelectingSet)
            )
        );


        selecting::build_app(app);
        ribbon::motion_planning::build_app(app);
        toolbar::build_app(app);
        properties::build_app(app);
    }
}

impl ManageUtilityWindowsExt for App {
    fn init_utility_window<Window: UtilityWindow + 'static>(&mut self, window: Window) -> &mut Self {
        self.insert_resource(window);
        self.add_event::<OpenUtilityWindow<Window>>();
        self.add_event::<UtilityOutputEvent<Window>>();
        self.add_systems(
            Update,
            (|
                mut open_events: ResMut<Events<OpenUtilityWindow<Window>>>,
                mut window: ResMut<Window>
            | {
                for e in open_events.drain() {
                    window.take_input(e.input);
                }
            }).in_set(UiSet::UtilWindowEvents)
        );
        self.add_systems(
            Update,
            (
                |mut egui_ctx: EguiContexts, mut window: ResMut<Window>, mut commands: Commands| {
                    window.window_ui(egui_ctx.ctx_mut(), &mut commands);
                },
                |mut window: ResMut<Window>, mut resources: FunctionalUiResources| -> Result<()> {
                    window.functionality(&mut resources)
                }
                    .pipe(error_handling_system)
            )
                .chain()
                .in_set(UiSet::UtilityWindows)
        );
        self
    }

    fn init_utility_window_gizmos<Window: UtilityWindow + GizmosUi + 'static>(&mut self, window: Window) -> &mut Self {
        self.init_utility_window::<Window>(window);
        self.add_systems(
            Update,
            (|mut window: ResMut<Window>, mut resources: GizmosUiResources| {
                window.gizmos_ui(&mut resources);
            })
                .in_set(UiSet::UtilityWindows)
        );

        self
    }
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum RobotLabUiSet {
    Prepare,
    Ui,
    Functionality,
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum UiSet {
    RibbonUi,
    UtilWindowEvents,
    UtilityWindows,
}

#[derive(SystemParam)]
pub struct FunctionalUiResources<'w, 's> {
    pub robots: Query<'w, 's, RobotQueryData>,
    pub robot_plans: Query<'w, 's, &'static mut Plan>,
    pub robot_set: Res<'w, RobotSet>,
    pub kinematic_nodes: Query<'w, 's, (Entity, &'static KinematicNode)>,
    pub all_instructions: NonSend<'w, AllInstructions>,
    pub sim_state: Res<'w, State<SimulationState>>,
    pub multibody_q: Query<'w, 's, &'static MultibodyJoint>,
    pub impulse_joint_q: Query<'w, 's, &'static ImpulseJoint>,
    pub timestep_mode: ResMut<'w, TimestepMode>,
    pub selections: Res<'w, RobotSelection>,
    pub scene_window_data: Res<'w, SceneWindowData>,
    pub robot_materials: Res<'w, RobotMaterials>,
    pub meshes: ResMut<'w, Assets<Mesh>>,
    pub parent: Query<'w, 's, &'static Parent>,
    pub transforms: Query<'w, 's, &'static mut GlobalTransform, Without<Camera3d>>,
    pub keyboard: Res<'w, ButtonInput<KeyCode>>,
    pub mouse: Res<'w, ButtonInput<MouseButton>>,
    pub commands: Commands<'w, 's>,
}

#[derive(SystemParam)]
pub struct GizmosUiResources<'w, 's> {
    functional_ui_resources: FunctionalUiResources<'w, 's>,
    pub camera: Single<'w, CameraQueryData>,
    pub gizmos: Gizmos<'w, 's, UiGizmoGroup>,
}

impl<'w, 's> Deref for GizmosUiResources<'w, 's> {
    type Target = FunctionalUiResources<'w, 's>;

    fn deref(&self) -> &Self::Target { &self.functional_ui_resources }
}

impl<'w, 's> DerefMut for GizmosUiResources<'w, 's> {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.functional_ui_resources }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct UiGizmoGroup;

#[derive(Event)]
pub struct OpenUtilityWindow<Window: UtilityWindow> {
    pub input: Window::Input,
}

#[derive(Event)]
pub struct UtilityOutputEvent<Window: UtilityWindow> {
    /// Is none if the utlity got canceled (e.g. window was closed)
    pub output: Option<Window::Output>,
}

#[derive(Resource)]
pub struct UtilityOutputResource<T>(T);

impl<T> Deref for UtilityOutputResource<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for UtilityOutputResource<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<Window: UtilityWindow> UtilityOutputEvent<Window> {
    pub fn is_canceled(&self) -> bool {
        self.output.is_none()
    }
}

#[derive(QueryData)]
pub struct CameraQueryData {
    entity: Entity,
    camera: &'static Camera3d,
    transform: &'static GlobalTransform,
}

pub trait View {
    fn prepare(&mut self, _functional_ui_resources: &mut FunctionalUiResources) {}

    fn ui(&mut self, ui: &mut egui::Ui, commands: &mut Commands);

    fn view_name(&self) -> &'static str {
        std::any::type_name::<Self>()
    }

    fn functionality(&mut self, _functional_resources: &mut FunctionalUiResources) -> Result<()> { Ok(()) }
}

pub trait WindowUi: View {
    fn window_ui(&mut self, egui_context: &mut egui::Context, commands: &mut Commands);

    fn set_open(&mut self, open: bool);
}

pub trait GizmosUi {
    fn gizmos_ui(&mut self, resources: &mut GizmosUiResources);
}

pub trait ManageUtilityWindowsExt {
    fn init_utility_window<Window: UtilityWindow + 'static>(&mut self, window: Window) -> &mut Self;

    fn init_utility_window_gizmos<Window: UtilityWindow + GizmosUi + 'static>(&mut self, window: Window) -> &mut Self;
}

pub type NoOutput = ();

pub trait UtilityWindow: WindowUi + Resource {
    type Input: Send + Sync;
    type Output: Send + Sync;

    fn take_input(&mut self, window_input: Self::Input);

    fn has_input(&mut self) -> bool;

    fn emit_output(&mut self) -> Option<Self::Output>;

    fn clear_input(&mut self);

    fn is_canceled(&self) -> bool;

    fn set_canceled(&mut self, canceled: bool);
}

#[macro_export]
macro_rules! impl_util_window {
    ($window:ident, $input_type:ty, $output_type:ty, $input_field:ident, $output_field:ident, $canceled_field:ident) => {
        impl crate::ui::UtilityWindow for $window {
            type Input = $input_type;
            type Output = $output_type;

            fn take_input(&mut self, window_input: Self::Input) {
                self.$input_field = Some(window_input);
                self.$canceled_field = false;
                self.set_open(true);
            }

            fn has_input(&mut self) -> bool {
                self.$input_field.is_some()
            }

            fn emit_output(&mut self) -> Option<Self::Output> {
                self.$output_field.take()
            }

            fn clear_input(&mut self) {
                self.$input_field = None;
            }

            fn is_canceled(&self) -> bool {
                self.$canceled_field
            }

            fn set_canceled(&mut self, canceled: bool) {self.$canceled_field = canceled;}
        }
    };
}

macro_rules! drag_value {
    ($ui:expr, $label:expr, $field:expr) => {
        crate::ui::drag_value_decimals!($ui, $label, $field, 3)
    };
}

macro_rules! drag_value_decimals {
    ($ui:expr, $label:expr, $field:expr, $decimals:expr) => {
        $ui.horizontal(|ui| {
            ui.label($label);
            ui.add(
                egui::DragValue::new(&mut $field)
                    .min_decimals($decimals)
            )
        }).inner
    };
}

pub(crate) use drag_value;
pub(crate) use drag_value_decimals;
use robot_selection::RobotSelection;

pub fn robot_lab_ui(
    mut egui_ctxs: EguiContexts,
    mut ribbon: NonSendMut<Ribbon>,

    mut commands: Commands
) {
    // Ribbon
    egui::TopBottomPanel::top("ribbon_panel")
        .show(egui_ctxs.ctx_mut(), |ui| {
            ribbon.ui(ui, &mut commands);
        });
}

pub fn robot_lab_functionality(
    mut functional_ui_resources: FunctionalUiResources,
    mut ribbon: NonSendMut<Ribbon>,
) -> Result<()> {
    ribbon.functionality(&mut functional_ui_resources)
}
