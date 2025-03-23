use std::fmt::Display;
use crate::prelude::*;
use crate::ui::import::{import_ui, import_window, RobotImporting};
use crate::ui::motion_planning::{
    ik_window, ik_window_function, motion_planning_ui, MotionPlanning,
};
use crate::ui::position_tools::{position_tools_ui, PositionTools};
use crate::ui::simulation::{simulation_control_window, simulation_ribbon_ui, PhysicsSimulation};
use crate::ui::{PointerUsageState, RobotLabUiAssets, SceneWindowData, SelectedEntities, UiGizmoGroup, UiResources};
use bevy::input::ButtonInput;
use bevy::prelude::{Commands, EventWriter, Gizmos, GlobalTransform, MouseButton, NonSend, NonSendMut, Or, Query, Res, ResMut, Resource, With};
use bevy_egui::egui;
use bevy_egui::egui::{Align, Color32, Layout, Rgba, Ui, UiBuilder};
use bevy_egui::EguiContexts;
use bevy_rapier3d::dynamics::{
    ImpulseJoint, MultibodyJoint, RapierImpulseJointHandle, RapierMultibodyJointHandle,
};
use bevy_rapier3d::plugin::RapierContext;
use bevy_rapier3d::prelude::ReadDefaultRapierContext;
use std::ops::Mul;
use std::result;

#[derive(Default, Resource)]
pub struct Ribbon {
    tab: RibbonTab,
}

#[derive(Default, PartialEq)]
pub enum RibbonTab {
    #[default]
    General,
    MotionPlanning,
    Fluids,
    //maybe Electromagnetics?
}

impl Display for RibbonTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let str = match self {
            RibbonTab::General => { "General Ribbon Tab" },
            RibbonTab::MotionPlanning => { "Motion Planning Ribbon Tab" },
            RibbonTab::Fluids => { "Fluids Ribbon Tab" },
        }.to_string();
        write!(f, "{}", str)
    }
}

macro_rules! finish_ui_section_vertical {
    ($ui:expr, $label_name:expr) => {{
        egui::Label::new("").layout_in_ui($ui);
        ($ui.min_rect(), $label_name)
    }};
}

pub(crate) use finish_ui_section_vertical;

pub fn ribbon_ui(
    mut ctxs: EguiContexts,
    mut ribbon: ResMut<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut ui_resources: UiResources,
    mut create_plan_event: EventWriter<CreatePlanEvent>,
    ui_assets: Res<RobotLabUiAssets>,
    scene_window_data: Res<SceneWindowData>,
    transform_q: Query<&GlobalTransform>,
    mut gizmos: Gizmos<UiGizmoGroup>,

    mut errors: EventWriter<ErrorEvent>
) {
    // Ribbon
    let result = egui::TopBottomPanel::top("Ribbon").show(ctxs.ctx_mut(), |ui| -> Result<()> {
        ui.set_max_height(150.0);
        macro_rules! ribbon_btn {
            ($txt:expr, $tab_enum:expr) => {
                egui::Button::new($txt).fill({
                    let c = Rgba::from(Color32::DARK_GRAY).multiply(if ribbon.tab == $tab_enum {
                        1.
                    } else {
                        0.25
                    });
                    Color32::from(Rgba::from_rgb(c[0], c[1], c[2]))
                })
            };
        }
        let tabs_rect = ui
            .horizontal(|ui| {
                let general = ui.add(ribbon_btn!("General", RibbonTab::General)).clicked();
                let motion_planning = ui
                    .add(ribbon_btn!("Motion Planning", RibbonTab::MotionPlanning))
                    .clicked();
                let fluids = ui.add(ribbon_btn!("Fluids", RibbonTab::Fluids)).clicked();
                if general {
                    ribbon.tab = RibbonTab::General
                } else if motion_planning {
                    ribbon.tab = RibbonTab::MotionPlanning;
                } else if fluids {
                    ribbon.tab = RibbonTab::Fluids;
                    bevy::log::warn!("Fluids tab isn't implemented yet!");
                }
                ui.min_rect()
            })
            .inner;
        let ribbon_height = ui.max_rect().height() - tabs_rect.height();
        match ribbon.tab {
            RibbonTab::General => {
                general_tab(
                    ui,
                    &mut ui_resources,
                    &mut selected_entities,
                    &scene_window_data,
                    &transform_q,
                    &mut gizmos,
                    &ui_assets,
                    ribbon_height,
                );
                Ok(())
            }
            RibbonTab::MotionPlanning => {
                motion_planning_ui(
                    ui,
                    &mut ui_resources,
                    &mut create_plan_event,
                    &selected_entities,
                )
            }
            _ => { Ok(()) }
        }
    }).inner;
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }

    // UI windows
    let result = match ribbon.tab {
        RibbonTab::General => Ok(general_windows(ctxs.ctx_mut(), &mut ui_resources)),
        RibbonTab::MotionPlanning => Ok(motion_planning_windows(ctxs.ctx_mut(), &mut ui_resources)),
        _ => {Ok(())}
    };
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }

    if ctxs.ctx_mut().is_using_pointer() {
        selected_entities.pointer_usage_state = PointerUsageState::UiUsingPointer;
    } else {
        selected_entities.pointer_usage_state = PointerUsageState::NotUsed;
    }
}

/// Finish ribbon tab by adding the names of each section
macro_rules! finish_ribbon_tab {
    ($ui:expr, $rects:expr) => {
        $ui.with_layout(Layout::bottom_up(Align::Center), |ui| {
            for (mut rect, section_name) in $rects {
                rect.max.y = ui.max_rect().max.y;
                ui.allocate_new_ui(UiBuilder::new().max_rect(rect), |ui| ui.label(section_name));
            }
        });
    };
}

use crate::kinematics::ik::KinematicNode;
use crate::robot::{RapierRobotHandles, Robot, RobotPart};
pub(crate) use finish_ribbon_tab;
use crate::error::ErrorEvent;
use crate::motion_planning::CreatePlanEvent;

fn general_tab(
    ui: &mut Ui,
    ui_resources: &mut UiResources,
    selected_entities: &mut SelectedEntities,
    scene_window_data: &SceneWindowData,
    transform_q: &Query<&GlobalTransform>,
    gizmos: &mut Gizmos<UiGizmoGroup>,
    ui_assets: &RobotLabUiAssets,
    ribbon_height: f32,
) {
    let mut rects = Vec::new();
    ui.horizontal(|ui| {
        rects.push(import_ui(ui, ui_resources));
        ui.add(egui::Separator::default().grow(ribbon_height));
        rects.push(position_tools_ui(
            ui,
            ui_resources,
            selected_entities,
            scene_window_data,
            transform_q,
            gizmos,
        ));
        ui.add(egui::Separator::default().grow(ribbon_height));
        rects.push(simulation_ribbon_ui(ui, ui_resources, &ui_assets));
        ui.add(egui::Separator::default().grow(ribbon_height));
    });

    finish_ribbon_tab!(ui, rects);
}

fn general_windows(
    egui_ctx: &mut egui::Context,
    resources: &mut UiResources
) {
    let UiResources {
        simulation,
        import,
        ..
    } = resources;
    simulation_control_window(egui_ctx, simulation);
    import_window(egui_ctx, import);
}

fn motion_planning_windows(
    egui_ctx: &mut egui::Context,
    resources: &mut UiResources,
) {
    ik_window(egui_ctx, resources);
}

pub fn ribbon_functionality(
    mut commands: Commands,
    ribbon: Res<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut transform_q: Query<&mut GlobalTransform>,
    mut ui_resources: UiResources,
    robot_q: Query<(&Robot, &RapierRobotHandles)>,
    joint_q: Query<
        (
            Option<&RapierImpulseJointHandle>, Option<&RapierMultibodyJointHandle>, Option<&KinematicNode>,
        ),
        Or<(With<RapierImpulseJointHandle>, With<RapierMultibodyJointHandle>,)>,
    >,
    scene_window_data: Res<SceneWindowData>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,

    mut errors: EventWriter<ErrorEvent>,
) {
    let result = match ribbon.tab {
        RibbonTab::General => || -> Result<()> {
            ui_resources.position_tools.functionality(
                &mut ui_resources.simulation,
                &scene_window_data,
                &mut selected_entities,
                &mut transform_q,
                mouse_button_input.just_released(MouseButton::Left),
                mouse_button_input.pressed(MouseButton::Left),
            )?;
            ui_resources.import.functionality(&mut commands)?;
            Ok(())
        }(),
        RibbonTab::MotionPlanning => || -> Result<()> {
            ik_window_function(
                &mut ui_resources,
                &mut commands,
                &mut selected_entities,
                &robot_q,
                &joint_q,
            )?;
            Ok(())
        }(),
        _ => { Ok(()) }
    };
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }
}
