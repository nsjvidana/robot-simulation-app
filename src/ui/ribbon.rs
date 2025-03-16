use std::ops::Mul;
use bevy::prelude::{Commands, Gizmos, GlobalTransform, MouseButton, NonSend, NonSendMut, Or, Query, Res, ResMut, Resource, With};
use bevy_egui::EguiContexts;
use bevy_egui::egui::{Align, Color32, Layout, Rgba, Ui, UiBuilder};
use crate::ui::import::{import_ui, RobotImporting};
use crate::ui::position_tools::{position_tools_functionality, position_tools_ui, PositionTools};
use crate::ui::{PointerUsageState, RobotLabUiAssets, SceneWindowData, SelectedEntities, UiGizmoGroup};
use crate::ui::simulation::{simulation_control_window, simulation_ribbon_ui, PhysicsSimulation};
use bevy_egui::egui as egui;
use bevy::input::ButtonInput;
use bevy_rapier3d::dynamics::{ImpulseJoint, MultibodyJoint, RapierImpulseJointHandle, RapierMultibodyJointHandle};
use bevy_rapier3d::plugin::RapierContext;
use bevy_rapier3d::prelude::ReadDefaultRapierContext;
use crate::ui::motion_planning::{ik_window, motion_planning_ui, ik_window_function, MotionPlanning};

#[derive(Default, Resource)]
pub struct Ribbon {
    tab: RibbonTab
}

#[derive(Default, PartialEq)]
pub enum RibbonTab {
    #[default]
    General,
    MotionPlanning,
    Fluids,
    //maybe Electromagnetics?
}

macro_rules! finish_ui_section_vertical {
    ($ui:expr, $label_name:expr) => {{
        egui::Label::new("").layout_in_ui($ui);
        ($ui.min_rect(), $label_name)
    }}
}

pub(crate) use finish_ui_section_vertical;

pub fn ribbon_ui(
    mut commands: Commands,
    mut ctxs: EguiContexts,
    mut ribbon: ResMut<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut position_tools: ResMut<PositionTools>,
    mut robot_importing: ResMut<RobotImporting>,
    mut physics_sim: ResMut<PhysicsSimulation>,
    mut motion_planning: NonSendMut<MotionPlanning>,
    ui_assets: Res<RobotLabUiAssets>,
    scene_window_data: Res<SceneWindowData>,
    transform_q: Query<&GlobalTransform>,
    robot_part_q: Query<&RobotPart>,
    mut gizmos: Gizmos<UiGizmoGroup>,
) {
    // Ribbon
    egui::TopBottomPanel::top("Ribbon").show(ctxs.ctx_mut(), |ui| {
        ui.set_max_height(150.0);
        macro_rules! ribbon_btn {
            ($txt:expr, $tab_enum:expr) => {
                egui::Button::new($txt).fill({
                    let c = Rgba::from(Color32::DARK_GRAY).multiply(
                        if ribbon.tab == $tab_enum { 1. }
                        else { 0.25 }
                    );
                    Color32::from(Rgba::from_rgb(c[0], c[1], c[2]))
                })
            };
        }
        let tabs_rect = ui.horizontal(|ui| {
            let general = ui.add(ribbon_btn!("General", RibbonTab::General)).clicked();
            let motion_planning = ui.add(ribbon_btn!("Motion Planning", RibbonTab::MotionPlanning)).clicked();
            let fluids = ui.add(ribbon_btn!("Fluids", RibbonTab::Fluids)).clicked();
            if general { ribbon.tab = RibbonTab::General }
            else if motion_planning { ribbon.tab = RibbonTab::MotionPlanning; }
            else if fluids { ribbon.tab = RibbonTab::Fluids; bevy::log::warn!("Fluids tab isn't implemented yet!"); }
            ui.min_rect()
        }).inner;
        let ribbon_height = ui.max_rect().height() - tabs_rect.height();
        match ribbon.tab {
            RibbonTab::General => {
                general_tab(
                    ui,
                    &mut commands,
                    &mut robot_importing,
                    &mut position_tools,
                    &mut selected_entities,
                    &scene_window_data,
                    &transform_q,
                    &mut gizmos,
                    &mut physics_sim,
                    &ui_assets,
                    ribbon_height
                );
            },
            RibbonTab::MotionPlanning => {
                motion_planning_ui(
                    ui,
                    &mut selected_entities,
                    &mut motion_planning,
                    ribbon_height
                );
            }
            _ => {}
        }
    });

    //Physics sim window
    match ribbon.tab {
        RibbonTab::General => {
            simulation_control_window(ctxs.ctx_mut(), &mut physics_sim);
        },
        RibbonTab::MotionPlanning => {
            ik_window(ctxs.ctx_mut(), &mut motion_planning);
        }
        _ => {}
    }

    if ctxs.ctx_mut().is_using_pointer() {
        selected_entities.pointer_usage_state = PointerUsageState::UiUsingPointer;
    }
    else {
        selected_entities.pointer_usage_state = PointerUsageState::NotUsed;
    }
}

/// Finish ribbon tab by adding the names of each section
macro_rules! finish_ribbon_tab {
    ($ui:expr, $rects:expr) => {
        $ui.with_layout(Layout::bottom_up(Align::Center), |ui| {
            for (mut rect, section_name) in $rects {
                rect.max.y = ui.max_rect().max.y;
                ui.allocate_new_ui(
                    UiBuilder::new().max_rect(rect),
                    |ui| ui.label(section_name)
                );
            }
        });
    };
}

pub(crate) use finish_ribbon_tab;
use crate::robot::{RapierRobotHandles, Robot, RobotPart};

fn general_tab(
    ui: &mut Ui,
    commands: &mut Commands,
    robot_importing: &mut RobotImporting,
    position_tools: &mut PositionTools,
    selected_entities: &mut SelectedEntities,
    scene_window_data: &SceneWindowData,
    transform_q: &Query<&GlobalTransform>,
    gizmos: &mut Gizmos<UiGizmoGroup>,
    physics_sim: &mut PhysicsSimulation,
    ui_assets: &RobotLabUiAssets,
    ribbon_height: f32,
) {
    let mut rects = Vec::new();
    ui.horizontal(|ui| {
        rects.push(
            import_ui(
                commands,
                ui,
                robot_importing,
            )
        );
        ui.add(egui::Separator::default().grow(ribbon_height));
        rects.push(
            position_tools_ui(
                ui,
                position_tools,
                selected_entities,
                scene_window_data,
                transform_q,
                gizmos,
                physics_sim
            )
        );
        ui.add(egui::Separator::default().grow(ribbon_height));
        rects.push(
            simulation_ribbon_ui(
                ui,
                physics_sim,
                &ui_assets
            )
        );
        ui.add(egui::Separator::default().grow(ribbon_height));
    });

    finish_ribbon_tab!(ui, rects);
}

pub fn ribbon_functionality(
    mut commands: Commands,
    ribbon: Res<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut position_tools: ResMut<PositionTools>,
    mut transform_q: Query<&mut GlobalTransform>,

    mut motion_planning: NonSendMut<MotionPlanning>,

    robot_q: Query<(&Robot, &RapierRobotHandles)>,
    robot_part_q: Query<&RobotPart>,
    joint_q: Query<
        (Option<&RapierImpulseJointHandle>, Option<&RapierMultibodyJointHandle>),
        Or<(With<RapierImpulseJointHandle>, With<RapierMultibodyJointHandle>)>
    >,
    rapier_ctx: ReadDefaultRapierContext,
    scene_window_data: Res<SceneWindowData>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    physics_sim: Res<PhysicsSimulation>,

    mut gizmos: Gizmos<UiGizmoGroup>,
) {
    match ribbon.tab {
        RibbonTab::General => {
            position_tools_functionality(
                &mut position_tools,
                &scene_window_data,
                &mut selected_entities,
                &mut transform_q,
                mouse_button_input.just_released(MouseButton::Left),
                mouse_button_input.pressed(MouseButton::Left),
                &physics_sim
            );
        },
        RibbonTab::MotionPlanning => {
            ik_window_function(
                &mut commands,
                &mut selected_entities,
                &mut motion_planning,
                &robot_q,
                &joint_q,
            );
        },
        _ => {}
    }
}