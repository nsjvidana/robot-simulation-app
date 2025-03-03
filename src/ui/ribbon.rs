use std::ops::Mul;
use bevy::prelude::{Commands, Gizmos, GlobalTransform, MouseButton, Query, Res, ResMut, Resource};
use bevy_egui::EguiContexts;
use bevy_egui::egui::{Align, Color32, Layout, Rgba, Ui, UiBuilder};
use crate::ui::import::{import_ui, RobotImporting};
use crate::ui::position_tools::{position_tools_functionality, position_tools_ui, PositionTools};
use crate::ui::{PointerUsageState, RobotLabUiAssets, SceneWindowData, SelectedEntities, UiGizmoGroup};
use crate::ui::simulation::{simulation_control_window, simulation_ribbon_ui, PhysicsSimulation};
use bevy_egui::egui as egui;
use bevy::input::ButtonInput;

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

#[macro_export]
macro_rules! finish_ui_section_vertical {
    ($ui:expr, $label_name:expr) => {{
        egui::Label::new("").layout_in_ui($ui);
        ($ui.min_rect(), $label_name)
    }}
}

pub fn ribbon_ui(
    mut commands: Commands,
    mut ctxs: EguiContexts,
    mut ribbon: ResMut<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut position_tools: ResMut<PositionTools>,
    mut robot_importing: ResMut<RobotImporting>,
    mut physics_sim: ResMut<PhysicsSimulation>,
    ui_assets: Res<RobotLabUiAssets>,
    scene_window_data: Res<SceneWindowData>,
    transform_q: Query<&GlobalTransform>,
    mut gizmos: Gizmos<UiGizmoGroup>,
) {
    // Ribbon
    egui::TopBottomPanel::top("Ribbon").show(ctxs.ctx_mut(), |ui| {
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
        ui.horizontal(|ui| {
            let general = ui.add(ribbon_btn!("General", RibbonTab::General)).clicked();
            let motion_planning = ui.add(ribbon_btn!("Motion Planning", RibbonTab::MotionPlanning)).clicked();
            let fluids = ui.add(ribbon_btn!("Fluids", RibbonTab::Fluids)).clicked();
            if general { ribbon.tab = RibbonTab::General }
            else if motion_planning { ribbon.tab = RibbonTab::MotionPlanning; bevy::log::warn!("Motion planning tab isn't implemented yet!"); }
            else if fluids { ribbon.tab = RibbonTab::Fluids; bevy::log::warn!("Fluids tab isn't implemented yet!"); }
        });
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
                );
            },
            _ => {}
        }
    });

    //Physics sim window
    simulation_control_window(ctxs.ctx_mut(), &mut physics_sim);

    if ctxs.ctx_mut().is_using_pointer() {
        selected_entities.pointer_usage_state = PointerUsageState::UiUsingPointer;
    }
    else {
        selected_entities.pointer_usage_state = PointerUsageState::NotUsed;
    }
}

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
        ui.add(egui::Separator::default().grow(50.));
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
        ui.add(egui::Separator::default().grow(50.));
        rects.push(
            simulation_ribbon_ui(
                ui,
                physics_sim,
                &ui_assets
            )
        );
        ui.add(egui::Separator::default().grow(50.));
    });

    // Finish the ribbon by adding the names of each section
    ui.with_layout(Layout::bottom_up(Align::Center), |ui| {
        for (mut rect, section_name) in rects {
            rect.max.y = ui.max_rect().max.y;
            ui.allocate_new_ui(
                UiBuilder::new().max_rect(rect),
                |ui| ui.label(section_name)
            );
        }
    });
}

pub fn ribbon_functionality(
    ribbon: Res<Ribbon>,
    mut selected_entities: ResMut<SelectedEntities>,
    mut position_tools: ResMut<PositionTools>,
    mut transform_q: Query<&mut GlobalTransform>,
    scene_window_data: Res<SceneWindowData>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    physics_sim: Res<PhysicsSimulation>
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
        _ => {}
    }
}