use std::fmt::Display;
use crate::prelude::*;
use crate::ui::motion_planning::MotionPlanning;
use crate::ui::{GizmosUi, GizmosUiParameters, PointerUsageState, RobotLabUiAssets, SceneWindowData, SelectedEntities, UiEvents, UiGizmoGroup, UiResources, View, WindowUI};
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
    mut ui_resources: UiResources,
    mut gizmos_resources: GizmosUiParameters,
    ui_assets: Res<RobotLabUiAssets>,
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
                ui_resources.general_tab.ui(ui, &ui_assets);
                Ok(())
            }
            RibbonTab::MotionPlanning => {
                ui_resources.motion_planning_tab.ui(ui, &ui_assets);
                Ok(())
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

    // Gizmos UI
    match ribbon.tab {
        RibbonTab::General => {
            GeneralTab::gizmos_ui(&mut ui_resources, &mut gizmos_resources)
        }
        RibbonTab::MotionPlanning => {

        }
        RibbonTab::Fluids => {

        }
    }

    // UI windows
    let egui_ctx = ctxs.ctx_mut();
    let result = match ribbon.tab {
        RibbonTab::General => Ok(ui_resources.general_tab.window_ui(egui_ctx, &ui_assets)),
        RibbonTab::MotionPlanning => Ok(ui_resources.motion_planning_tab.window_ui(egui_ctx, &ui_assets)),
        _ => {Ok(())}
    };
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }

    if ctxs.ctx_mut().is_using_pointer() {
        ui_resources.selected_entities.pointer_usage_state = PointerUsageState::UiUsingPointer;
    } else {
        ui_resources.selected_entities.pointer_usage_state = PointerUsageState::NotUsed;
    }
}

/// Finish ribbon tab by adding the names of each section
macro_rules! finish_ribbon_tab {
    ($ui:expr, $rects:expr) => {
        $ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
            for (mut rect, section_name) in $rects {
                rect.max.y = ui.max_rect().max.y;
                ui.allocate_new_ui(egui::UiBuilder::new().max_rect(rect), |ui| ui.label(section_name));
            }
        });
    };
}

use crate::kinematics::ik::KinematicNode;
use crate::robot::{RapierRobotHandles, Robot, RobotPart};
pub(crate) use finish_ribbon_tab;
use crate::error::ErrorEvent;
use crate::ui::general::GeneralTab;

pub fn ribbon_functionality(
    ribbon: Res<Ribbon>,
    mut ui_resources: UiResources,
    mut gizmos_resources: GizmosUiParameters,
    mut ui_events: UiEvents,

    mut errors: EventWriter<ErrorEvent>,
) {
    let result = match ribbon.tab {
        RibbonTab::General =>
            GeneralTab::functionality(
                &mut ui_resources,
                &mut ui_events
            ),
        RibbonTab::MotionPlanning => || -> Result<()> {
            MotionPlanning::functionality(
                &mut ui_resources,
                &mut ui_events,
            )
        }(),
        _ => { Ok(()) }
    };
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }

    // Gizmos functionality
    let result = match ribbon.tab {
        RibbonTab::General => {
            GeneralTab::gizmos_functionality(&mut ui_resources, &mut gizmos_resources, &mut ui_events)
        }
        RibbonTab::MotionPlanning => {Ok(())}
        RibbonTab::Fluids => {Ok(())}
    };
    if let Err(error) = result {
        errors.send(ErrorEvent {
            error,
            location: Some(ribbon.tab.to_string())
        });
    }
}
