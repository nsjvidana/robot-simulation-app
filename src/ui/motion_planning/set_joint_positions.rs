use std::ops::Deref;
use std::sync::Arc;
use bevy_egui::egui;
use crate::motion_planning::{InstructionObject, PlanEvent};
use crate::ui::{RobotLabUiAssets, UiEvents, UiResources, View, WindowUI};
use bevy_egui::egui::{Context, Ui, Widget};
use parking_lot::Mutex;
use crate::math::Real;
use crate::motion_planning::set_joint_positions::SetJointPositionsInstruction;
use crate::ui::motion_planning::{IkOutput, IkWindowUiEvent};

#[derive(Default)]
pub struct SetJointPositionsWindow {
    open: bool,
    instruction: Option<SetJointPositionsInstruction>,
    inverse_kinematics_clicked: bool,
    // Inverse kinematics data
    ik_output: Arc<Mutex<Option<IkOutput>>>,
    /// Whether the IK solver was canceled (e.g. if the IK window was closed)
    pub canceled: Arc<Mutex<bool>>,
    waiting_for_ik: bool,
}

impl View for SetJointPositionsWindow {
    fn ui(&mut self, ui: &mut Ui, ui_assets: &RobotLabUiAssets) {
        let Some(instruction) = self.instruction.clone() else {
            self.open = false;
            return;
        };

        ui.label("Joint Positions:");
        let table = egui_extras::TableBuilder::new(ui)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(
                egui_extras::Column::auto()
                    .at_least(40.0)
                    .resizable(true)
            )
            .column(
                egui_extras::Column::remainder()
                    .at_least(40.0)
                    .resizable(true)
            );
        table
            .header(20.0, |mut header_row| {
                header_row.col(|ui| { ui.strong("Joint"); });
                header_row.col(|ui| { ui.strong("Positions"); });
            })
            .body(|mut body| {
                for (joint, pos) in instruction.joints_and_positions.iter() {
                    body.row(18.0, |mut row| {
                        row.col(|ui| { ui.label(format!("{}", joint)); } );
                        row.col(|ui| { ui.label(format!("{:?}", pos)); } );
                    });
                }
            });
        // ui.withla
        self.inverse_kinematics_clicked = ui.button("Inverse Kinematics").clicked();
    }

    fn functionality(resources: &mut UiResources, events: &mut UiEvents) -> crate::prelude::Result<()> {
        let window = &mut resources.motion_planning_tab.set_joint_positions_window;
        for event in events.plan_events.get_cursor().read(&events.plan_events) {
            if let PlanEvent::EditInstructionEvent(instruction) = event {
                let lock = instruction.lock();
                if let Some(instruction_copy) = (*lock)
                    .downcast_ref::<SetJointPositionsInstruction>()
                    .cloned()
                {
                    window.open = true;
                    window.instruction = Some(instruction_copy);
                }
            }
        }

        let Some(instruction) = &mut window.instruction else {
            return Ok(());
        };
        let Some(robot_entity) = resources.selected_entities.active_robot else {
            return Ok(());
        };
        let (robot, kinematics) = resources.robot_q.get(robot_entity).unwrap();

        if !window.waiting_for_ik && window.inverse_kinematics_clicked {
            let owned_chain = k::Chain::from_nodes(kinematics.chain.iter().cloned().collect());
            events.ik_window_events.send(IkWindowUiEvent {
                chain: owned_chain,
                solver_output: window.ik_output.clone(),
                canceled: window.canceled.clone(),
                urdf: robot.urdf.clone(),
                urdf_path: robot.robot_file_path.clone(),
                target_pose: kinematics.chain.origin()
            });
            window.waiting_for_ik = true;
        }
        else if window.waiting_for_ik {
            let (ik_output, mut canceled) = (&*window.ik_output.lock(), window.canceled.lock());
            if ik_output.is_some() || *canceled {
                window.waiting_for_ik = false;
                *canceled = false;
            }

            if let Some(ik_output) = ik_output {
                let entities = ik_output.serial_chain.iter()
                    .map(|node| resources.kinematic_nodes
                        .iter()
                        .find(|node_cmp| node_cmp.0 == *node)
                    )
                    .collect::<Vec<_>>();
                println!("{}, {}", entities.len(), ik_output.positions.first().unwrap().len());
            }
        }

        Ok(())
    }
}

impl WindowUI for SetJointPositionsWindow {
    fn window_ui(&mut self, egui_ctx: &mut Context, ui_assets: &RobotLabUiAssets) {
        let mut open = self.open;
        egui::Window::new("Set Joint Positions Instruction")
            .open(&mut open)
            .show(egui_ctx, |ui| {
                self.ui(ui, ui_assets)
            });
        self.open = open;
    }
}
