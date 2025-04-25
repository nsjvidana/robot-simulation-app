use crate::functionality::motion_planning::set_joint_positions::SetJointPositionsInstruction;
use crate::impl_instruction_window;
use crate::ui::ribbon::motion_planning::ik::{IkOutput, IkWindow};
use crate::ui::ribbon::motion_planning::InstructionInput;
use crate::ui::{drag_value, FunctionalUiResources, NoOutput, OpenUtilityWindow, UtilityOutputEvent, UtilityOutputResource, View, WindowUi};
use bevy::prelude::{Commands, Entity, Res, Resource, Trigger};
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use derivative::Derivative;
use parking_lot::Mutex;
use std::ops::Deref;
use std::sync::Arc;

#[derive(Resource, Derivative)]
#[derivative(Default)]
pub struct SetJointPositionsWindow {
    input: Option<InstructionInput>,
    out: Option<()>,
    ik_utility_data: Option<(Entity, IkOutputArc)>,
    instruction_clone: Option<SetJointPositionsInstruction>,

    save_clicked: bool,
    should_close: bool,
    open: bool,
    #[derivative(Default(value = "true"))]
    canceled: bool,
}

impl WindowUi for SetJointPositionsWindow {
    fn window_ui(&mut self, egui_context: &mut egui::Context, commands: &mut Commands) {
        let mut open = self.open;
        egui::Window::new("Set Joint Positions")
            .open(&mut open)
            .show(egui_context, |ui| {
                self.ui(ui, commands);
            });
        self.open = open;

        if self.should_close && self.input.is_some() {
            self.instruction_clone = None;
            self.input = None;
            self.out = None;
            self.save_clicked = false;
            self.should_close = true;
            self.open = false;
            self.canceled = true;
            if let Some((observer, _)) = self.ik_utility_data.take() {
                commands.entity(observer).despawn();
            }
        }

        self.canceled = !self.open;
    }

    fn set_open(&mut self, open: bool) {
        self.open = open;
    }
}

impl View for SetJointPositionsWindow {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        let Some(input) = &self.input else {
            return;
        };
        let Some(instruction_clone) = self.instruction_clone.as_mut() else {
            return;
        };
        
        drag_value!(ui, "Epsilon: ", instruction_clone.eps);

        // Joint positions table
        ui.label("Joint Positions:");
        {
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
                    for (joint_name, _, pos) in instruction_clone.joints_and_positions.iter() {
                        body.row(18.0, |mut row| {
                            row.col(|ui| { ui.label(format!("{}", joint_name)); });
                            row.col(|ui| { ui.label(format!("{:?}", pos)); });
                        });
                    }
                });
        }

        let ik_btn = ui.button("Inverse Kinematics");
            if ik_btn.hovered() {
                ik_btn.show_tooltip_text("Create joint positions using inverse kinematics");
            }
        if ik_btn.clicked() && self.ik_utility_data.is_none() {
            commands.send_event(OpenUtilityWindow::<IkWindow> {
                input: input.robot,
            });
            let ik_output = IkOutputArc::default();
            commands.insert_resource(UtilityOutputResource(ik_output.clone()));
            self.ik_utility_data = Some((
                commands.add_observer(ik_utility_observer_system).id(),
                ik_output
            ));
        }

        self.save_clicked = ui.button("Save").clicked();
    }

    fn view_name(&self) -> &'static str {
        "Set Joint Positions Instruction"
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let Some(input) = &self.input else {
            return Ok(());
        };
        self.instruction_clone = self.instruction_clone
            .take()
            .or_else(|| {
                let mut guard = input.instruction.lock();
                (*guard).downcast_mut::<SetJointPositionsInstruction>().cloned()
            });
        let Some(instruction) = &mut self.instruction_clone else {
            panic!("Didn't add correct instruction type to SetJointPositionsWindow")
        };

        let mut wipe_ik_data = false;
        if let Some((observer, output)) = self.ik_utility_data.as_ref() {
            let mut output = output.lock();
            if let Some(mut output) = output.take() {
                if let Some(positions) = output.pop() {
                    instruction.joints_and_positions = positions;
                }
                else {
                    instruction.joints_and_positions.clear();
                }
                res.commands.entity(*observer).despawn();
                wipe_ik_data = true;
            }
        }
        if wipe_ik_data {
            self.ik_utility_data = None;
        }

        if self.save_clicked {
            *input.instruction.lock() = Box::new(self.instruction_clone.take().unwrap());
            self.instruction_clone = None;
            self.input = None;
            self.out = None;
            self.save_clicked = false;
            self.should_close = false;
            self.open = false;
            self.canceled = true;
            if let Some((observer, _)) = self.ik_utility_data.take() {
                res.commands.entity(observer).despawn();
            }
        }

        Ok(())
    }
}

impl_instruction_window!(SetJointPositionsWindow, SetJointPositionsInstruction, NoOutput, input, out, canceled);

#[derive(Default, Clone)]
struct IkOutputArc(Arc<Mutex<Option<IkOutput>>>);

impl Deref for IkOutputArc {
    type Target = Arc<Mutex<Option<IkOutput>>>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

fn ik_utility_observer_system(
    trigger: Trigger<UtilityOutputEvent<IkWindow>>,
    res: Res<UtilityOutputResource<IkOutputArc>>,
) {
    *res.lock() = trigger.output.clone();
}
