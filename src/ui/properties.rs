use crate::ui::{FunctionalUiResources, RobotLabUiSet, UiSet, View};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::Ui;
use crate::error::error_handling_system;

pub fn build_app(app: &mut App) {
    app.init_resource::<PropertiesUi>();

    let properties_prep = |
        mut properties: ResMut<PropertiesUi>,
        mut res: FunctionalUiResources,
    | {
        properties.prepare(&mut res);
    };

    let properties_ui_system = |
        mut properties: ResMut<PropertiesUi>,
        mut ctxs: EguiContexts,
        mut commands: Commands,
    | { properties.panel_ui(ctxs.ctx_mut(), &mut commands); };

    let properties_functionality = |
        mut properties: ResMut<PropertiesUi>,
        mut res: FunctionalUiResources,
    | { properties.functionality(&mut res) };

    app.add_systems(
        Update,
        (
            properties_prep
                .in_set(RobotLabUiSet::Prepare),
            properties_ui_system
                .in_set(RobotLabUiSet::Ui)
                .after(UiSet::RibbonUi),
            properties_functionality.pipe(error_handling_system)
                .in_set(RobotLabUiSet::Functionality),
        )
    );
}

#[derive(Resource, Default)]
pub struct PropertiesUi {
    pub selected_entity: Option<EntityProperties>,
    open: bool,
}

impl PropertiesUi {
    pub fn panel_ui(&mut self, ctx: &mut egui::Context, commands: &mut Commands) {
        if !self.open { return; }
        egui::SidePanel::right("properties_panel")
            .show(ctx, |ui| {
                ui.vertical(|ui| {
                    self.ui(ui, commands);
                });
            });
    }
}

impl View for PropertiesUi {
    fn prepare(&mut self, res: &mut FunctionalUiResources) {
        let Some(props) = &mut self.selected_entity else {
            return;
        };
        match props {
            EntityProperties::Robot { entity, transform, .. } => {
                let mut curr_trans = res.transforms.get_mut(*entity).unwrap();
                *transform = (*curr_trans).into();
            }
            EntityProperties::GenericObject { .. } => {}
        }
    }

    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        let Some(props) = &mut self.selected_entity else {
            ui.heading("Properites (None)");
            return;
        };
        match props {
            EntityProperties::Robot { name, transform, .. } => {
                ui.heading(format!("\"{name}\" Robot Properties"));
                ui.collapsing("Transform (XYZ)", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Translation: ");
                        let x_resp = ui.add(egui::DragValue::new(&mut transform.translation.x).min_decimals(3));
                        let y_resp = ui.add(egui::DragValue::new(&mut transform.translation.y).min_decimals(3));
                        let z_resp = ui.add(egui::DragValue::new(&mut transform.translation.z).min_decimals(3));
                    });

                    ui.horizontal(|ui| {
                        let (mut x, mut y, mut z) = transform.rotation.to_euler(EulerRot::XYZ);
                            x = x.to_degrees();
                            y = y.to_degrees();
                            z = z.to_degrees();
                        ui.label("Rotation: ");
                        let x_resp = ui.add(egui::DragValue::new(&mut x).min_decimals(3));
                        let y_resp = ui.add(egui::DragValue::new(&mut y).min_decimals(3));
                        let z_resp = ui.add(egui::DragValue::new(&mut z).min_decimals(3));
                        let changed = x_resp.changed() ||
                            y_resp.changed() ||
                            z_resp.changed();
                        if changed {
                            transform.rotation = Quat::from_euler(EulerRot::XYZ, x.to_radians(), y.to_radians(), z.to_radians());
                            // TODO: register undo action
                        }
                    });
                });
            }
            EntityProperties::GenericObject { entity, transform, .. } => {
                todo!()
            }
        }
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        if res.keyboard.just_pressed(KeyCode::KeyN) {
            self.open = ! self.open;
        }
        if !self.open { return Ok(()); }

        let Some(props) = &mut self.selected_entity else {
            return Ok(());
        };
        match props {
            EntityProperties::Robot { entity, transform,.. } => {
                let mut curr_trans = res.transforms.get_mut(*entity).unwrap();
                *curr_trans = (*transform).into();
            }
            EntityProperties::GenericObject { entity, .. } => {}
        }

        Ok(())
    }
}

pub enum EntityProperties {
    Robot {
        entity: Entity,
        name: String,
        transform: Transform,
    },
    GenericObject {
        entity: Entity,
        transform:  Transform,
    },
}
