use crate::error::error_handling_system;
use crate::ui::{FunctionalUiResources, RobotLabUiSet, UiSet, View};
use bevy::prelude::*;
use bevy_egui::egui::{Ui, Widget};
use bevy_egui::{egui, EguiContexts};

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
    /// (Entity name, Vec of entity properties)
    ///
    /// Set to [`None`] if no entity is selected
    pub entity_properties: Option<(String, Vec<Box<dyn EntityProperty>>)>,
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
        if let Some((_, props)) = &mut self.entity_properties {
            for property in props {
                property.prepare(res);
            }
        }
    }

    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        if let Some((entity_name, props)) = self.entity_properties.as_mut() {
            ui.heading(format!("Properties: {}", entity_name));
            for property in props.iter_mut() {
                ui.collapsing(property.property_name(), |ui| {
                    property.ui(ui, commands);
                });
            }
        }
        else {
            ui.heading("Properties");
            ui.label("No properties available");
        }
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        if res.keyboard.just_pressed(KeyCode::KeyN) {
            self.open = ! self.open;
        }
        if !self.open { return Ok(()); }

        if let Some((_, props)) = self.entity_properties.as_mut() {
            for property in props.iter_mut() {
                property.functionality(res)?
            }
        }
        Ok(())
    }
}

pub struct TransformProperty {
    pub entity: Entity,
    pub transform: Transform,
    pub edit_scale: bool,
    pub nonzero_scale: bool,
}

impl TransformProperty {
    pub fn new(entity: Entity, transform: Transform) -> Self {
        Self {
            entity,
            transform,
            edit_scale: false,
            nonzero_scale: false,
        }
    }

    pub fn edit_scale(mut self, nonzero_scale: bool) -> Self {
        self.edit_scale = true;
        self.nonzero_scale = nonzero_scale;
        self
    }
}

impl View for TransformProperty {
    fn prepare(&mut self, res: &mut FunctionalUiResources) {
        let TransformProperty { entity, transform, ..} = self;
        let mut curr_trans = res.transforms.get_mut(*entity).unwrap();
        *transform = (*curr_trans).into();
    }

    fn ui(&mut self, ui: &mut Ui, _commands: &mut Commands) {
        let TransformProperty {
            transform,
            edit_scale,
            nonzero_scale ,
            ..
        } = self;
        ui.horizontal(|ui| {
            ui.label("Translation: ");
            let _x_resp = ui.add(egui::DragValue::new(&mut transform.translation.x).min_decimals(3));
            let _y_resp = ui.add(egui::DragValue::new(&mut transform.translation.y).min_decimals(3));
            let _z_resp = ui.add(egui::DragValue::new(&mut transform.translation.z).min_decimals(3));
            // TODO: register undo action
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

        if *edit_scale {
            ui.horizontal(|ui| {
                ui.label("Scale: ");
                let prev_scale = transform.scale;
                let x_resp = ui.add(egui::DragValue::new(&mut transform.scale.x).min_decimals(3).speed(0.1));
                let y_resp = ui.add(egui::DragValue::new(&mut transform.scale.y).min_decimals(3).speed(0.1));
                let z_resp = ui.add(egui::DragValue::new(&mut transform.scale.z).min_decimals(3).speed(0.1));
                let changed =  x_resp.changed() || y_resp.changed() || z_resp.changed();
                if *nonzero_scale &&
                    changed &&
                    (transform.scale.x <= 0. ||
                    transform.scale.y <= 0. ||
                    transform.scale.z <= 0.)
                {
                    transform.scale = prev_scale;
                }
            });
            // TODO: register undo action
        }
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let TransformProperty { transform, entity, .. } = self;
        let mut curr_trans = res.transforms.get_mut(*entity).unwrap();
        *curr_trans = (*transform).into();
        Ok(())
    }
}

impl EntityProperty for TransformProperty {
    fn property_name(&self) -> &'static str {
        "Transform"
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

// TODO: rename this to EntityProperties
pub trait EntityProperty: View + Send + Sync {
    fn property_name(&self) -> &'static str;
}

#[macro_export]
macro_rules! entity_properties {
    ($entity_name:expr, $($rest:expr),+) => {
        ($entity_name, vec![$(Box::new($rest)),+])
    };
}
