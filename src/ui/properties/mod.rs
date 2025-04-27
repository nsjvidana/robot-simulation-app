pub mod transform_prop;

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
