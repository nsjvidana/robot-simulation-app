use crate::ui::{RobotLabUiSet, View};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use bevy_egui::{egui, EguiContexts};
use std::ops::{Deref, DerefMut};

pub fn build_app(app: &mut App) {
    app.init_resource::<Events<PropertiesSelectionEvent>>();
    app.init_resource::<PropertiesUi>();

    app.add_systems(Update, props_prepare.in_set(RobotLabUiSet::Prepare));

    app.add_systems(Update, props_ui_system.in_set(RobotLabUiSet::Ui));
}

#[derive(Component)]
pub struct EntityProperties(Vec<Box<dyn EntityProperty>>);

impl Deref for EntityProperties {
    type Target = Vec<Box<dyn EntityProperty>>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl DerefMut for EntityProperties {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

/// Global transform property
pub struct TransformProperty {
    pub transform: Transform,
    pub scaling_options: Option<ScalingOptions>,
}

impl EntityProperty for TransformProperty {
    fn prepare(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        self.transform = *res.local_transforms.get(entity).unwrap();
    }

    fn ui(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Translation: ");
            ui.add(egui::DragValue::new(&mut self.transform.translation.x).min_decimals(3).speed(0.1));
            ui.add(egui::DragValue::new(&mut self.transform.translation.y).min_decimals(3).speed(0.1));
            ui.add(egui::DragValue::new(&mut self.transform.translation.z).min_decimals(3).speed(0.1));
        });
    }

    fn functionality(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        *res.local_transforms
            .get_mut(entity)
            .unwrap() = self.transform
            .with_scale(Vec3::ONE)
            .into();
    }

    fn cleanup(&mut self, _entity: Entity, _res: &mut PropertyFunctionalityResources) {}

    fn property_name(&self) -> &'static str {
        "Transform"
    }
}

pub struct ScalingOptions {
    pub non_negative_scale: bool,
    pub uniform_scale: bool,
    pub uniform_scale_subdivisions: u32,
}

impl Default for ScalingOptions {
    fn default() -> Self {
        Self {
            non_negative_scale: true,
            uniform_scale: true,
            uniform_scale_subdivisions: 16,
        }
    }
}

pub trait EntityProperty: Send + Sync {
    fn prepare(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources);

    fn ui(&mut self, ui: &mut Ui);

    fn functionality(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources);

    fn cleanup(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources);

    fn property_name(&self) -> &'static str;
}

#[derive(SystemParam)]
pub struct PropertyFunctionalityResources<'w, 's> {
    pub global_transforms: Query<'w, 's, &'static mut GlobalTransform>,
    pub local_transforms: Query<'w, 's, &'static mut Transform>,
    pub keyboard: Res<'w, ButtonInput<KeyCode>>,
}

#[derive(Event)]
pub struct PropertiesSelectionEvent {
    pub entity: Entity,
    pub name: String,
}

#[derive(Resource, Default)]
pub struct PropertiesUi {
    entity: Option<(String, Entity)>,
    panel_open: bool,
}

impl PropertiesUi {
    fn set_entity(
        &mut self,
        name: String,
        entity: Entity,
        entity_props: &mut Query<&mut EntityProperties>,
        res: &mut PropertyFunctionalityResources
    ) -> &mut Self {
        // First cleanup current properties, if any.
        if let Some((_, entity)) = self.entity.take() {
            let mut props = entity_props.get_mut(entity).unwrap();
            for p in props.iter_mut() {
                p.cleanup(entity, res);
            }
        }

        // then set entity.
        self.entity = Some((name, entity));
        self
    }

    pub fn panel_ui(
        &mut self,
        ctx: &mut egui::Context,
        entity_props: &mut Query<&mut EntityProperties>,
    ) {
        if !self.panel_open { return; };
        egui::SidePanel::right("properties_panel")
            .show(ctx, |ui| {
                self.ui(ui, entity_props);
            });
    }

    pub fn ui(&mut self, ui: &mut Ui, entity_props: &mut Query<&mut EntityProperties>) {
        if self.entity.is_none() {
            ui.heading("Properties");
            ui.label("No properties available.");
            return;
        }

        let (name, entity) = self.entity.as_ref().unwrap();
        ui.heading(format!("{} Properties", name));
        let mut props = entity_props.get_mut(*entity).unwrap();
        for p in props.iter_mut() {
            ui.collapsing(p.property_name(), |ui| {
                p.ui(ui);
            });
        }
    }

    pub fn functionality(&mut self, entity_props: &mut Query<&mut EntityProperties>, res: &mut PropertyFunctionalityResources) {
        if res.keyboard.just_pressed(KeyCode::KeyN) {
            self.panel_open = !self.panel_open;
        }

        let Some((_, entity)) = self.entity else { return; };
        let mut properties = entity_props.get_mut(entity).unwrap();
        for p in properties.iter_mut() {
            p.functionality(entity, res);
        }
    }
}

pub fn props_prepare(
    mut props_ui: ResMut<PropertiesUi>,
    mut entity_props: Query<&mut EntityProperties>,
    mut props_res: PropertyFunctionalityResources,
    mut events: ResMut<Events<PropertiesSelectionEvent>>
) {
    for event in events.update_drain() {
        props_ui.set_entity(
            event.name,
            event.entity,
            &mut entity_props,
            &mut props_res
        );
    }
}

pub fn props_ui_system(
    mut props_ui: ResMut<PropertiesUi>,
    mut egui_context: EguiContexts,
    mut entity_props: Query<&mut EntityProperties>,
) {
    props_ui.panel_ui(egui_context.ctx_mut(), &mut entity_props);
}

pub fn props_functionality_system(
    mut props_ui: ResMut<PropertiesUi>,
    mut props_res: PropertyFunctionalityResources,
    mut entity_props: Query<&mut EntityProperties>,
) {
    props_ui.functionality(&mut entity_props, &mut props_res);
}
