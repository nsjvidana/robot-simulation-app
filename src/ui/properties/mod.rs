pub mod physics_property;
pub mod transform_prop;

use crate::ui::{RobotLabUiSet, View};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use bevy_egui::{egui, EguiContexts};
use std::ops::{Deref, DerefMut};
use bevy_rapier3d::prelude::{ColliderMassProperties, RigidBody};
use bevy_salva3d::fluid::FluidPositions;
use crate::ui::selecting::SceneWindowData;

pub fn build_app(app: &mut App) {
    app.init_resource::<Events<PropertiesSelectionEvent>>();
    app.init_resource::<PropertiesUi>();

    app.add_systems(Update, props_prepare.in_set(RobotLabUiSet::Prepare));

    app.add_systems(Update,
        (
            props_ui_system.in_set(RobotLabUiSet::Ui),
            props_functionality_system.in_set(RobotLabUiSet::Functionality)
        )
    );
}

pub struct AddEntityPropertiesCommand {
    props: Vec<Box<dyn EntityProperty>>,
}

impl EntityCommand for AddEntityPropertiesCommand {
    fn apply(self, entity: Entity, world: &mut World) {
        let props = world.query::<&mut EntityProperties>()
            .get_mut(world, entity);
        if let Ok(mut props) = props {
            let mut ps = self.props;
            props.append(&mut ps);
        }
        else {
            world.entity_mut(entity).insert(EntityProperties(self.props));
        }
    }
}

pub trait EntityPropertiesExt {
    fn insert_entity_properties(&mut self, props: Vec<Box<dyn EntityProperty>>) -> &mut Self;
}

impl<'a, 's> EntityPropertiesExt for EntityCommands<'a> {
    fn insert_entity_properties(&mut self, props: Vec<Box<dyn EntityProperty>>) -> &mut Self {
        self.queue(AddEntityPropertiesCommand {
            props,
        });
        self
    }
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
    pub bodies: Query<'w, 's, &'static mut RigidBody>,
    pub mass_properties: Query<'w, 's, &'static mut ColliderMassProperties>,
    pub fluids: Query<'w, 's, &'static mut FluidPositions>,
    pub global_transforms: Query<'w, 's, &'static mut GlobalTransform>,
    pub local_transforms: Query<'w, 's, &'static mut Transform>,
    pub keyboard: Res<'w, ButtonInput<KeyCode>>,
    pub mouse: Res<'w, ButtonInput<MouseButton>>,
    pub scene_window_data: Res<'w, SceneWindowData>,
    pub commands: Commands<'w, 's>,
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
    pub fn prepare(&mut self, entity_props: &mut Query<&mut EntityProperties>, res: &mut PropertyFunctionalityResources) {
        if !self.panel_open {
            return;
        }
        let Some((_, entity)) = self.entity else { return; };
        let mut props = entity_props.get_mut(entity).unwrap();
        for p in props.iter_mut() {
            p.prepare(entity, res);
        }
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
            // Cleanup when closing panel
            if !self.panel_open && self.entity.is_some() {
                self.cleanup(entity_props, res);
            }
        }
        if !self.panel_open { return; }

        let Some((_, entity)) = self.entity else { return; };
        let mut properties = entity_props.get_mut(entity).unwrap();
        for p in properties.iter_mut() {
            p.functionality(entity, res);
        }
    }

    fn set_entity(
        &mut self,
        name: String,
        entity: Entity,
        entity_props: &mut Query<&mut EntityProperties>,
        res: &mut PropertyFunctionalityResources
    ) -> &mut Self {
        // First cleanup current properties, if any,
        self.cleanup(entity_props, res);
        // then set entity.
        self.entity = Some((name, entity));
        self
    }

    fn cleanup(&mut self, entity_props: &mut Query<&mut EntityProperties>, res: &mut PropertyFunctionalityResources) {
        if let Some((_, entity)) = self.entity {
            let mut props = entity_props.get_mut(entity).unwrap();
            for p in props.iter_mut() {
                p.cleanup(entity, res);
            }
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
    // Call prepare() on properties
    props_ui.prepare(&mut entity_props, &mut props_res);
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

#[macro_export]
macro_rules! box_vec {
    ($($rest:expr),+) => {
        vec![$(Box::new($rest)),+]
    };
}
