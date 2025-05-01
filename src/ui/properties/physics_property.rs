use crate::ui::drag_value_decimals;
use crate::ui::properties::{EntityProperty, PropertyFunctionalityResources};
use bevy::prelude::Entity;
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use bevy_rapier3d::prelude::{ColliderMassProperties, RigidBody};

#[derive(Default)]
pub struct PhysicsProperty {
    physics_data: PhysicsData,
    data_changed: bool,
}

impl PhysicsProperty {
    pub fn new() -> Self {
        Self::default()
    }
}

impl EntityProperty for PhysicsProperty {
    fn prepare(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        let rb = *res.bodies.get(entity).unwrap();
        let mprops = res.mass_properties.get(entity).unwrap().clone();
        self.physics_data.body_type = rb;
        self.physics_data.mprops = mprops;
    }

    fn ui(&mut self, ui: &mut Ui) {
        let physics = &mut self.physics_data;

        let txt = match physics.body_type {
            RigidBody::Dynamic => "Dynamic",
            RigidBody::Fixed => "Fixed",
            RigidBody::KinematicPositionBased => "Kinematic Position-Based",
            RigidBody::KinematicVelocityBased => "Kinematic Velocity-Based",
        };
        let prev_rb_type = physics.body_type;
        egui::ComboBox::from_label("Rigid Body Type")
            .selected_text(txt)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut physics.body_type,
                    RigidBody::Fixed,
                    "Fixed"
                );
                ui.selectable_value(
                    &mut physics.body_type,
                    RigidBody::Dynamic,
                    "Dynamic"
                );
            });
        let rb_type_changed = physics.body_type != prev_rb_type;

        let ColliderMassProperties::MassProperties(mprops) = &mut physics.mprops else {
            return;
        };
        let prev_mass = mprops.mass;
        let mass_changed = drag_value_decimals!(ui, "Mass (kg): ", mprops.mass, 3)
            .changed();
        if mprops.mass <= 0. {
            mprops.mass = prev_mass;
        }

        self.data_changed = rb_type_changed || mass_changed;
    }

    fn functionality(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        if self.data_changed {
            let mut rb = res.bodies.get_mut(entity).unwrap();
            let mut mprops = res.mass_properties.get_mut(entity).unwrap();
            *rb = self.physics_data.body_type;
            *mprops = self.physics_data.mprops;
        }
    }

    fn cleanup(&mut self, _entity: Entity, _res: &mut PropertyFunctionalityResources) {}

    fn property_name(&self) -> &'static str { "Physics" }
}


#[derive(Default)]
struct PhysicsData {
    body_type: RigidBody,
    mprops: ColliderMassProperties,
}
