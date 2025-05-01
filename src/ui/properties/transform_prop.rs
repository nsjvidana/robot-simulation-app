use bevy::prelude::{Entity, Transform};
use bevy_egui::egui::Ui;
use bevy::math::Vec3;
use bevy_egui::egui;
use crate::ui::properties::{EntityProperty, PropertyFunctionalityResources, ScalingOptions};

/// Global transform property
pub struct TransformProperty {
    pub transform: Transform,
    pub scaling_options: Option<ScalingOptions>,
}

impl TransformProperty {
    pub fn new() -> Self {
        Self {
            transform: Transform::IDENTITY,
            scaling_options: None,
        }
    }
}

impl EntityProperty for TransformProperty {
    fn prepare(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        self.transform = res.global_transforms.get(entity).unwrap().compute_transform();
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
        *res.global_transforms
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