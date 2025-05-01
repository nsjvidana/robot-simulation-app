use bevy::prelude::{Entity, EulerRot, Transform};
use bevy_egui::egui::Ui;
use bevy::math::{Quat, Vec3};
use bevy_egui::egui;
use bevy_rapier3d::prelude::ColliderScale;
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
    
    pub fn with_scaling(mut self, positive_scale_only: bool) -> Self {
        self.scaling_options = Some(ScalingOptions {
            positive_scale_only,
            uniform_scale: true,
        });
        self
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
        
        ui.horizontal(|ui| {
            ui.label("Rotation: ");
            let (mut x, mut y, mut z) = self.transform.rotation.to_euler(EulerRot::XYZ);
                x = x.to_degrees();
                y = y.to_degrees();
                z = z.to_degrees();
            let x_changed = ui.add(egui::DragValue::new(&mut x).min_decimals(3).speed(0.1)).changed();
            let y_changed = ui.add(egui::DragValue::new(&mut y).min_decimals(3).speed(0.1)).changed();
            let z_changed = ui.add(egui::DragValue::new(&mut z).min_decimals(3).speed(0.1)).changed();
            if x_changed || y_changed || z_changed {
                self.transform.rotation = Quat::from_euler(
                    EulerRot::XYZ,
                    x.to_radians(),
                    y.to_radians(),
                    z.to_radians()
                );
            }
        });

        if let Some(scaling_options) = self.scaling_options.as_mut() {
            ui.horizontal(|ui| {
                ui.label("Scale: ");
                let x_changed = ui.add(egui::DragValue::new(&mut self.transform.scale.x).min_decimals(3).speed(0.1))
                    .changed();
                let y_changed = ui.add(egui::DragValue::new(&mut self.transform.scale.y).min_decimals(3).speed(0.1))
                    .changed();
                let z_changed = ui.add(egui::DragValue::new(&mut self.transform.scale.z).min_decimals(3).speed(0.1))
                    .changed();
                if scaling_options.uniform_scale {
                    if x_changed { self.transform.scale = Vec3::splat(self.transform.scale.x) }
                    if y_changed { self.transform.scale = Vec3::splat(self.transform.scale.y) }
                    if z_changed { self.transform.scale = Vec3::splat(self.transform.scale.z) }
                }
                if scaling_options.positive_scale_only {
                    self.transform.scale.x = self.transform.scale.x.max(0.000001);
                    self.transform.scale.y = self.transform.scale.y.max(0.000001);
                    self.transform.scale.z = self.transform.scale.z.max(0.000001);
                }
            });
            ui.checkbox(&mut scaling_options.uniform_scale, "Uniform Scale");
        }
    }

    fn functionality(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        let scale =
            if self.scaling_options.is_some() {
                res.commands.entity(entity).insert(ColliderScale::Relative(Vec3::ONE));
                self.transform.scale
            }
            else { Vec3::ONE };
        *res.global_transforms
            .get_mut(entity)
            .unwrap() = self.transform
                .with_scale(scale)
                .into();
    }

    fn cleanup(&mut self, _entity: Entity, _res: &mut PropertyFunctionalityResources) {}

    fn property_name(&self) -> &'static str {
        "Transform"
    }
}