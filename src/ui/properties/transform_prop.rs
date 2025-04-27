use crate::ui::properties::EntityProperty;
use crate::ui::{FunctionalUiResources, View};
use bevy::math::{EulerRot, Quat};
use bevy::prelude::{Commands, Entity, Transform};
use bevy_egui::egui;
use bevy_egui::egui::Ui;

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