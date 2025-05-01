use crate::ui::properties::EntityProperty;
use crate::ui::{FunctionalUiResources, View};
use bevy::math::{EulerRot, Quat};
use bevy::prelude::{Commands, Entity, Transform, Vec3};
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use bevy_rapier3d::prelude::ColliderScale;

pub struct TransformProperty {
    pub entity: Entity,
    pub transform: Transform,
    pub scaling_options: Option<ScalingOptions>,
}

impl TransformProperty {
    pub fn new(entity: Entity, transform: Transform) -> Self {
        Self {
            entity,
            transform,
            scaling_options: None,
        }
    }

    /// Enable editing the transform's scale, and specify whether the scale can be negative or not.
    pub fn edit_scale(mut self, non_negative_scale: bool) -> Self {
        let mut options = ScalingOptions::default();
        options.non_negative_scale = non_negative_scale;
        self.scaling_options = Some(options);
        self
    }
}

impl View for TransformProperty {
    fn prepare(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let TransformProperty { entity, transform, ..} = self;
        let scale = transform.scale;
        let mut curr_trans = res.transforms.get_mut(*entity).unwrap();
        *transform = Transform::from(*curr_trans);
        // Conserve scale
        transform.scale = scale;
        Ok(())
    }

    fn ui(&mut self, ui: &mut Ui, _commands: &mut Commands) {
        let TransformProperty {
            transform,
            scaling_options,
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

        if let Some(scaling_options) = scaling_options {
            ui.horizontal(|ui| {
                ui.label("Scale: ");
                let prev_scale = transform.scale;

                let uniform_resp = ui.checkbox(&mut scaling_options.uniform_scale, "Uniform Scale");
                let x_resp = ui.add(egui::DragValue::new(&mut transform.scale.x).min_decimals(3).speed(0.1));
                let y_resp = ui.add(egui::DragValue::new(&mut transform.scale.y).min_decimals(3).speed(0.1));
                let z_resp = ui.add(egui::DragValue::new(&mut transform.scale.z).min_decimals(3).speed(0.1));

                let changed =  x_resp.changed() || y_resp.changed() || z_resp.changed() || uniform_resp.changed();
                if changed {
                    // Ensure that the scale is a nonzero scale
                    if scaling_options.non_negative_scale &&
                        (transform.scale.x <= 0. ||
                            transform.scale.y <= 0. ||
                            transform.scale.z <= 0.)
                    {
                        transform.scale = prev_scale;
                    }
                    // Ensure that the scale is uniform
                    if scaling_options.uniform_scale {
                        if x_resp.changed() {
                            transform.scale = Vec3::splat(transform.scale.x);
                        }
                        if y_resp.changed() {
                            transform.scale = Vec3::splat(transform.scale.y);
                        }
                        if z_resp.changed() {
                            transform.scale = Vec3::splat(transform.scale.z);
                        }
                    }
                    // TODO: register undo action
                }
                });
            if !scaling_options.uniform_scale {
                ui.horizontal(|ui| {
                    ui.label("Non-Uniform Scale Subdivisions: ");
                    ui.add(
                        egui::DragValue::new(&mut scaling_options.uniform_scale_subdivisions).min_decimals(0)
                    );
                });
            }
        }
    }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let TransformProperty { transform, entity, scaling_options, .. } = self;
        *res.local_transforms
            .get_mut(*entity)
            .unwrap() = transform
                .with_scale(Vec3::ONE)
                .into();
        if scaling_options.is_some() {
            // Update collider scales
            for coll_e in res.colliders
                .iter_mut()
                .filter(|(e, _, par)| par.is_some_and(|p| p.get() == *entity) || *e == *entity)
                .map(|v| v.0)
            {
                res.commands.entity(coll_e)
                    .insert(ColliderScale::Relative(Vec3::ONE));
                if let Ok(mut trans) = res.local_transforms.get_mut(coll_e) {
                    *trans = trans.with_scale(transform.scale).into();
                }
            }
            // Udpate scale of visuals
            for vis_e in res.visual_entities
                .iter_mut()
                .filter(|(e, par)| par.is_some_and(|p| p.get() == *entity) || *e == *entity)
                .map(|v| v.0)
            {
                if let Ok(mut trans) = res.local_transforms.get_mut(vis_e) {
                    *trans = trans.with_scale(transform.scale).into();
                }
            }
        }
        Ok(())
    }

    fn cleanup(&mut self, _functional_resources: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        Ok(())
    }
}

impl EntityProperty for TransformProperty {
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
