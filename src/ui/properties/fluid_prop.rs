use bevy::prelude::{Click, Component, Entity, GlobalTransform, Pointer, Trigger, Visibility};
use bevy::math::bounding::Aabb3d;
use bevy::math::{Isometry3d, Vec3};
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use std::sync::Arc;
use bevy_egui::egui;
use parking_lot::Mutex;
use bevy_rapier3d::geometry::{Collider, Sensor};
use bevy_rapier3d::picking_backend::RapierPickable;
use bevy_rapier3d::prelude::ColliderDebug;
use bevy_egui::egui::Ui;
use bevy_salva3d::utils::cube_particle_positions;
use crate::ui::properties::{EntityProperty, PropertyFunctionalityResources};
use crate::ui::toolbar::MovableEntityPickedEvent;

pub struct FluidProperty {
    aabb_entities: Option<[FluidBoundEntity; 2]>,
    aabb_changed: bool
}

impl FluidProperty {
    pub fn new() -> Self {
        Self {
            aabb_entities: None,
            aabb_changed: false,
        }
    }
}

impl EntityProperty for FluidProperty {
    fn prepare(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        if let Some([
            min,
            max
        ]) = self.aabb_entities.as_mut()
        {
            let prev_min = min.pos;
            let prev_max = max.pos;
            min.pos = res.global_transforms
                .get(min.entity)
                .unwrap()
                .translation();
            max.pos = res.global_transforms
                .get(max.entity)
                .unwrap()
                .translation();
            self.aabb_changed = min.pos != prev_min || max.pos != prev_max;
            return;
        }

        let positions = res.fluids.get(entity).unwrap();
        let aabb = Aabb3d::from_point_cloud(
            Isometry3d::default(),
            positions
                .iter()
                .cloned()
        );
        let diameter = SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS * 2.;
        let min_picked = Arc::new(Mutex::new(false));
        let max_picked = Arc::new(Mutex::new(false));
        let min_picked_clone = min_picked.clone();
        let max_picked_clone = max_picked.clone();
        let min_entity = res.commands
            .spawn((
                GlobalTransform::from_translation(aabb.min.into()),
                Collider::cuboid(diameter, diameter, diameter),
                Sensor,
                Visibility::default(),
                RapierPickable,
                ColliderDebug::AlwaysRender,
            ))
            .observe(move |trigger: Trigger<Pointer<Click>>| *min_picked_clone.lock() = true)
            .id();
        let max_entity = res.commands
            .spawn((
                GlobalTransform::from_translation(aabb.max.into()),
                Collider::cuboid(diameter, diameter, diameter),
                Sensor,
                Visibility::default(),
                RapierPickable,
                ColliderDebug::AlwaysRender,
            ))
            .observe(move |trigger: Trigger<Pointer<Click>>| *max_picked_clone.lock() = true)
            .id();
        self.aabb_entities = Some([
            FluidBoundEntity {
                entity: min_entity,
                pos: aabb.min.into(),
                picked: min_picked
            },
            FluidBoundEntity {
                entity: max_entity,
                pos: aabb.max.into(),
                picked: max_picked
            },
        ]);
        self.aabb_changed = false;
    }

    fn ui(&mut self, ui: &mut Ui) {
        let Some([min, max]) = self.aabb_entities.as_mut()
        else { return; };

        // Minimum pos
        ui.horizontal(|ui| {
            ui.label("Minimum: ");
            let x = ui.add(egui::DragValue::new(&mut min.pos.x).min_decimals(3).speed(0.1));
            let y = ui.add(egui::DragValue::new(&mut min.pos.y).min_decimals(3).speed(0.1));
            let z = ui.add(egui::DragValue::new(&mut min.pos.z).min_decimals(3).speed(0.1));
            let changed = x.changed() || y.changed() || z.changed();
            if changed {
                self.aabb_changed = true;
            }
        });

        // Maximum pos
        ui.horizontal(|ui| {
            ui.label("Maximum: ");
            let x = ui.add(egui::DragValue::new(&mut max.pos.x).max_decimals(3).speed(0.1));
            let y = ui.add(egui::DragValue::new(&mut max.pos.y).max_decimals(3).speed(0.1));
            let z = ui.add(egui::DragValue::new(&mut max.pos.z).max_decimals(3).speed(0.1));
            let changed = x.changed() || y.changed() || z.changed();
            if changed {
                self.aabb_changed = true;
            }
        });
    }

    fn functionality(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        let Some([min, max]) = self.aabb_entities.as_mut()
        else { return; };

        if *min.picked.lock() {
            *min.picked.lock() = false;
            res.commands.send_event(MovableEntityPickedEvent(min.entity));
        }

        if *max.picked.lock() {
            *max.picked.lock() = false;
            res.commands.send_event(MovableEntityPickedEvent(max.entity));
        }

        if self.aabb_changed {
            *res.global_transforms
                .get_mut(min.entity)
                .unwrap() = GlobalTransform::from_translation(min.pos);
            *res.global_transforms
                .get_mut(max.entity)
                .unwrap() = GlobalTransform::from_translation(max.pos);

            let r = SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS;
            let diameter = r * 2.;
            let center = (min.pos + max.pos) / 2.;
            let diagonal = max.pos - min.pos;
            let x_count =  (diagonal.x.abs() / diameter) as usize;
            let y_count =  (diagonal.y.abs() / diameter) as usize;
            let z_count =  (diagonal.z.abs() / diameter) as usize;
            let mut positions = cube_particle_positions(x_count, y_count, z_count, r);
                positions
                    .iter_mut()
                    .for_each(|pos| *pos = *pos + center);
            let mut curr_positions = res.fluids.get_mut(entity).unwrap();
                **curr_positions = positions;
        }
    }

    fn cleanup(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        let Some([min, max]) = self.aabb_entities.take()
        else { return; };
        res.commands.entity(min.entity).despawn();
        res.commands.entity(max.entity).despawn();
    }

    fn property_name(&self) -> &'static str { "Fluid" }
}

#[derive(Component)]
struct FluidBoundEntity {
    entity: Entity,
    pos: Vec3,
    picked: Arc<Mutex<bool>>
}