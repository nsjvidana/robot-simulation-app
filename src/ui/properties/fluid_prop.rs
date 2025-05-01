use crate::ui::properties::{EntityProperty, PropertyFunctionalityResources};
use crate::ui::toolbar::MovableEntityPickedEvent;
use crate::prelude::Real;
use bevy::math::bounding::Aabb3d;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{Click, Component, Entity, GlobalTransform, Pointer, Transform, Trigger, Visibility};
use bevy_egui::egui;
use bevy_egui::egui::Ui;
use bevy_rapier3d::geometry::{Collider, Sensor};
use bevy_rapier3d::picking_backend::RapierPickable;
use bevy_rapier3d::prelude::{ColliderDebug, ColliderScale};
use bevy_salva3d::plugin::{AppendNonPressureForces, SalvaPhysicsPlugin};
use bevy_salva3d::salva::geometry::ParticlesContacts;
use bevy_salva3d::salva::object::{Boundary, Fluid};
use bevy_salva3d::salva::solver::{Akinci2013SurfaceTension, ArtificialViscosity, Becker2009Elasticity, NonPressureForce};
use bevy_salva3d::salva::TimestepManager;
use bevy_salva3d::utils::cube_particle_positions;
use parking_lot::{Mutex, MutexGuard};
use std::sync::Arc;
use bevy_salva3d::fluid::FluidNonPressureForces;

pub struct FluidProperty {
    aabb_entities: Option<[FluidBoundEntity; 2]>,
    aabb_changed: bool,
    nonpressure_forces: NonpressureForceList,
    forces_added: bool,
    npforce_modal_open: bool,
}

impl FluidProperty {
    pub fn new() -> Self {
        Self {
            aabb_entities: None,
            aabb_changed: false,
            nonpressure_forces: NonpressureForceList::new(),
            npforce_modal_open: false,
            forces_added: false,
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

        ui.strong("Nonpressure forces:");
        ui.group(|ui| {
            let mut forces = self.nonpressure_forces.lock();
            for (idx, f) in forces.iter_mut().enumerate() {
                egui::CollapsingHeader::new(f.force_name())
                    .id_salt(idx)
                    .show(ui, |ui| {
                        f.ui(ui);
                    });
            }
        });

        // Nonpressure force modal
        if ui.button("Add force").clicked() {
            self.npforce_modal_open = true;
        }
        if self.npforce_modal_open {
            let mut forces = self.nonpressure_forces.lock();
            let modal = egui::Modal::new(egui::Id::new("np_force_modal"))
                .show(ui.ctx(), |ui| {
                    ui.heading("Choose a Nonpressure Force");
                    let mut force_was_picked = false;
                    ui.group(|ui| {
                        if ui.button("Surface Tension").clicked() {
                            forces.push(Box::new(
                                Akinci2013SurfaceTensionInterface::new(1., 10.)
                            ));
                            force_was_picked = true;
                        }
                        if ui.button("Artificial Viscosity").clicked() {
                            forces.push(Box::new(
                                ArtificialViscosity::new(0.01, 0.01)
                            ));
                            force_was_picked = true;
                        }
                        if ui.button("Becker Elasticity").clicked() {
                            forces.push(Box::new(
                                Becker2009ElasticityInterface::new(100_000.0, 0.3, true)
                            ));
                            force_was_picked = true;
                        }
                    });
                    if force_was_picked {
                        self.npforce_modal_open = false;
                    }
                });
            if modal.should_close() {
                self.npforce_modal_open = false;
            }
        }
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
            let diff = max.pos - min.pos;
            let x_count =  (diff.x.abs() / diameter) as usize;
            let y_count =  (diff.y.abs() / diameter) as usize;
            let z_count =  (diff.z.abs() / diameter) as usize;
            let mut positions = cube_particle_positions(x_count, y_count, z_count, r);
                positions
                    .iter_mut()
                    .for_each(|pos| *pos = *pos + center);
            let mut curr_positions = res.fluids.get_mut(entity).unwrap();
                **curr_positions = positions;

            let mut coll_trans = res.global_transforms.get_mut(entity).unwrap();
                *coll_trans = Transform::from(*coll_trans).with_scale(diff).into();

            res.commands.entity(entity)
                .insert((
                    ColliderScale::Relative(Vec3::ONE),
                ));
        }

        if !self.forces_added {
            let forces = self.nonpressure_forces.clone();
            res.commands
                .entity(entity)
                .insert(AppendNonPressureForces(vec![Box::new(forces)]));
            self.forces_added = true;
        }
    }

    fn cleanup(&mut self, entity: Entity, res: &mut PropertyFunctionalityResources) {
        let Some([min, max]) = self.aabb_entities.take()
        else { return; };
        res.commands.entity(min.entity).despawn();
        res.commands.entity(max.entity).despawn();


        res.commands.entity(entity)
            .insert(ColliderDebug::NeverRender);
    }

    fn property_name(&self) -> &'static str { "Fluid" }
}

#[derive(Component)]
struct FluidBoundEntity {
    entity: Entity,
    pos: Vec3,
    picked: Arc<Mutex<bool>>
}

/// List of Nonpressure forces that can be handled by Salva and RobotLab at the same time via Mutex.
///
/// This list can be added as a Nonpressure force to a fluid.
#[derive(Clone)]
pub struct NonpressureForceList(Arc<Mutex<Vec<Box<dyn NonpressureForceUi>>>>);

impl NonpressureForceList {
    pub fn new() -> Self {
        NonpressureForceList(Arc::new(Mutex::new(Vec::new())))
    }

    pub fn lock(&self) -> MutexGuard<Vec<Box<dyn NonpressureForceUi>>> {
        self.0.lock()
    }
}

impl NonPressureForce for NonpressureForceList {
    fn solve(
        &mut self,
        timestep: &TimestepManager,
        kernel_radius: Real,
        fluid_fluid_contacts: &ParticlesContacts,
        fluid_boundaries_contacts: &ParticlesContacts,
        fluid: &mut Fluid,
        boundaries: &[Boundary],
        densities: &[Real]
    ) {
        let mut forces = self.lock();
        for f in  forces.iter_mut() {
            f.solve(
                timestep,
                kernel_radius,
                fluid_fluid_contacts,
                fluid_boundaries_contacts,
                fluid,
                boundaries,
                densities
            );
        }
    }

    fn apply_permutation(&mut self, permutation: &[usize]) {
        let mut forces = self.lock();
        for f in forces.iter_mut() {
            f.apply_permutation(&permutation);
        }
    }
}

macro_rules! display_val {
    ($ui:expr, $label:expr, $val:expr) => {
        $ui.horizontal(|ui| {
            ui.label($label);
            ui.add(egui::DragValue::new(&mut $val).min_decimals(3).speed(0.1));
        }).response
    };
}

impl NonpressureForceUi for ArtificialViscosity {
    fn ui(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Alpha: ")
                .on_hover_text("The coefficient of the linear part of the viscosity.");
            ui.add(egui::DragValue::new(&mut self.alpha).min_decimals(3).speed(0.1));
        });
        ui.horizontal(|ui| {
            ui.label("Beta: ")
                .on_hover_text("The coefficient of the quadratic part of the viscosity.");
            ui.add(egui::DragValue::new(&mut self.beta).min_decimals(3).speed(0.1));
        });
        ui.horizontal(|ui| {
            ui.label("Speed of sound: ")
                .on_hover_text("The speed of sound within this fluid.");
            ui.add(egui::DragValue::new(&mut self.speed_of_sound).min_decimals(3).speed(0.1));
        });
        display_val!(ui, "Fluid Viscosity Coefficient: ", self.fluid_viscosity_coefficient);
        display_val!(ui, "Boundary Viscosity Coefficient: ", self.boundary_viscosity_coefficient);
    }

    fn force_name(&self) -> &'static str { "Artificial Viscosity" }
}

/// Interface for editing [`Akinci2013SurfaceTension`]
pub struct Akinci2013SurfaceTensionInterface {
    force: Akinci2013SurfaceTension,
    fluid_tension_coefficient: Real,
    boundary_adhesion_coefficient: Real,
}

impl Akinci2013SurfaceTensionInterface {
    pub fn new(fluid_tension_coefficient: Real, boundary_adhesion_coefficient: Real) -> Self {
        Self {
            force: Akinci2013SurfaceTension::new(fluid_tension_coefficient, boundary_adhesion_coefficient),
            fluid_tension_coefficient,
            boundary_adhesion_coefficient,
        }
    }
}

impl NonPressureForce for Akinci2013SurfaceTensionInterface {
    fn solve(
        &mut self,
        timestep: &TimestepManager,
        kernel_radius: bevy_salva3d::salva::math::Real,
        fluid_fluid_contacts: &ParticlesContacts,
        fluid_boundaries_contacts: &ParticlesContacts,
        fluid: &mut Fluid,
        boundaries: &[Boundary],
        densities: &[bevy_salva3d::salva::math::Real]
    ) {
        self.force.solve(
            timestep,
            kernel_radius,
            fluid_fluid_contacts,
            fluid_boundaries_contacts,
            fluid,
            boundaries,
            densities
        );
    }

    fn apply_permutation(&mut self, permutation: &[usize]) {
        self.force.apply_permutation(permutation);
    }
}

impl NonpressureForceUi for Akinci2013SurfaceTensionInterface {
    fn ui(&mut self, ui: &mut Ui) {
        let fluid = display_val!(ui, "Fluid  Tension Coefficient: ", self.fluid_tension_coefficient);
        let boundary = display_val!(ui, "Boundary Adhesion Coefficient: ", self.boundary_adhesion_coefficient);

        if fluid.changed() || boundary.changed() {
            self.force = Akinci2013SurfaceTension::new(self.fluid_tension_coefficient, self.boundary_adhesion_coefficient);
        }
    }

    fn force_name(&self) -> &'static str { "Akinci Surface Tension" }
}

pub struct Becker2009ElasticityInterface {
    force: Becker2009Elasticity,
    young_modulus: Real,
    poisson_ratio: Real,
    nonlinear_strain: bool,
}

impl Becker2009ElasticityInterface {
    pub fn new(young_modulus: Real, poisson_ratio: Real, nonlinear_strain: bool) -> Self {
        Self {
            force: Becker2009Elasticity::new(young_modulus, poisson_ratio, nonlinear_strain),
            young_modulus,
            poisson_ratio,
            nonlinear_strain,
        }
    }
}

impl NonPressureForce for Becker2009ElasticityInterface {
    fn solve(
        &mut self,
        timestep: &TimestepManager,
        kernel_radius: bevy_salva3d::salva::math::Real,
        fluid_fluid_contacts: &ParticlesContacts,
        fluid_boundaries_contacts: &ParticlesContacts,
        fluid: &mut Fluid,
        boundaries: &[Boundary],
        densities: &[bevy_salva3d::salva::math::Real]
    ) {
        self.force.solve(
            timestep,
            kernel_radius,
            fluid_fluid_contacts,
            fluid_boundaries_contacts,
            fluid,
            boundaries,
            densities
        );
    }

    fn apply_permutation(&mut self, permutation: &[usize]) {
        self.force.apply_permutation(permutation);
    }
}

impl NonpressureForceUi for Becker2009ElasticityInterface {
    fn ui(&mut self, ui: &mut Ui) {
        let young_modulus = display_val!(ui, "Young Modulus: ", self.young_modulus);
        let poisson_ratio = display_val!(ui, "Poisson Ratio: ", self.poisson_ratio);
        let nonlinear_strain = display_val!(ui, "Poisson Ratio: ", self.poisson_ratio);
        let changed = young_modulus.changed() || poisson_ratio.changed() || nonlinear_strain.changed();
        if changed {
            self.force = Becker2009Elasticity::new(self.young_modulus, self.poisson_ratio, self.nonlinear_strain);
        }
    }

    fn force_name(&self) -> &'static str { "Becker Elasticity" }
}

/// A NonpressureForce that has a Ui for editing the underlying force
pub trait NonpressureForceUi: NonPressureForce {
    fn ui(&mut self, ui: &mut Ui);

    fn force_name(&self) -> &'static str;
}
