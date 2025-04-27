use crate::prelude::Real;
use crate::ui::View;
use bevy::prelude::Commands;
use bevy_egui::egui::Ui;
use bevy_rapier3d::prelude::{Collider, Sensor};
use bevy_salva3d::fluid::FluidParticlePositions;
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use bevy_salva3d::utils::cube_particle_positions;
use crate::ui::selecting::PickingExt;

#[derive(Default)]
pub struct FluidsUi {

}

impl View for FluidsUi {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        if ui.button("New Fluid").clicked() {
            let r = SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS;
            let half_size = ((1. / r)/2.) as usize;
            commands.spawn((
                FluidParticlePositions {
                    positions: cube_particle_positions(half_size, half_size, half_size, r),
                },
                Collider::cuboid(
                    half_size as Real * r, 
                    half_size as Real * r, 
                    half_size as Real * r
                ),
                Sensor,
            ))
                .make_entity_pickable();
        }
    }

    fn view_name(&self) -> &'static str {
        "Fluids"
    }
}
