use crate::box_vec;
use crate::prelude::Real;
use crate::ui::properties::{EntityPropertiesExt, EntityProperty, PropertiesSelectionEvent};
use crate::ui::selecting::{MakeSelectionsSet, PickingExt, PickingRequest, PickingRequestCommandExt, PickingResponse};
use crate::ui::View;
use bevy::math::bounding::BoundingVolume;
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use bevy_rapier3d::prelude::{Collider, Sensor};
use bevy_salva3d::fluid::FluidPositions;
use bevy_salva3d::plugin::SalvaPhysicsPlugin;
use bevy_salva3d::utils::cube_particle_positions;
pub fn build_app(app: &mut App) {
    app.init_resource::<FluidSelection>();

    app.add_systems(Update, select_fluids.in_set(MakeSelectionsSet));
}

#[derive(Default)]
pub struct FluidsUi;

impl View for FluidsUi {
    fn ui(&mut self, ui: &mut Ui, commands: &mut Commands) {
        if ui.button("New Fluid").clicked() {
            let r = SalvaPhysicsPlugin::DEFAULT_PARTICLE_RADIUS;
            let half_size = ((1. / r)/2.) as usize;
            commands.spawn((
                Transform::default(),
                FluidPositions(cube_particle_positions(half_size, half_size, half_size, r)),
                Collider::cuboid(
                    half_size as Real * r, 
                    half_size as Real * r, 
                    half_size as Real * r
                ),
                Sensor,
            ))
                .insert_entity_properties(box_vec![FluidProperty::new()])
                .make_entity_pickable();
        }
    }

    fn view_name(&self) -> &'static str { "Fluids" }
}

#[derive(Resource)]
pub struct FluidSelection {
    picking_resp: PickingResponse,
}

impl FromWorld for FluidSelection {
    fn from_world(world: &mut World) -> Self {
        let request = PickingRequest::new(|res, event| {
            res.fluids.contains(event.entity)
        })
            .prevent_blocking(true)
            .continuous(true)
            .detect_unpicking(true)
            .at_priority(2); // Don't block other requests.
        let resp = world.send_picking_request(request);
        FluidSelection {
            picking_resp: resp,
        }
    }
}

pub fn select_fluids(
    mut selection: ResMut<FluidSelection>,
    mut fluid_positions: Query<(Option<&Name>, &FluidPositions)>,
    mut commands: Commands
) {
    if let Some(fluid_entity) = selection.picking_resp
        .take_response()
        .map(|v| v.event.entity)
    {
        let (name, _) = fluid_positions.get(fluid_entity).unwrap();
        commands.send_event(PropertiesSelectionEvent {
            entity: fluid_entity,
            name: name
                .map(|v| v.to_string())
                .unwrap_or_else(|| "Fluid".to_string()),
        });
    }
}
