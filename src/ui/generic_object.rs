use crate::box_vec;
use crate::ui::selecting::{MakeSelectionsSet, PickingRequest, PickingRequestCommandExt, PickingResponse};
use crate::ui::toolbar::{MovableEntityPickedEvent, ToolbarWindow};
use bevy::prelude::*;
use crate::ui::properties::PropertiesSelectionEvent;

pub fn build_app(app: &mut App) {
    app.init_resource::<GenericObjSelection>();

    app.add_systems(
        Update,
        select_generic_objects
            .in_set(MakeSelectionsSet)
    );
}

#[derive(Component)]
pub struct GenericObject;

#[derive(Resource)]
pub struct GenericObjSelection {
    picking_resp: PickingResponse,
}

impl FromWorld for GenericObjSelection {
    fn from_world(world: &mut World) -> Self {
        let request = PickingRequest::new(|res, event| {
            res.generic_objects.contains(event.entity)
        })
            .prevent_blocking(true)
            .continuous(true)
            .detect_unpicking(true)
            .at_priority(1); // Don't block the robot selection request.
        let resp = world.send_picking_request(request);
        GenericObjSelection { picking_resp: resp }
    }
}

pub fn select_generic_objects(
    selection: ResMut<GenericObjSelection>,
    mut toolbar_window: ResMut<ToolbarWindow>,
    obj_names: Query<&Name, With<GenericObject>>,
    mut commands: Commands,
) {
    if let Some(entity) = selection.picking_resp
        .take_response()
        .map(|v| v.event.entity)
    {
        let name = obj_names.get(entity).unwrap();
        commands.send_event(PropertiesSelectionEvent {
            entity,
            name: name.to_string(),
        });
        commands.send_event(MovableEntityPickedEvent(entity));
    }
}
