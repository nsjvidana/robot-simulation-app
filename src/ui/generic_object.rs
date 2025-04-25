use bevy::prelude::*;
use bevy_rapier3d::prelude::Collider;
use crate::ui::selecting::{MakeSelectionsSet, PickingRequest, PickingRequestCommandExt, PickingResponse, PickingServer};
use crate::ui::toolbar::ToolbarWindow;

pub fn build_app(app: &mut App) {
    app.init_resource::<GenericObjSelection>();

    app.add_systems(Update, select_generic_objects.in_set(MakeSelectionsSet));
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
    mut toolbar_window: ResMut<ToolbarWindow>
) {
    if let Some(entity) = selection.picking_resp
        .take_response()
        .map(|v| v.event.entity)
    {
        toolbar_window.active_movable_entity = Some(entity);
    }

    if selection.picking_resp.unpicked() {
        selection.picking_resp.reset_unpicked();
        toolbar_window.active_movable_entity = None;
    }
}
