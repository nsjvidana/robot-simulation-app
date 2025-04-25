use crate::functionality::robot::{RobotEntity, RobotPart};
use crate::ui::selecting::{MakeSelectionsSet, PickingExt, PickingRequest, PickingRequestCommandExt, PickingResponse, SelectingSet};
use crate::ui::toolbar::ToolbarWindow;
use bevy::prelude::*;
use crate::ui::properties::{EntityProperties, PropertiesUi};

pub fn build_app(app: &mut App) {
    let robot_selection = RobotSelection::from_world(app.world_mut());
    app.insert_resource(robot_selection);

    app.add_systems(
        PreUpdate,
        (
            add_picking_to_robot_parts,
        )
            .chain()
            .in_set(SelectingSet)
    );

    app.add_systems(
        Update,
        select_robots
            .in_set(MakeSelectionsSet)
    );
}

#[derive(Resource, Debug)]
pub struct RobotSelection {
    pub active_robot: Option<Entity>,
    pub selected_robots: Vec<Entity>,

    robot_picking_resp: PickingResponse,
}

impl FromWorld for RobotSelection {
    fn from_world(world: &mut World) -> Self {
        let request =
            PickingRequest::new(|resources, event| {
                resources.robot_parts.contains(event.entity)
            })
                .continuous(true)
                .at_priority(0)
                .detect_unpicking(true);
        let resp = world.send_picking_request(request);

        Self {
            active_robot: None,
            selected_robots: Default::default(),
            robot_picking_resp: resp,
        }
    }
}

pub fn select_robots(
    mut selections: ResMut<RobotSelection>,
    robot_parts: Query<&RobotPart>,
    robots: Query<(&GlobalTransform, &RobotEntity)>,
    mut toolbar_window: ResMut<ToolbarWindow>,
    mut properties_ui: ResMut<PropertiesUi>,
) {
    if let Some(resp) = selections.robot_picking_resp.take_response() {
        if let Ok(robot_e) = robot_parts
            .get(resp.event.entity)
            .map(|v| v.0)
        {
            selections.active_robot = Some(robot_e);
            toolbar_window.active_movable_entity = Some(robot_e);
            let (trans, robot) = robots.get(robot_e).unwrap();
            properties_ui.selected_entity = Some(EntityProperties::Robot {
                entity: robot_e,
                name: robot.urdf.name.clone(),
                transform: Transform {
                    translation: trans.translation(),
                    rotation: trans.rotation(),
                    scale: trans.scale(),
                },
            });
            if !resp.shift_click {
                selections.selected_robots.clear();
            }
            selections.selected_robots.push(robot_e);
        }
    }

    if selections.robot_picking_resp.unpicked() {
        selections.robot_picking_resp.reset_unpicked();
        selections.active_robot = None;
        toolbar_window.active_movable_entity = None;
        selections.selected_robots.clear();
    }
}

pub fn add_picking_to_robot_parts(
    new_robot_parts: Query<Entity, Added<RobotPart>>,
    mut commands: Commands,
) {
    for entity in new_robot_parts.iter() {
        commands.entity(entity)
            .make_entity_pickable();
    }
}