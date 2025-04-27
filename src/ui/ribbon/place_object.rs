use crate::ui::generic_object::GenericObject;
use crate::ui::{FunctionalUiResources, View};
use bevy::math::Vec3;
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::geometry::Collider;
use crate::ui::selecting::PickingExt;

#[derive(Default)]
pub struct PlaceObjectUi {
    cuboid: bool,
    ball: bool,
    cone: bool,
    cylinder: bool,
}

impl View for PlaceObjectUi {
    fn ui(&mut self, ui: &mut Ui, _commands: &mut Commands) {
        ui.set_min_width(100.);
        ui.horizontal(|ui| {
            self.cuboid = ui.button("Cuboid").clicked();
            self.ball = ui.button("Ball").clicked();
        });
        ui.horizontal(|ui| {
            self.cone = ui.button("Cone").clicked();
            self.cylinder = ui.button("Cylinder").clicked();
        });
    }

    fn view_name(&self) -> &'static str { "Place Object" }

    fn functionality(&mut self, res: &mut FunctionalUiResources) -> crate::prelude::Result<()> {
        let FunctionalUiResources {
            commands,
            robot_materials,
            meshes,
            ..
        } = res;
        if self.cuboid {
            commands
                .spawn((
                    Name::new("Cuboid"),
                    RigidBody::Dynamic,
                    GlobalTransform::default(),
                    Collider::cuboid(0.5, 0.5, 0.5),
                    Mesh3d(meshes.add(Cuboid::new(1., 1., 1.))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                ))
                .make_entity_pickable();
        }
        if self.ball {
            commands
                .spawn((
                    Name::new("Ball"),
                    RigidBody::Dynamic,
                    GlobalTransform::default(),
                    Collider::ball(0.5),
                    Mesh3d(meshes.add(Sphere::new(0.5))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                ))
                .make_entity_pickable();
        }
        if self.cone {

        }
        if self.cylinder {

        }

        Ok(())
    }
}
