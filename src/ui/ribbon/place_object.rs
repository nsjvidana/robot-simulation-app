use crate::ui::generic_object::GenericObject;
use crate::ui::selecting::PickingExt;
use crate::ui::{FunctionalUiResources, View, VisualEntity};
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::geometry::Collider;
use bevy_salva3d::rapier_integration::{ColliderSamplingMethod, RapierColliderSampling};
use crate::box_vec;
use crate::functionality::simulation::SimulationState;
use crate::ui::properties::{EntityPropertiesExt, TransformProperty};

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
            sim_state,
            ..
        } = res;
        if **sim_state != SimulationState::Reset {
            return Ok(());
        }

        if self.cuboid {
            commands
                .spawn((
                    Name::new("Cuboid"),
                    RigidBody::Fixed,
                    Collider::cuboid(0.5, 0.5, 0.5),
                    InheritedVisibility::default(),
                    VisualEntity,
                    Mesh3d(meshes.add(Cuboid::new(1., 1., 1.))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                    RapierColliderSampling {
                        sampling_method: ColliderSamplingMethod::DynamicContact
                    }
                ))
                .insert_entity_properties(box_vec![TransformProperty::new()])
                .make_entity_pickable();
        }
        if self.ball {
            commands
                .spawn((
                    Name::new("Ball"),
                    RigidBody::Dynamic,
                    Collider::ball(0.5),
                    InheritedVisibility::default(),
                    VisualEntity,
                    Mesh3d(meshes.add(Sphere::new(0.5))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                ))
                .insert_entity_properties(box_vec![TransformProperty::new()])
                .make_entity_pickable();
        }
        if self.cone {
            commands
                .spawn((
                    Name::new("Cone"),
                    RigidBody::Dynamic,
                    Collider::cone(0.5, 0.5),
                    InheritedVisibility::default(),
                    VisualEntity,
                    Mesh3d(meshes.add(Cone::new(0.5, 1.))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                ))
                .insert_entity_properties(box_vec![TransformProperty::new()])
                .make_entity_pickable();
        }
        if self.cylinder {
            commands
                .spawn((
                    Name::new("Cylinder"),
                    RigidBody::Dynamic,
                    Collider::cylinder(0.5, 0.5),
                    InheritedVisibility::default(),
                    VisualEntity,
                    Mesh3d(meshes.add(Cylinder::new(0.5, 1.))),
                    MeshMaterial3d(robot_materials.white_mat.clone()),
                    GenericObject,
                ))
                .insert_entity_properties(box_vec![TransformProperty::new()])
                .make_entity_pickable();
        }

        Ok(())
    }
}
