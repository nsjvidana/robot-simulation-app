use bevy::ecs::query::QueryEntityError;
use bevy::prelude::Entity;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Generic: {0}")]
    Generic(String),
    #[error("Entity {entity_name} missing component {component_name}")]
    MissingComponent {
        entity_name: String,
        component_name: String,
    },
    #[error("URDF error with robot {robot_name}: {error}")]
    Urdf {
        error: String,
        robot_name: String
    },
    #[error(transparent)]
    IO(#[from] std::io::Error),
}
