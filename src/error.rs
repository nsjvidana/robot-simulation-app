use bevy::prelude::{Event, EventWriter, In};
use crate::prelude::*;

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
    #[error("Operation Failed: {0}")]
    FailedOperation(String),
    #[error(transparent)]
    IO(#[from] std::io::Error),
}

#[derive(Event)]
pub struct ErrorEvent {
    pub error: Error,
    pub location: Option<String>,
}

pub fn error_handling_system(
    In(result): In<Result<()>>,
    mut events: EventWriter<ErrorEvent>,
) {
    if let Err(error) = result {
        events.send(ErrorEvent { error, location: None });
    }
}
