use std::ops::{Deref, DerefMut};
use bevy_rapier3d::prelude::*;

pub use crate::error::Error;
pub use crate::kinematics::*;
pub use crate::math::Real;
pub use crate::robot::systems::*;
pub use crate::robot::*;
pub use crate::ui;
pub use ui::RobotLabUiPlugin;

pub type Result<T> = core::result::Result<T, Error>;

pub(crate) type RapierContextTuple = (
    RapierContextSimulation,
    RapierContextColliders,
    RapierContextJoints,
    RapierQueryPipeline,
    RapierRigidBodySet,
);

// Generic wrapper type
pub struct W<T>(pub T);

impl<T> Deref for W<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for W<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

macro_rules! send_err_events {
    ($error_events:expr, $code:block) => {
        {
            let result = || -> crate::Result<()> {$code} ();
            if let Err(e) = result {
                $error_events.send(crate::error::ErrorEvent {
                    error: e,
                    location: None,
                });
            }
        }
    };
}

pub(crate) use send_err_events;
