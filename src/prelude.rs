pub use crate::error::Error;

pub use crate::convert::*;
pub use crate::kinematics::*;
pub use crate::math::Real;
pub use crate::robot::systems::*;
pub use crate::robot::*;
pub use crate::ui;
pub use ui::RobotLabUiPlugin;

pub type Result<T> = core::result::Result<T, Error>;

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
