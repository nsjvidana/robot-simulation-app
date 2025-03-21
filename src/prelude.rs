pub use crate::error::Error;

pub use crate::ui;
pub use ui::RobotLabUiPlugin;
pub use crate::robot::*;
pub use crate::robot::systems::*;
pub use crate::kinematics::*;
pub use crate::convert::*;
pub use crate::math::Real;

pub type Result<T> = core::result::Result<T, Error>;