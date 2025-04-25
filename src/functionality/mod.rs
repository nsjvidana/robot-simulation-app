use bevy::app::App;
use bevy::prelude::Plugin;

pub mod simulation;
pub mod import;
pub mod robot;
pub mod motion_planning;

pub struct RobotLabPlugin;

impl Plugin for RobotLabPlugin {
    fn build(&self, app: &mut App) {
        simulation::build_plugin(app);
        import::build_plugin(app);
        robot::build_plugin(app);
        motion_planning::build_app(app);
    }
}
