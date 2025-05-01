use crate::error::{error_handling_system, Error};
use crate::functionality::robot::{create_rapier_robot, RobotEntity};
use crate::prelude::*;
use bevy::prelude::*;
use bevy_rapier3d::geometry::VHACDParameters;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions};
use std::fmt;
use std::path::{Path, PathBuf};

#[derive(Event)]
pub struct ImportEvent {
    pub urdf_path: PathBuf,
    pub mesh_dir: PathBuf,
    pub urdf_loader_options: UrdfLoaderOptions,
    pub robot_joint_type: RobotJointType,
    pub approximate_collisions: bool,
    pub convex_decomp_options: VHACDParameters,
    pub all_joints_have_motors: bool,
    pub sampling_meth
}

#[derive(PartialEq, Debug, Copy, Clone)]
pub enum RobotJointType {
    Multibody(UrdfMultibodyOptions),
    Impulse,
}

impl Default for RobotJointType {
    fn default() -> Self {
        Self::Multibody(UrdfMultibodyOptions::default())
    }
}

impl fmt::Display for RobotJointType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            RobotJointType::Multibody(_) => {"Multibody"}
            RobotJointType::Impulse => {"Impulse"}
        };
        write!(f, "{name}")
    }
}

pub fn build_plugin(app: &mut App) {
    app.add_event::<ImportEvent>();
    app.add_systems(
        PostUpdate,
        import_robots.pipe(error_handling_system)
    );
}

pub fn import_robots(
    mut events: ResMut<Events<ImportEvent>>,
    mut commands: Commands,
) -> Result<()> {
    for event in events.drain() {
        let urdf_path = event.urdf_path.as_path();
        let urdf = urdf_rs::read_file(urdf_path)
            .map_err(|_| Error::ImportError(
                format!("Failed loading URDF file \"{}\"", urdf_path.display())
            ))?;

        let mesh_dir =
            if event.mesh_dir.display().to_string().trim().is_empty() {
                event.urdf_path.parent()
                    .unwrap_or_else(|| Path::new("./"))
            }
            else if !event.mesh_dir.is_dir() {
                return Err(Error::ImportError(
                    format!("Mesh Directory {} is not a directory!", event.mesh_dir.display())
                ))
            }
            else {
                event.mesh_dir.as_path()
            };
        let rapier_robot = create_rapier_robot(
            urdf.clone(),
            event.urdf_loader_options.clone(),
            mesh_dir,
            event.approximate_collisions,
            event.convex_decomp_options,
            event.all_joints_have_motors,
        )?;

        commands.spawn(RobotEntity::new(
            urdf,
            event.urdf_path.clone(),
            mesh_dir.to_path_buf(),
            event.urdf_loader_options,
            event.robot_joint_type,
            rapier_robot,
        ));
    }
    Ok(())
}
