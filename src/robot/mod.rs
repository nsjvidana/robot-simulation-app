use crate::prelude::urdf_rs_robot_to_xurdf;
use bevy::app::App;
use bevy::prelude::{Component, Entity, GlobalTransform, Plugin, Resource, Transform};
use bevy_rapier3d::rapier::data::{Arena, Index};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};
use serde::{Deserialize, Serialize};
use std::ops::{Deref, DerefMut};
use std::path::{Path, PathBuf};

pub mod systems;

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RobotSet>();
    }
}

#[derive(Resource, Default, Serialize, Deserialize)]
pub struct RobotSet {
    pub robots: Arena<RobotSerData>,
}

/// Data for serializing the state of the robot sim
#[derive(Serialize, Deserialize)]
pub struct RobotSerData {
    pub robot_entity: Entity,
    pub entities: RobotEntities,
    pub transform: GlobalTransform,
}

#[derive(Component)]
pub struct Robot {
    pub urdf: urdf_rs::Robot,
    pub mesh_dir: Option<PathBuf>,
    pub robot_joint_type: RobotJointType,
    pub robot_file_path: PathBuf,
}

impl Robot {
    pub fn new(robot: urdf_rs::Robot, path: PathBuf, mesh_dir: Option<PathBuf>) -> Self {
        Self {
            urdf: robot,
            mesh_dir,
            robot_joint_type: RobotJointType::ImpulseJoints,
            robot_file_path: path,
        }
    }

    pub fn with_impulse_joints(mut self) -> Self {
        self.robot_joint_type = RobotJointType::ImpulseJoints;
        self
    }

    pub fn with_multibody_joints(mut self, joint_options: UrdfMultibodyOptions) -> Self {
        self.robot_joint_type = RobotJointType::MultibodyJoints(joint_options);
        self
    }
}

#[derive(Component)]
pub struct RapierRobotHandles(pub(crate) UrdfRobotHandles<Option<Index>>);

impl Deref for RapierRobotHandles {
    type Target = UrdfRobotHandles<Option<Index>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for RapierRobotHandles {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub enum RobotJointType {
    ImpulseJoints,
    MultibodyJoints(UrdfMultibodyOptions),
}

impl Default for RobotJointType {
    fn default() -> Self {
        Self::ImpulseJoints
    }
}

#[derive(Serialize, Deserialize)]
pub struct RobotEntities {
    link_entities: Vec<RobotLink>,
}

#[derive(Component)]
pub struct RobotHandle(pub Index);

#[derive(Component)]
pub struct LinkEntity {
    pub colliders: Vec<Entity>,
}

#[derive(Component)]
pub struct RobotPart(pub Entity);

#[derive(Serialize, Deserialize)]
pub struct RobotLink {
    rigid_body: Entity,
    colliders: Vec<Entity>,
    initial_pos: Transform,
}
