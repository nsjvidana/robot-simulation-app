use bevy::app::App;
use bevy::prelude::{Component, Entity, GlobalTransform, Plugin, Resource, Transform};
use bevy_rapier3d::rapier::data::{Arena, Index};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobot};
use serde::{Deserialize, Serialize};

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
    pub transform: GlobalTransform
}

#[derive(Component)]
pub struct Robot {
    pub(crate) rapier_urdf_robot: Option<UrdfRobot>,
    pub(crate) robot_joint_type: RobotJointType,
}

impl Robot {
    pub fn new(robot: UrdfRobot) -> Self {
        Self {
            rapier_urdf_robot: Some(robot),
            robot_joint_type: RobotJointType::ImpulseJoints,
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
