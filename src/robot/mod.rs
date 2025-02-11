use bevy::app::App;
use bevy::prelude::{Component, Entity, Plugin, Resource, Transform};
use bevy_rapier3d::rapier::data::{Arena, Index};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobot};

pub mod systems;

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RobotSet>();
    }
}

#[derive(Resource, Default)]
pub struct RobotSet {
    pub robots: Arena<RobotEntities>,
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

pub struct RobotLink {
    rigid_body: Entity,
    colliders: Vec<Entity>,
    initial_pos: Transform,
}
