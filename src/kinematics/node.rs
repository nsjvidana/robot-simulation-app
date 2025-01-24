use std::{ops::{Deref, DerefMut}, sync::{Arc, Weak}};

use crate::error::KError;
use bevy_rapier3d::{math::Real, na::{Isometry3, Translation3, UnitQuaternion, UnitVector3}};
use parking_lot::{Mutex, MutexGuard};

#[derive(Default, Clone)]
pub struct KNodeData {
    pub parent: Option<Weak<Mutex<KNodeData>>>,
    pub child: Option<KNode>,
    pub joint: KJoint,
}

#[derive(Clone)]
pub struct KNode(pub(crate) Arc<Mutex<KNodeData>>);

impl KNode {
    pub fn set_parent(&self, parent: &KNode) {
        self.0.lock().parent = Some(Arc::downgrade(&parent.0));
        parent.0.lock().child = Some(self.clone());
    }

    pub fn iter_children(&self) -> KNodeChildren {
        KNodeChildren::new(self.clone())
    }

    pub fn joint(&self) -> KJointRef {
        KJointRef {
            guard: self.0.lock()
        }
    }

    pub fn joint_position(&self) -> Real {
        self.joint().position
    }
}

pub struct KJointRef<'a> {
    guard: MutexGuard<'a, KNodeData>
}

impl<'a> Deref for KJointRef<'a> {
    type Target = KJoint;
    fn deref(&self) -> &Self::Target {
        &self.guard.joint
    }
}

impl<'a> DerefMut for KJointRef<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.guard.joint
    }
}

#[derive(Default)]
#[allow(unused)]
pub struct KNodeBuilder(pub(crate) KNodeData);

impl KNodeBuilder {
    pub fn new() -> Self {
        KNodeBuilder::default()
    }

    pub fn joint_type(mut self, joint_type: KJointType) -> Self {
        self.0.joint.joint_type = joint_type;
        self
    }

    pub fn origin(mut self, origin: Isometry3<Real>) -> Self {
        self.0.joint.origin = origin;
        self
    }

    pub fn translation(mut self, translation: Translation3<Real>) -> Self {
        self.0.joint.origin.translation = translation;
        self
    }

    pub fn rotation(mut self, rotation: UnitQuaternion<Real>) -> Self {
        self.0.joint.origin.rotation = rotation;
        self
    }

    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.joint.limits = limits;
        self
    }

    pub fn limits_deg(mut self, limits: [Real; 2]) -> Self {
        self.0.joint.limits = [
            limits[0].to_radians(),
            limits[1].to_radians(),
        ];
        self
    }

    pub fn name(mut self, name: String) -> Self {
        self.0.joint.name = name;
        self
    }

    pub fn build(self) -> KNode {
        KNode(Arc::new(Mutex::new(self.0)))
    }
}

#[derive(Clone)]
pub struct KJoint {
    pub name: String,
    joint_type: KJointType,
    /// The origin local transform of the joint.
    origin: Isometry3<Real>,
    position: Real,
    pub limits: [Real; 2],
    pub(crate) world_transform_cache: Option<Isometry3<Real>>,
}

impl KJoint {
    pub fn position(&self) -> Real {
        self.position
    }

    pub fn set_position(&mut self, pos: Real) -> Result<&mut Self, KError> {
        match self.joint_type {
            KJointType::Fixed => {
                return Err(KError::SettingFixedJointPos { joint_name: self.name.clone() });
            },
            _ => {
                if pos < self.limits[0] || pos > self.limits[1] {
                    return Err(KError::OutOfLimits { joint_name: self.name.clone(), position: pos, min_limit: self.limits[0], max_limit: self.limits[1] })
                }
                self.set_position_unchecked(pos);
            }
        }

        Ok(self)
    }

    pub fn set_position_clamped(&mut self, pos: Real) {
        if pos < self.limits[0] {
            self.position = self.limits[0];
        }
        else if pos > self.limits[1] {
            self.position = self.limits[1];
        }
        else {
            self.position = pos;
        }
        self.clear_cache();
    }

    pub fn set_position_unchecked(&mut self, pos: Real) -> &mut Self {
        self.position = pos;
        self.clear_cache();
        self
    }

    pub fn increment_position(&mut self, increment: Real) -> &mut Self {
        self.set_position_clamped(self.position + increment);
        self
    }

    #[inline]
    pub fn set_origin(&mut self, origin: Isometry3<Real>) -> &mut Self {
        self.origin = origin;
        self.clear_cache();
        self
    }

    #[inline]
    pub fn origin(&self) -> &Isometry3<Real> {
        &self.origin
    }

    pub fn new(joint_type: KJointType) -> Self {
        Self {
            joint_type,
            ..Default::default()
        }
    }

    #[inline]
    pub fn clear_cache(&mut self) {
        self.world_transform_cache = None;
    }

    pub fn joint_type(&self) -> &KJointType {
        &self.joint_type
    }

    pub fn local_transform(&self) -> Isometry3<Real> {
        match self.joint_type {
            KJointType::Fixed =>
                self.origin,
            KJointType::Linear { axis } =>
                self.origin * Translation3::from(axis.into_inner() * self.position),
            KJointType::Revolute { axis } =>
                self.origin * UnitQuaternion::from_axis_angle(&axis, self.position)
        }
    }

    pub fn world_transform(&self) -> Option<Isometry3<Real>> {
        self.world_transform_cache
    }
}

impl Default for KJoint {
    fn default() -> Self {
        Self {
            name: String::new(),
            joint_type: KJointType::default(),
            origin: Isometry3::identity(),
            position: 0.0,
            limits: [f32::NEG_INFINITY, f32::INFINITY],
            world_transform_cache: None,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub enum KJointType {
    #[default]
    Fixed,
    Revolute {
        axis: UnitVector3<Real>
    },
    Linear {
        axis: UnitVector3<Real>
    },
}

pub struct KNodeChildren {
    curr_node: KNode
}

impl KNodeChildren {
    pub fn new(node: KNode) -> Self {
        Self {
            curr_node: node
        }
    }
}

impl Iterator for KNodeChildren {
    type Item = KNode;

    fn next(&mut self) -> Option<Self::Item> {
        let child_opt = self.curr_node.0.lock().child.clone();
        match child_opt {
            Some(child) => {
                self.curr_node = child.clone();
                Some(self.curr_node.clone())
            },
            None => None
        }
    }
}

