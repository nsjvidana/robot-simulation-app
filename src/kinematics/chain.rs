use crate::kinematics::node::{KJointType, KNode};
use crate::math::Real;

#[derive(Default)]
pub struct KChain {
    nodes: Vec<KNode>,
    movable_nodes: Vec<KNode>,
}

impl KChain {
    pub fn from_root(root: &KNode) -> Self {
        let mut nodes = Vec::new();

        nodes.push(root.clone());
        for child in root.iter_children() {
            nodes.push(child);
        }

        Self::from_nodes(nodes)
    }

    pub fn from_nodes(nodes: Vec<KNode>) -> Self {
        let movable_nodes: Vec<KNode> = nodes
            .iter()
            .filter(|node| !matches!(node.joint().joint_type(), KJointType::Fixed))
            .cloned()
            .collect();
        Self {
            nodes,
            movable_nodes
        }
    }

}

pub struct SerialKChain {
    chain: KChain,
}

impl SerialKChain {
    pub fn num_movable_nodes(&self) -> usize {
        self.chain.movable_nodes.len()
    }

    pub fn iter(&self) -> Iter<KNode> {
        self.nodes.iter()
    }

    pub fn end(&self) -> Option<&KNode> {
        self.nodes.last()
    }

    pub fn root(&self) -> Option<&KNode> {
        self.nodes.first()
    }

    pub fn get_node(&self, idx: usize) -> Option<&KNode> {
        self.nodes.get(idx)
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn set_joint_positions(&self, positions: &[Real]) -> Result<(), KError> {
        if positions.len() != self.num_movable_nodes() {
            return Err(KError::SizeMismatchError { input_size: positions.len(), required_size: self.num_movable_nodes() })
        }

        for (node, pos) in self.nodes.iter().zip(positions.iter()) {
            node.joint().set_position(*pos)?;
        }
        Ok(())
    }

    pub fn set_joint_positions_deg(&self, positions: &[Real]) -> Result<(), KError> {
        if positions.len() != self.num_movable_nodes() {
            return Err(KError::SizeMismatchError { input_size: positions.len(), required_size: self.num_movable_nodes() })
        }

        for (node, pos) in self.nodes.iter().zip(positions.iter()) {
            node.joint().set_position(pos.to_radians())?;
        }
        Ok(())
    }

    pub fn update_world_transforms(&mut self) {
        for (i, node) in self.iter().enumerate() {
            let mut curr_joint = node.joint();

            let mut transform = Isometry3::identity();
            for joint in self.iter_joints().take(i) {
                transform *= joint.local_transform();
            }
            transform *= curr_joint.local_transform();

            curr_joint.world_transform_cache = Some(transform);
        }
    }
}
