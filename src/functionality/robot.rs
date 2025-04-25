use std::collections::HashMap;
use std::ops::{Deref, DerefMut, Mul};
use std::path::{Path, PathBuf};
use bevy::asset::RenderAssetUsages;
use bevy::ecs::query::QueryData;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy_rapier3d::dynamics::{JointAxesMask, JointAxis};
use bevy_rapier3d::geometry::VHACDParameters;
use bevy_rapier3d::na;
use bevy_rapier3d::na::RealField;
use bevy_rapier3d::parry::mass_properties::MassProperties;
use bevy_rapier3d::parry::math::{Isometry, Point, Vector};
use bevy_rapier3d::parry::shape::SharedShape;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::prelude::systems::apply_initial_rigid_body_impulses;
use bevy_rapier3d::rapier;
use bevy_rapier3d::rapier::data::{Arena, Index};
use bevy_rapier3d::rapier::prelude::SPATIAL_DIM;
use bevy_rapier3d::utils::iso_to_transform;
use rapier3d_urdf::{UrdfJoint, UrdfJointHandle, UrdfLink, UrdfLoaderOptions, UrdfRobot, UrdfRobotHandles};
use serde::{Deserialize, Serialize};
use urdf_rs::{Geometry, Inertial, Joint, Pose};
use crate::error::{error_handling_system, Error};
use crate::functionality::import::RobotJointType;
use crate::prelude::*;
use crate::prelude::Real;

pub fn build_plugin(app: &mut App) {
    app
        .init_resource::<RobotSet>()
        .init_resource::<RobotMaterials>();

    app.add_systems(
        FixedUpdate,
        (
            init_robots.pipe(error_handling_system)
                .in_set(PhysicsSet::SyncBackend)
                .after(apply_initial_rigid_body_impulses),
            sync_robots
                .before(PhysicsSet::SyncBackend),
            delete_robots
                .before(PhysicsSet::SyncBackend),
        )
    );
}

#[derive(Resource)]
pub struct RobotMaterials {
    pub white_mat: Handle<StandardMaterial>,
}

impl FromWorld for RobotMaterials {
    fn from_world(world: &mut World) -> Self {
        let white_mat = world.resource_mut::<Assets<StandardMaterial>>()
            .add(Color::WHITE);
        Self {
            white_mat,
        }
    }
}

#[derive(Resource, Default, Serialize, Deserialize)]
pub struct RobotSet {
    pub robots: Arena<RobotSerdeData>
}

#[derive(Serialize, Deserialize)]
pub struct RobotSerdeData {
    pub robot_entity: Entity,
    pub links: Vec<RobotLink>,
}

#[derive(Component)]
pub struct RobotSetHandle(pub Index);

#[derive(QueryData)]
pub struct RobotQueryData {
    pub robot: &'static RobotEntity,
    pub robot_handle: &'static RobotSetHandle,
    pub rapier_handles: &'static RapierRobotHandles,
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct RobotQueryDataMut {
    pub robot: &'static mut RobotEntity,
    pub robot_handle: &'static mut RobotSetHandle,
    pub rapier_handles: &'static mut RapierRobotHandles,
    pub transform: &'static mut GlobalTransform,
}

#[derive(Component)]
pub struct RobotEntity {
    pub urdf: urdf_rs::Robot,
    pub chain: k::Chain<Real>,
    pub urdf_path: PathBuf,
    pub mesh_dir: PathBuf,
    pub loader_options: UrdfLoaderOptions,
    robot_joint_type: RobotJointType,
    rapier_robot: Option<UrdfRobot>,
}

impl RobotEntity {
    pub fn new(
        urdf: urdf_rs::Robot,
        urdf_path: PathBuf,
        mesh_dir: PathBuf,
        loader_options: UrdfLoaderOptions,
        robot_joint_type: RobotJointType,
        rapier_robot: UrdfRobot,
    ) -> Self {
        Self {
            chain: k::Chain::from(&urdf),
            urdf,
            urdf_path,
            mesh_dir,
            loader_options,
            robot_joint_type,
            rapier_robot: Some(rapier_robot),
        }
    }

    pub fn robot_joint_type(&self) -> &RobotJointType {
        &self.robot_joint_type
    }
}

#[derive(Component, Clone)]
pub struct RobotLinks(pub Vec<RobotLink>);

impl Deref for RobotLinks {
    type Target = Vec<RobotLink>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl DerefMut for RobotLinks {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

#[derive(Component)]
pub struct RapierRobotHandles(pub UrdfRobotHandles<Index>);

#[derive(Component)]
pub struct RobotPart(pub Entity);

#[derive(Serialize, Deserialize, Clone)]
pub struct RobotLink {
    pub rigid_body: Entity,
    pub colliders: Vec<Entity>,
    pub visuals: Vec<Entity>,
    pub initial_pos: Transform,
}

#[derive(Component)]
pub struct KinematicNode(pub k::Node<Real>);

impl Deref for KinematicNode {
    type Target = k::Node<Real>;
    fn deref(&self) -> &Self::Target { &self.0 }
}

pub fn init_robots(
    mut new_robots: Query<(Entity, &mut RobotEntity, Option<&GlobalTransform>, Option<&RapierContextEntityLink>), Added<RobotEntity>>,
    mut robot_set: ResMut<RobotSet>,
    mut context_q: WriteRapierContext<()>,
    default_context_q: Query<Entity, With<DefaultRapierContext>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_materials: Res<RobotMaterials>,
    mut commands: Commands,
) -> Result<()> {
    for (entity, mut robot, transform, ctx_link) in new_robots.iter_mut() {
        let Some(rapier_robot) = robot.rapier_robot.take() else {
            continue;
        };
        let context_link = RapierContextEntityLink(
            ctx_link.map_or_else(|| default_context_q.single(), |link| link.0),
        );
        let (
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        ) = context_q.rapier_context.get_mut(context_link.0)
            .expect("Invalid rapier context entity!");
        let mut rapier_context = RapierContextMut {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        };

        let handles = get_robot_handles(
            rapier_robot,
            robot.robot_joint_type.clone(),
            &mut rapier_context
        )?;
        
        let links = create_rapier_robot_entities(
            &robot,
            &handles,
            &mut rapier_context,
            &context_link,
            &robot.robot_joint_type,
            entity,
            &mut *meshes,
            &mut *materials,
            &robot_materials.white_mat,
            &mut commands
        );

        // Setting KinematicNode components
        for node in robot.chain.iter() {
            let link = node.link();
            let name = &link.as_ref().unwrap().name;
            let link_idx = robot.urdf.links
                .iter()
                .position(|v| v.name == *name)
                .unwrap();
            commands.entity(links[link_idx].rigid_body)
                .insert(KinematicNode(node.clone()));
        }

        let transform = transform
            .copied()
            .unwrap_or_else(GlobalTransform::default);

        let robot_set_idx = robot_set.robots.insert(RobotSerdeData {
            robot_entity: entity,
            links: links.clone(),
        });
        commands.entity(entity)
            .insert((
                RapierRobotHandles(handles),
                RobotLinks(links),
                RobotSetHandle(robot_set_idx),
                transform,
            ));
    }

    Ok(())
}

pub fn sync_robots(
    moved_robots: Query<
        (&mut RobotEntity, &RobotSetHandle, &GlobalTransform),
        (Changed<GlobalTransform>, With<RobotEntity>)
    >,
    mut part_transform_q: Query<
        &mut GlobalTransform,
        (With<RobotPart>, Without<RobotEntity>),
    >,
    mut robot_set: ResMut<RobotSet>,
) {
    for (robot, robot_handle, curr_transform) in moved_robots.iter() {
        let RobotSerdeData {
            links,
            ..
        } = robot_set.robots.get_mut(robot_handle.0).unwrap();

        for (rb_e, init_trans) in links
            .iter()
            .map(|v| (v.rigid_body, v.initial_pos))
        {
            let rb_transform = &mut *part_transform_q.get_mut(rb_e).unwrap();
            *rb_transform = curr_transform.mul(init_trans);
        }

        let iso = k::Isometry3 {
            translation: k::Translation3::from(curr_transform.translation().to_array()),
            rotation: k::UnitQuaternion::new_unchecked(k::nalgebra::Quaternion::from(curr_transform.rotation().to_array()))
        };
        robot.chain.set_origin(iso);
    }
}

pub fn delete_robots(
    mut deleted_robots: RemovedComponents<RobotEntity>,
    robot_set: Res<RobotSet>,
    mut commands: Commands,
) {
    for robot in deleted_robots.read() {
        let Some((_, robot_data)) = robot_set.robots
            .iter()
            .find(|(_, data)| data.robot_entity == robot)
        else { continue };
        for l in robot_data.links.iter() {
            commands.entity(l.rigid_body).despawn();
            for coll in l.colliders.iter() {
                commands.entity(*coll).despawn();
            }
        }
    }
}

fn get_robot_handles(
    robot: UrdfRobot,
    robot_joint_type: RobotJointType,
    context: &mut RapierContextMut,
) -> Result<UrdfRobotHandles<Index>> {
    let RapierContextMut {
        rigidbody_set,
        colliders,
        joints,
        ..
    } = context;
    let handles = match robot_joint_type {
        RobotJointType::Multibody(options) => {
            let h = robot.insert_using_multibody_joints(
                &mut rigidbody_set.bodies,
                &mut colliders.colliders,
                &mut joints.multibody_joints,
                options.clone(),
            );
            if h.joints.iter().any(|v| v.joint.is_none()) {
                return Err(Error::ImportError("Joint loops are not supported when using Multibody joints".to_string()));
            }
            let joints: Vec<_> = h.joints.into_iter()
                .map(|v| UrdfJointHandle {
                    joint: v.joint.unwrap().0,
                    link1: v.link1,
                    link2: v.link2,
                })
                .collect();
            UrdfRobotHandles { joints, links: h.links }
        },
        RobotJointType::Impulse => {
            let h = robot.insert_using_impulse_joints(
                &mut rigidbody_set.bodies,
                &mut colliders.colliders,
                &mut joints.impulse_joints
            );
            let joints: Vec<_> = h.joints.into_iter()
                .map(|v| UrdfJointHandle {
                    joint: v.joint.0,
                    link1: v.link1,
                    link2: v.link2,
                })
                .collect();
            UrdfRobotHandles { joints, links: h.links }
        },
    };
    Ok(handles)
}

/// Returns list of created rigid body entities.
fn create_rapier_robot_entities(
    robot: &RobotEntity,
    handles: &UrdfRobotHandles<Index>,
    context: &mut RapierContextMut,
    context_link: &RapierContextEntityLink,
    robot_joint_type: &RobotJointType,
    robot_entity: Entity,
    mut meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    white_mat: &Handle<StandardMaterial>,
    commands: &mut Commands,
) -> Vec<RobotLink> {
    use rapier::prelude::{RigidBodyType, ColliderHandle, ImpulseJointHandle, MultibodyJointHandle};
    let RapierContextMut {
        rigidbody_set,
        colliders,
        joints,
        ..
    } = context;
    let crate::convert::RapierRigidBodySet {
        bodies,
        entity2body,
        ..
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierRigidBodySet>(&mut **rigidbody_set) };
    let crate::convert::RapierContextColliders {
        colliders,
        entity2collider
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierContextColliders>(&mut **colliders) };
    let crate::convert::RapierContextJoints {
        impulse_joints,
        multibody_joints,
        entity2impulse_joint,
        entity2multibody_joint,
    } = unsafe { std::mem::transmute::<_, &mut crate::convert::RapierContextJoints>(&mut **joints) };

    let mut link_entities = Vec::with_capacity(handles.links.len());
    //Spawning link & collider entities
    for (idx, link) in handles.links.iter().enumerate() {
        let rb = bodies.get_mut(link.body).unwrap();

        let mut visual_entities = Vec::new();
        for visual in robot.urdf.links[idx].visual.iter() {
            let mut mesh_handles = Vec::new();
                match &visual.geometry {
                    Geometry::Box { size } => {
                        let [x,y,z] = size.0;
                        mesh_handles.push(meshes.add(Cuboid::new(x as Real, y as Real, z as Real)))
                    }
                    Geometry::Cylinder { radius, length } => {
                        mesh_handles.push(meshes.add(Cylinder::new(*radius as Real, *length as Real)))
                    }
                    Geometry::Capsule { radius, length } => {
                        mesh_handles.push(meshes.add(Capsule3d::new(*radius as Real, *length as Real)))
                    }
                    Geometry::Sphere { radius } => {
                        mesh_handles.push(meshes.add(Sphere::new(*radius as Real)))
                    }
                    Geometry::Mesh { filename, scale } => {
                        let scale = scale
                            .map(|s| Vector::new(s[0] as Real, s[1] as Real, s[2] as Real))
                            .unwrap_or_else(|| Vector::<Real>::repeat(1.0));
                        let full_path = robot.mesh_dir.join(filename);
                        let Ok(shapes) = rapier3d_meshloader::load_from_path(
                            full_path,
                            &rapier::prelude::MeshConverter::TriMeshWithFlags(robot.loader_options.trimesh_flags),
                            scale,
                        ) else {
                            continue;
                        };

                        let mut mesh_handles2 = Vec::with_capacity(shapes.len());
                        for mesh in shapes.into_iter()
                            .filter_map(|r| r.map(|s| s.raw_mesh).ok())
                        {
                            let mut indices = Vec::with_capacity(mesh.faces.len()*3);
                            for f in mesh.faces.into_iter() {
                                indices.push(f[0]);
                                indices.push(f[1]);
                                indices.push(f[2]);
                            }
                            let mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD)
                                .with_inserted_attribute(
                                    Mesh::ATTRIBUTE_POSITION,
                                    mesh.vertices
                                )
                                .with_inserted_attribute(
                                    Mesh::ATTRIBUTE_NORMAL,
                                    mesh.normals
                                )
                                .with_inserted_indices(Indices::U32(indices));
                            mesh_handles2.push(meshes.add(mesh));
                        }
                        mesh_handles.append(&mut mesh_handles2);
                    }
                };
            let mat_h =
                if visual.material.as_ref().is_some_and(|v| v.color.is_some()) {
                    let c = visual.material.as_ref().unwrap().color.as_ref().unwrap().rgba;
                    materials.add(
                        Color::linear_rgba(c[0] as Real, c[1] as Real, c[2] as Real, c[3] as Real)
                    )
                }
                else { white_mat.clone() };
            for h in mesh_handles {
                visual_entities.push(commands.spawn((
                    Mesh3d(h),
                    MeshMaterial3d(mat_h.clone()),
                )).id());
            }
        }

        let mut rb_e_cmd = {
            commands.spawn((
                iso_to_transform(rb.position()),
                ColliderMassProperties::MassProperties(bevy_rapier3d::prelude::MassProperties {
                    mass: rb.mass(),
                    local_center_of_mass: rb.mass_properties().local_mprops.local_com.coords.into(),
                    principal_inertia: W(rb.mass_properties().local_mprops.principal_inertia()).into(),
                    principal_inertia_local_frame: rb.mass_properties().local_mprops.principal_inertia_local_frame.into(),
                }),
                Velocity {
                    angvel: W(*rb.angvel()).into(),
                    linvel: W(*rb.linvel()).into(),
                },
                Damping {
                    linear_damping: rb.linear_damping(),
                    angular_damping: rb.angular_damping(),
                },
                Ccd { enabled: rb.is_ccd_enabled() },
                Sleeping {
                    sleeping: rb.activation().sleeping,
                    angular_threshold: rb.activation().angular_threshold,
                    normalized_linear_threshold: rb.activation().normalized_linear_threshold
                },
                match rb.body_type() {
                    RigidBodyType::Dynamic => {RigidBody::Dynamic}
                    RigidBodyType::Fixed => {RigidBody::Fixed}
                    RigidBodyType::KinematicVelocityBased => {RigidBody::KinematicPositionBased}
                    RigidBodyType::KinematicPositionBased => {RigidBody::KinematicVelocityBased}
                },
                Dominance { groups: rb.dominance_group() },
                AdditionalSolverIterations(rb.additional_solver_iterations()),
                RapierRigidBodyHandle(link.body),
                *context_link,
                InheritedVisibility::default(),

                RobotPart(robot_entity),
            ))
        };
        // Rigid body enable/disable
        if !rb.is_enabled() { rb_e_cmd.insert(RigidBodyDisabled); }
        // Adding visuals
        rb_e_cmd.add_children(visual_entities.as_slice());

        // Adding colliders
        let mut collider_entities = Vec::with_capacity(link.colliders.len());
        let collider_components = |coll: &rapier::prelude::Collider, coll_handle: ColliderHandle| {
            (
                bevy_rapier3d::prelude::Collider::from(coll.shared_shape().clone()),
                RapierColliderHandle(coll_handle),
            )
        };
        if let Some(first_coll_handle) = link.colliders.first().cloned() {
            let coll = colliders.get_mut(first_coll_handle).unwrap();
            rb_e_cmd.insert(collider_components(coll, first_coll_handle));
            coll.user_data = rb_e_cmd.id().to_bits() as u128;
            entity2collider.insert(rb_e_cmd.id(), first_coll_handle);
            collider_entities.push(rb_e_cmd.id());
        }
        let rb_e = rb_e_cmd.id();
        std::mem::drop(rb_e_cmd);
        for coll_handle in link.colliders.iter().cloned().skip(1) {
            let coll = colliders.get_mut(coll_handle).unwrap();
            let coll_e = commands.spawn((
                collider_components(coll, coll_handle),
                RobotPart(robot_entity),
                *context_link,
            ))
                .id();
            coll.user_data = coll_e.to_bits() as u128;
            entity2collider.insert(coll_e, coll_handle);
            collider_entities.push(coll_e);
        }
        rb.user_data = rb_e.to_bits() as u128;
        entity2body.insert(rb_e, link.body);

        link_entities.push(RobotLink {
            rigid_body: rb_e,
            colliders: collider_entities,
            visuals: visual_entities,
            initial_pos: iso_to_transform(rb.position()),
        });
    }

    // Adding joint components
    for UrdfJointHandle {
        joint,
        link1,
        link2,
    } in handles.joints.iter()
    {
        let par_ent = bodies
            .get(*link1)
            .map(|rb| Entity::from_bits(rb.user_data as u64))
            .unwrap();
        let child_ent = bodies
            .get(*link2)
            .map(|rb| Entity::from_bits(rb.user_data as u64))
            .unwrap();

        let mut child_ent_cmd = commands.entity(child_ent);
        match robot_joint_type {
            RobotJointType::Impulse => {
                let handle = ImpulseJointHandle(*joint);
                let impulse_joint = impulse_joints.get(handle).unwrap();
                child_ent_cmd.insert((
                    RapierImpulseJointHandle(handle),
                    ImpulseJoint::new(
                        par_ent,
                        TypedJoint::GenericJoint(GenericJoint {
                            raw: impulse_joint.data,
                        }),
                    ),
                ));
                entity2impulse_joint
                    .insert(child_ent_cmd.id(), handle);
            }
            RobotJointType::Multibody(..) => {
                let handle = MultibodyJointHandle(*joint);
                let (mb, link_id) = multibody_joints.get(handle).unwrap();
                child_ent_cmd.insert((
                    RapierMultibodyJointHandle(handle),
                    MultibodyJoint::new(
                        par_ent,
                        TypedJoint::GenericJoint(GenericJoint {
                            raw: mb.link(link_id).unwrap().joint.data,
                        }),
                    ),
                ));
                entity2multibody_joint
                    .insert(child_ent_cmd.id(), handle);
            }
        }
    }

    link_entities
}

pub fn create_rapier_robot(
    robot: urdf_rs::Robot,
    options: UrdfLoaderOptions,
    mesh_dir: &Path,
    approximate_collisions: bool,
    convex_decomp_options: VHACDParameters,
    all_joints_have_motors: bool,
) -> Result<UrdfRobot> {
    let mut name_to_link_id = HashMap::new();
    let mut link_is_root = vec![true; robot.links.len()];
    let mut approximated_meshes = HashMap::new();
    let mut links: Vec<_> = robot
        .links
        .iter()
        .enumerate()
        .map(|(id, link)| {
            name_to_link_id.insert(&link.name, id);
            let mut colliders = vec![];
            if options.create_colliders_from_collision_shapes {
                colliders.extend(link.collision.iter().flat_map(|co| {
                    urdf_to_colliders(
                        &options,
                        mesh_dir,
                        &co.geometry,
                        &co.origin,
                        approximate_collisions,
                        convex_decomp_options.clone(),
                        &mut approximated_meshes
                    )
                }))
            }
            if options.create_colliders_from_visual_shapes {
                colliders.extend(link.visual.iter().flat_map(|vis| {
                    urdf_to_colliders(
                        &options,
                        mesh_dir,
                        &vis.geometry,
                        &vis.origin,
                        approximate_collisions,
                        convex_decomp_options.clone(),
                        &mut approximated_meshes
                    )
                }))
            }
            let mut body = urdf_to_rigid_body(&options, &link.inertial);
            body.set_position(options.shift * body.position(), false);
            UrdfLink { body, colliders }
        })
        .collect();
    let mut joints: Vec<_> = robot
        .joints
        .iter()
        .map(|joint| {
            let link1 = name_to_link_id[&joint.parent.link];
            let link2 = name_to_link_id[&joint.child.link];
            let pose1 = *links[link1].body.position();
            let rb2 = &mut links[link2].body;
            let joint = urdf_to_joint(&options, joint, &pose1, rb2);
            link_is_root[link2] = false;

            UrdfJoint {
                joint,
                link1,
                link2,
            }
        })
        .collect();
    if !all_joints_have_motors {
        for transmission in robot.transmissions.iter() {
            let Some(joint_name) = transmission
                .joints
                .first()
                .map(|v| &v.name)
            else {
                return Err(Error::Urdf(
                    robot.name,
                    format!("Transmission {} has no joints", transmission.name),
                ))
            };
            let Some(joint_idx) = robot.joints.iter()
                .position(|v| v.name == *joint_name)
            else {
                return Err(Error::Urdf(
                    robot.name,
                    format!("Couldn't find joint {} specified by transmission {}", joint_name, transmission.name),
                ))
            };
            let joint = joints
                .get_mut(joint_idx)
                .map(|v| &mut v.joint)
                .unwrap();
            let urdf = &robot.joints[joint_idx];

            joint.motor_axes = !joint.locked_axes;
            for motor_idx in 0..SPATIAL_DIM {
                if joint.motor_axes.bits() & 1 << motor_idx != 0 {
                    let motor = &mut joint.motors[motor_idx];
                    motor.max_force = urdf.limit.effort as Real;
                    motor.target_pos = ((urdf.limit.lower + urdf.limit.upper)/2.) as Real;
                    motor.target_vel = urdf.limit.velocity as Real;
                    motor.stiffness = 1.;
                    motor.damping = 0.;
                    motor.model = MotorModel::AccelerationBased;
                    if let Some(ctrl) = &urdf.safety_controller {
                        motor.target_pos = ctrl.k_position as Real;
                        motor.target_vel = ctrl.k_velocity as Real;
                    }
                }
            }
        }
    }
    else {
        for (idx, urdf) in robot.joints.iter().enumerate() {
            let joint = &mut joints[idx].joint;
            joint.motor_axes = !joint.locked_axes;
            for motor_idx in 0..SPATIAL_DIM {
                if joint.motor_axes.bits() & 1 << motor_idx != 0 {
                    let motor = &mut joint.motors[motor_idx];
                    motor.max_force = urdf.limit.effort as Real;
                    motor.target_pos = urdf.limit.lower as Real;
                    motor.target_vel = urdf.limit.velocity as Real;
                    motor.stiffness = 1.;
                    motor.model = MotorModel::AccelerationBased;
                }
            }
        }
    }

    if options.make_roots_fixed {
        for (link, is_root) in links.iter_mut().zip(link_is_root.into_iter()) {
            if is_root {
                link.body.set_body_type(rapier::prelude::RigidBodyType::Fixed, false)
            }
        }
    }

    Ok(UrdfRobot { links, joints })
}

#[rustfmt::skip]
fn urdf_to_rigid_body(options: &UrdfLoaderOptions, inertial: &Inertial) -> rapier::prelude::RigidBody {
    let origin = urdf_to_isometry(&inertial.origin);
    let mut builder = options.rigid_body_blueprint.clone();
    builder.body_type = rapier::prelude::RigidBodyType::Dynamic;

    if options.apply_imported_mass_props {
        builder = builder.additional_mass_properties(MassProperties::with_inertia_matrix(
            origin.translation.vector.into(),
            inertial.mass.value as Real,
            // See http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Inertia
            na::Matrix3::new(
                inertial.inertia.ixx as Real, inertial.inertia.ixy as Real, inertial.inertia.ixz as Real,
                inertial.inertia.ixy as Real, inertial.inertia.iyy as Real, inertial.inertia.iyz as Real,
                inertial.inertia.ixz as Real, inertial.inertia.iyz as Real,inertial.inertia.izz as Real,
            ),
        ))
    }

    builder.build()
}

fn urdf_to_colliders(
    options: &UrdfLoaderOptions,
    _mesh_dir: &Path,
    geometry: &Geometry,
    origin: &Pose,
    approximate_collisions: bool,
    convex_decomp_options: VHACDParameters,
    approximated_meshes: &mut HashMap<u64, Vec<SharedShape>>
) -> Vec<rapier::prelude::Collider> {
    let mut shape_transform = Isometry::identity();

    let mut colliders = Vec::new();

    match &geometry {
        Geometry::Box { size } => {
            colliders.push(SharedShape::cuboid(
                size[0] as Real / 2.0,
                size[1] as Real / 2.0,
                size[2] as Real / 2.0,
            ));
        }
        Geometry::Cylinder { radius, length } => {
            // This rotation will make the cylinder Z-up as per the URDF spec,
            // instead of rapier’s default Y-up.
            shape_transform = Isometry::rotation(Vector::x() * Real::frac_pi_2());
            colliders.push(SharedShape::cylinder(
                *length as Real / 2.0,
                *radius as Real,
            ));
        }
        Geometry::Capsule { radius, length } => {
            colliders.push(SharedShape::capsule_z(
                *length as Real / 2.0,
                *radius as Real,
            ));
        }
        Geometry::Sphere { radius } => {
            colliders.push(SharedShape::ball(*radius as Real));
        }
        Geometry::Mesh { filename, scale } => {
            let full_path = _mesh_dir.join(filename);
            let scale = scale
                .map(|s| Vector::new(s[0] as Real, s[1] as Real, s[2] as Real))
                .unwrap_or_else(|| Vector::<Real>::repeat(1.0));

            let Ok(loaded_mesh) = rapier3d_meshloader::load_from_path(
                full_path,
                &rapier::prelude::MeshConverter::TriMeshWithFlags(options.trimesh_flags),
                scale,
            ) else {
                return Vec::new();
            };

            // Use convex decomposition to approximate geometry
            if approximate_collisions {
                bevy::log::info!("Approximating collider {filename}");
                let file_hash = ahash::RandomState::generate_with(1,2,3,4).hash_one(filename);
                let mut shapes: Vec<_> =
                    if let Some((_, shape)) = approximated_meshes
                        .iter()
                        .find(|(v, _)| **v == file_hash)
                    { shape.clone() }
                    else {
                        let shapes: Vec<_> = loaded_mesh
                            .into_iter()
                            .filter_map(|v| v.map(|s| s.raw_mesh).ok())
                            .map(|mesh| {
                                let ae: Vec<_> = mesh.vertices.into_iter()
                                    .map(|v| rapier::na::Point3::from(v))
                                    .collect();
                                SharedShape::convex_decomposition_with_params(
                                    ae.as_slice(),
                                    mesh.faces.as_slice(),
                                    &convex_decomp_options,
                                )
                            })
                            .collect();
                        approximated_meshes.insert(file_hash, shapes.clone());
                        shapes
                    };
                colliders.append(&mut shapes)
            }
            // Use trimesh when not approximating geometry
            else {
                colliders.append(
                    &mut loaded_mesh
                        .into_iter()
                        .filter_map(|x| x.map(|s| s.shape).ok())
                        .collect(),
                );
            }
        }
    }

    colliders
        .drain(..)
        .map(move |shape| {
            let mut builder = options.collider_blueprint.clone();
            builder.shape = shape;
            builder
                .position(urdf_to_isometry(origin) * shape_transform)
                .build()
        })
        .collect()
}

fn urdf_to_isometry(pose: &Pose) -> Isometry<Real> {
    Isometry::from_parts(
        Point::new(
            pose.xyz[0] as Real,
            pose.xyz[1] as Real,
            pose.xyz[2] as Real,
        )
            .into(),
        bevy_rapier3d::na::UnitQuaternion::from_euler_angles(
            pose.rpy[0] as Real,
            pose.rpy[1] as Real,
            pose.rpy[2] as Real,
        ),
    )
}

fn urdf_to_joint(
    options: &UrdfLoaderOptions,
    joint: &Joint,
    pose1: &Isometry<Real>,
    link2: &mut rapier::prelude::RigidBody,
) -> rapier::prelude::GenericJoint {
    let locked_axes = match joint.joint_type {
        urdf_rs::JointType::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        urdf_rs::JointType::Continuous | urdf_rs::JointType::Revolute => {
            JointAxesMask::LOCKED_REVOLUTE_AXES
        }
        urdf_rs::JointType::Floating => JointAxesMask::empty(),
        urdf_rs::JointType::Planar => JointAxesMask::ANG_AXES | JointAxesMask::LIN_X,
        urdf_rs::JointType::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
        urdf_rs::JointType::Spherical => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let joint_to_parent = urdf_to_isometry(&joint.origin);
    let joint_axis = na::Unit::try_new(
        Vector::new(
            joint.axis.xyz[0] as Real,
            joint.axis.xyz[1] as Real,
            joint.axis.xyz[2] as Real,
        ),
        1.0e-5,
    );

    link2.set_position(pose1 * joint_to_parent, false);

    let mut builder = rapier::prelude::GenericJointBuilder::new(locked_axes)
        .local_anchor1(joint_to_parent.translation.vector.into())
        .contacts_enabled(options.enable_joint_collisions);

    if let Some(joint_axis) = joint_axis {
        builder = builder
            .local_axis1(joint_to_parent * joint_axis)
            .local_axis2(joint_axis);
    }

    match joint.joint_type {
        urdf_rs::JointType::Prismatic => {
            builder = builder.limits(
                JointAxis::LinX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        urdf_rs::JointType::Revolute => {
            builder = builder.limits(
                JointAxis::AngX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        _ => {}
    }

    // TODO: the following fields are currently ignored:
    //       - Joint::dynamics
    //       - Joint::limit.effort / limit.velocity
    //       - Joint::mimic
    //       - Joint::safety_controller
    builder.build()
}
