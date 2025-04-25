use bevy_rapier3d::na::*;
use bevy_rapier3d::rapier::prelude::Ray;
use crate::prelude::Real;

/// Returns a value `a` such that `ray_origin + (ray_dir * a)` gives a value `interesction_point`
///
/// Note: `a` can be negative, in which case `intersection_point` will be behind the ray.
pub fn ray_scale_for_plane_intersect_local(
    normal: &UnitVector3<Real>,
    ray_origin: &Vector3<Real>,
    ray_dir: &Vector3<Real>,
) -> Option<Real> {
    let numerator = normal.dot(ray_origin);
    let denom = normal.dot(ray_dir);
    if denom == 0. {
        return None;
    }
    Some(-numerator / denom)
}

pub fn compute_plane_intersection_pos(
    ray: &Ray,
    plane_pos: &Vector3<Real>,
    plane_normal: &UnitVector3<Real>,
) -> Option<Vector3<Real>> {
    let scale = ray_scale_for_plane_intersect_local(
        &plane_normal,
        &(ray.origin.coords - plane_pos),
        &ray.dir,
    );
    if let Some(scale) = scale {
        if scale >= 0. {
            // Ensure the intersection isn't behind the camera
            return Some(ray.origin.coords + ray.dir * scale);
        }
    }
    None
}