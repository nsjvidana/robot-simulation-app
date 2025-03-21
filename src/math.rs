pub use bevy_rapier_math::*;
pub use k_math::*;

pub type Real = f32;

mod k_math {
    use crate::math::Real;
    use k::nalgebra::{Quaternion, UnitVector3, Vector4};
    use k::{UnitQuaternion, Vector3};

    pub fn project_onto_plane(
        vector: &Vector3<Real>,
        plane_normal: &UnitVector3<Real>,
    ) -> Vector3<Real> {
        let normsq = plane_normal.norm_squared();
        let dot = vector.dot(plane_normal);
        let div = dot / normsq;
        let [[x, y, z]] = vector.data.0;
        let [[n_x, n_y, n_z]] = plane_normal.data.0;
        Vector3::<Real>::new(x - n_x * div, y - n_y * div, z - n_z * div)
    }

    /// Computes the shortest angle from vector `a` to vector `b` that lie on the same plane
    /// that has a normal `n`.
    ///
    /// The angle that is returned comes with the appropriate sign for a right-handed rotation
    /// (Positive angle for counterclockwise rotation, negative for clockwise)
    pub fn angle_to(a: &Vector3<Real>, b: &Vector3<Real>, n: &UnitVector3<Real>) -> Real {
        bevy_rapier3d::na::SimdRealField::simd_atan2(a.cross(b).dot(n), a.dot(b))
    }

    /// Computes a quaternion representing the shortest rotation from vector `a` to vector `b`.
    pub fn rotation_between_vectors(a: &Vector3<Real>, b: &Vector3<Real>) -> UnitQuaternion<Real> {
        let axis = a.cross(b);
        let [[x, y, z]] = axis.data.0;
        let q = Quaternion {
            coords: Vector4::new(
                x,
                y,
                z,
                Real::sqrt(a.norm_squared() * b.norm_squared()) + a.dot(b),
            ),
        };
        UnitQuaternion::new_normalize(q)
    }
}

mod bevy_rapier_math {
    use crate::math::Real;
    use bevy_rapier3d::na::{UnitVector3, Vector3};

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
}
