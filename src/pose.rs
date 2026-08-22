//! Engineering 3D Pose representation (Position + Orientation) with Zero-Cost Frame Typestates.

use crate::dcm::DirectionCosineMatrix;
use crate::frames::Frame;
use crate::point::Point3;
use crate::transform::Transform3D;
use crate::units::Unit;
use crate::vector::Vector3;
use core::fmt::Debug;

/// 3D Spatial Pose combining a 3D position (`Point3<To, U, T>`) and attitude orientation (`DirectionCosineMatrix<From, To, T>`).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(serialize = "T: serde::Serialize", deserialize = "T: serde::de::DeserializeOwned"))
)]
pub struct Pose3D<From: Frame, To: Frame, U: Unit, T: Copy = f32> {
    /// Position point of origin in `To` reference frame.
    pub position: Point3<To, U, T>,
    /// Orientation rotation matrix transforming vectors from `From` to `To` frame.
    pub orientation: DirectionCosineMatrix<From, To, T>,
}

impl<From: Frame, To: Frame, U: Unit> Pose3D<From, To, U, f32> {
    /// Create a new 3D Pose from position point and orientation DCM.
    #[inline(always)]
    pub const fn new(
        position: Point3<To, U, f32>,
        orientation: DirectionCosineMatrix<From, To, f32>,
    ) -> Self {
        Self {
            position,
            orientation,
        }
    }

    /// Convert Pose3D to an SE(3) Rigid Body Transformation matrix.
    #[inline(always)]
    pub fn to_transform(&self) -> Transform3D<From, To, U, f32> {
        Transform3D::new(self.orientation, self.position.to_vector())
    }

    /// Transform a point from `From` frame to `To` frame using this pose.
    #[inline(always)]
    pub fn transform_point(&self, pt: Point3<From, U, f32>) -> Point3<To, U, f32> {
        self.to_transform().transform_point(pt)
    }

    /// Transform a vector from `From` frame to `To` frame using this pose orientation.
    #[inline(always)]
    pub fn transform_vector(&self, vec: Vector3<From, U, f32>) -> Vector3<To, U, f32> {
        self.orientation.rotate_vector(vec)
    }

    /// Invert pose to transform from `To` frame to `From` frame.
    #[inline]
    pub fn inverse(&self) -> Pose3D<To, From, U, f32> {
        let inv_orientation = self.orientation.transpose();
        let inv_vec = inv_orientation.rotate_vector(self.position.to_vector());
        let inv_position = Point3::origin() - inv_vec;

        Pose3D::new(inv_position, inv_orientation)
    }
}
