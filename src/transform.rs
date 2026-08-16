//! 3D Rigid Body Transformations (SE(3)) combining rotation (SO(3) DCM) and translational offset.

use core::fmt::Debug;
use core::marker::PhantomData;

use crate::dcm::DirectionCosineMatrix;
use crate::frames::Frame;
use crate::point::Point3;
use crate::units::Unit;
use crate::vector::Vector3;

/// A 3D SE(3) Rigid Body Transformation transforming points and vectors from frame `From` to frame `To`.
///
/// Contains:
/// - `dcm`: SO(3) Rotation Matrix (`From -> To`)
/// - `translation`: Translational origin displacement of `From` expressed in `To` frame.
#[repr(C)]
pub struct Transform3D<From: Frame, To: Frame, U: Unit, T = f32> {
    pub dcm: DirectionCosineMatrix<From, To, T>,
    pub translation: Vector3<To, U, T>,
    _from: PhantomData<From>,
    _to: PhantomData<To>,
}

impl<From: Frame, To: Frame, U: Unit, T: Debug> Debug for Transform3D<From, To, U, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Transform3D")
            .field("from", &core::any::type_name::<From>())
            .field("to", &core::any::type_name::<To>())
            .field("unit", &core::any::type_name::<U>())
            .field("dcm", &self.dcm)
            .field("translation", &self.translation)
            .finish()
    }
}

impl<From: Frame, To: Frame, U: Unit, T: Copy> Clone for Transform3D<From, To, U, T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        *self
    }
}

impl<From: Frame, To: Frame, U: Unit, T: Copy> Copy for Transform3D<From, To, U, T> {}

impl<From: Frame, To: Frame, U: Unit, T: PartialEq> PartialEq for Transform3D<From, To, U, T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.dcm == other.dcm && self.translation == other.translation
    }
}

impl<From: Frame, To: Frame, U: Unit, T> Transform3D<From, To, U, T> {
    /// Create a rigid transformation from a DCM and translation vector.
    #[inline(always)]
    pub const fn new(
        dcm: DirectionCosineMatrix<From, To, T>,
        translation: Vector3<To, U, T>,
    ) -> Self {
        Self {
            dcm,
            translation,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}

impl<From: Frame, To: Frame, U: Unit> Transform3D<From, To, U, f32> {
    /// Pure rotation transform (zero translation).
    #[inline(always)]
    pub const fn pure_rotation(dcm: DirectionCosineMatrix<From, To, f32>) -> Self {
        Self::new(dcm, Vector3::<To, U, f32>::ZERO)
    }

    /// Transform a 3D point from frame `From` to frame `To`.
    #[inline(always)]
    pub fn transform_point(&self, p: Point3<From, U, f32>) -> Point3<To, U, f32> {
        let rotated_vec = self.dcm.rotate_vector(p.to_vector());
        Point3::origin() + self.translation + rotated_vec
    }

    /// Transform a 3D direction/velocity vector from frame `From` to frame `To` (rotates only).
    #[inline(always)]
    pub fn transform_vector(&self, v: Vector3<From, U, f32>) -> Vector3<To, U, f32> {
        self.dcm.rotate_vector(v)
    }

    /// Compute the inverse rigid transformation (`To -> From`).
    #[inline(always)]
    pub fn inverse(&self) -> Transform3D<To, From, U, f32> {
        let inv_dcm = self.dcm.transpose();
        let inv_translation = -inv_dcm.rotate_vector(self.translation);
        Transform3D::new(inv_dcm, inv_translation)
    }

    /// Chain two rigid transformations together (`self`: From -> Inter, `next`: Inter -> Target).
    #[inline(always)]
    pub fn chain<Target: Frame>(
        &self,
        next: &Transform3D<To, Target, U, f32>,
    ) -> Transform3D<From, Target, U, f32> {
        let combined_dcm = self.dcm.chain(&next.dcm);
        let combined_translation = next.translation + next.dcm.rotate_vector(self.translation);
        Transform3D::new(combined_dcm, combined_translation)
    }
}

impl<From: Frame, To: Frame, U: Unit> Transform3D<From, To, U, f64> {
    /// Pure rotation transform (zero translation).
    #[inline(always)]
    pub const fn pure_rotation(dcm: DirectionCosineMatrix<From, To, f64>) -> Self {
        Self::new(dcm, Vector3::<To, U, f64>::ZERO)
    }

    /// Transform a 3D point from frame `From` to frame `To`.
    #[inline(always)]
    pub fn transform_point(&self, p: Point3<From, U, f64>) -> Point3<To, U, f64> {
        let rotated_vec = self.dcm.rotate_vector(p.to_vector());
        Point3::origin() + self.translation + rotated_vec
    }

    /// Transform a 3D direction/velocity vector from frame `From` to frame `To` (rotates only).
    #[inline(always)]
    pub fn transform_vector(&self, v: Vector3<From, U, f64>) -> Vector3<To, U, f64> {
        self.dcm.rotate_vector(v)
    }

    /// Compute the inverse rigid transformation (`To -> From`).
    #[inline(always)]
    pub fn inverse(&self) -> Transform3D<To, From, U, f64> {
        let inv_dcm = self.dcm.transpose();
        let inv_translation = -inv_dcm.rotate_vector(self.translation);
        Transform3D::new(inv_dcm, inv_translation)
    }

    /// Chain two rigid transformations together (`self`: From -> Inter, `next`: Inter -> Target).
    #[inline(always)]
    pub fn chain<Target: Frame>(
        &self,
        next: &Transform3D<To, Target, U, f64>,
    ) -> Transform3D<From, Target, U, f64> {
        let combined_dcm = self.dcm.chain(&next.dcm);
        let combined_translation = next.translation + next.dcm.rotate_vector(self.translation);
        Transform3D::new(combined_dcm, combined_translation)
    }
}
