//! Direction Cosine Matrix (DCM) for SO(3) frame rotations.

use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::Mul;

use crate::frames::Frame;
use crate::units::Unit;
use crate::vector::Vector3;

/// A 3x3 Direction Cosine Matrix (SO(3) rotation matrix) transforming vectors from `From` frame to `To` frame.
///
/// Under standard aerospace / robotics conventions:
/// `v_To = DCM_{From->To} * v_From`
#[repr(C)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(serialize = "T: serde::Serialize", deserialize = "T: serde::de::DeserializeOwned"))
)]
pub struct DirectionCosineMatrix<From: Frame, To: Frame, T = f32> {
    pub m: [[T; 3]; 3],
    #[cfg_attr(feature = "serde", serde(skip))]
    _from: PhantomData<From>,
    #[cfg_attr(feature = "serde", serde(skip))]
    _to: PhantomData<To>,
}

impl<From: Frame, To: Frame, T: Debug> Debug for DirectionCosineMatrix<From, To, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DirectionCosineMatrix")
            .field("from", &core::any::type_name::<From>())
            .field("to", &core::any::type_name::<To>())
            .field("m", &self.m)
            .finish()
    }
}

impl<From: Frame, To: Frame, T: Copy> Clone for DirectionCosineMatrix<From, To, T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        *self
    }
}

impl<From: Frame, To: Frame, T: Copy> Copy for DirectionCosineMatrix<From, To, T> {}

impl<From: Frame, To: Frame, T: PartialEq> PartialEq for DirectionCosineMatrix<From, To, T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.m == other.m
    }
}

impl<From: Frame, To: Frame, T> DirectionCosineMatrix<From, To, T> {
    /// Create a DCM from raw 3x3 array matrix `[[r0c0, r0c1, r0c2], [r1c0, ...], ...]`.
    #[inline(always)]
    pub const fn new(m: [[T; 3]; 3]) -> Self {
        Self {
            m,
            _from: PhantomData,
            _to: PhantomData,
        }
    }

    /// Create DCM from three row vectors.
    #[inline(always)]
    pub const fn from_rows(r0: [T; 3], r1: [T; 3], r2: [T; 3]) -> Self {
        Self::new([r0, r1, r2])
    }
}

impl<F: Frame, T: Copy + Default> DirectionCosineMatrix<F, F, T> {
    /// Identity rotation matrix for identical from/to frames.
    #[inline(always)]
    pub fn identity() -> Self
    where
        T: From<u8>,
    {
        let one = T::from(1);
        let zero = T::default();
        Self::new([[one, zero, zero], [zero, one, zero], [zero, zero, one]])
    }
}

impl<From: Frame, To: Frame> DirectionCosineMatrix<From, To, f32> {
    /// Identity rotation.
    pub const IDENTITY: Self = Self::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// Construct DCM from Z-Y-X Euler angles (Yaw, Pitch, Roll in radians).
    ///
    /// Transforms a vector from reference frame `From` to `To`.
    #[cfg(feature = "std")]
    pub fn from_euler_zyx(yaw: f32, pitch: f32, roll: f32) -> Self {
        let (cy, sy) = (yaw.cos(), yaw.sin());
        let (cp, sp) = (pitch.cos(), pitch.sin());
        let (cr, sr) = (roll.cos(), roll.sin());

        Self::new([
            [cy * cp, sy * cp, -sp],
            [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
            [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr],
        ])
    }

    /// Construct DCM from Z-Y-X Euler angles (`no_std` using `libm`).
    #[cfg(not(feature = "std"))]
    pub fn from_euler_zyx(yaw: f32, pitch: f32, roll: f32) -> Self {
        let (cy, sy) = (libm::cosf(yaw), libm::sinf(yaw));
        let (cp, sp) = (libm::cosf(pitch), libm::sinf(pitch));
        let (cr, sr) = (libm::cosf(roll), libm::sinf(roll));

        Self::new([
            [cy * cp, sy * cp, -sp],
            [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
            [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr],
        ])
    }

    /// Transpose DCM matrix, effectively reversing the rotation direction (`To -> From`).
    ///
    /// Since DCM matrices are orthogonal, $R^{-1} = R^T$.
    #[inline(always)]
    pub fn transpose(self) -> DirectionCosineMatrix<To, From, f32> {
        DirectionCosineMatrix::new([
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]],
        ])
    }

    /// Rotate a vector from `From` frame to `To` frame.
    #[inline(always)]
    pub fn rotate_vector<U: Unit>(&self, v: Vector3<From, U, f32>) -> Vector3<To, U, f32> {
        let x = self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z;
        let y = self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z;
        let z = self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z;
        Vector3::new(x, y, z)
    }

    /// Chain two rotations together: `self` (From -> To) followed by `next` (To -> Target).
    /// Returns DCM from `From` frame directly to `Target` frame.
    #[inline(always)]
    pub fn chain<Target: Frame>(
        &self,
        next: &DirectionCosineMatrix<To, Target, f32>,
    ) -> DirectionCosineMatrix<From, Target, f32> {
        let m = [
            [
                next.m[0][0] * self.m[0][0]
                    + next.m[0][1] * self.m[1][0]
                    + next.m[0][2] * self.m[2][0],
                next.m[0][0] * self.m[0][1]
                    + next.m[0][1] * self.m[1][1]
                    + next.m[0][2] * self.m[2][1],
                next.m[0][0] * self.m[0][2]
                    + next.m[0][1] * self.m[1][2]
                    + next.m[0][2] * self.m[2][2],
            ],
            [
                next.m[1][0] * self.m[0][0]
                    + next.m[1][1] * self.m[1][0]
                    + next.m[1][2] * self.m[2][0],
                next.m[1][0] * self.m[0][1]
                    + next.m[1][1] * self.m[1][1]
                    + next.m[1][2] * self.m[2][1],
                next.m[1][0] * self.m[0][2]
                    + next.m[1][1] * self.m[1][2]
                    + next.m[1][2] * self.m[2][2],
            ],
            [
                next.m[2][0] * self.m[0][0]
                    + next.m[2][1] * self.m[1][0]
                    + next.m[2][2] * self.m[2][0],
                next.m[2][0] * self.m[0][1]
                    + next.m[2][1] * self.m[1][1]
                    + next.m[2][2] * self.m[2][1],
                next.m[2][0] * self.m[0][2]
                    + next.m[2][1] * self.m[1][2]
                    + next.m[2][2] * self.m[2][2],
            ],
        ];
        DirectionCosineMatrix::new(m)
    }
}

impl<From: Frame, To: Frame> DirectionCosineMatrix<From, To, f64> {
    /// Identity rotation.
    pub const IDENTITY: Self = Self::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// Construct DCM from Z-Y-X Euler angles (Yaw, Pitch, Roll in radians).
    #[cfg(feature = "std")]
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        let (cy, sy) = (yaw.cos(), yaw.sin());
        let (cp, sp) = (pitch.cos(), pitch.sin());
        let (cr, sr) = (roll.cos(), roll.sin());

        Self::new([
            [cy * cp, sy * cp, -sp],
            [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
            [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr],
        ])
    }

    /// Construct DCM from Z-Y-X Euler angles (`no_std` using `libm`).
    #[cfg(not(feature = "std"))]
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        let (cy, sy) = (libm::cos(yaw), libm::sin(yaw));
        let (cp, sp) = (libm::cos(pitch), libm::sin(pitch));
        let (cr, sr) = (libm::cos(roll), libm::sin(roll));

        Self::new([
            [cy * cp, sy * cp, -sp],
            [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
            [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr],
        ])
    }

    /// Transpose DCM matrix (`To -> From`).
    #[inline(always)]
    pub fn transpose(self) -> DirectionCosineMatrix<To, From, f64> {
        DirectionCosineMatrix::new([
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]],
        ])
    }

    /// Rotate a vector from `From` frame to `To` frame.
    #[inline(always)]
    pub fn rotate_vector<U: Unit>(&self, v: Vector3<From, U, f64>) -> Vector3<To, U, f64> {
        let x = self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z;
        let y = self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z;
        let z = self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z;
        Vector3::new(x, y, z)
    }

    /// Chain two rotations together: `self` (From -> To) followed by `next` (To -> Target).
    #[inline(always)]
    pub fn chain<Target: Frame>(
        &self,
        next: &DirectionCosineMatrix<To, Target, f64>,
    ) -> DirectionCosineMatrix<From, Target, f64> {
        let m = [
            [
                next.m[0][0] * self.m[0][0]
                    + next.m[0][1] * self.m[1][0]
                    + next.m[0][2] * self.m[2][0],
                next.m[0][0] * self.m[0][1]
                    + next.m[0][1] * self.m[1][1]
                    + next.m[0][2] * self.m[2][1],
                next.m[0][0] * self.m[0][2]
                    + next.m[0][1] * self.m[1][2]
                    + next.m[0][2] * self.m[2][2],
            ],
            [
                next.m[1][0] * self.m[0][0]
                    + next.m[1][1] * self.m[1][0]
                    + next.m[1][2] * self.m[2][0],
                next.m[1][0] * self.m[0][1]
                    + next.m[1][1] * self.m[1][1]
                    + next.m[1][2] * self.m[2][1],
                next.m[1][0] * self.m[0][2]
                    + next.m[1][1] * self.m[1][2]
                    + next.m[1][2] * self.m[2][2],
            ],
            [
                next.m[2][0] * self.m[0][0]
                    + next.m[2][1] * self.m[1][0]
                    + next.m[2][2] * self.m[2][0],
                next.m[2][0] * self.m[0][1]
                    + next.m[2][1] * self.m[1][1]
                    + next.m[2][2] * self.m[2][1],
                next.m[2][0] * self.m[0][2]
                    + next.m[2][1] * self.m[1][2]
                    + next.m[2][2] * self.m[2][2],
            ],
        ];
        DirectionCosineMatrix::new(m)
    }
}

// Convenience methods to rotate a vector into a target frame via a DCM.
impl<F: Frame, U: Unit> Vector3<F, U, f32> {
    /// Rotate vector into target frame using a Direction Cosine Matrix.
    #[inline(always)]
    pub fn rotate_to<TargetFrame: Frame>(
        self,
        dcm: &DirectionCosineMatrix<F, TargetFrame, f32>,
    ) -> Vector3<TargetFrame, U, f32> {
        dcm.rotate_vector(self)
    }
}

impl<F: Frame, U: Unit> Vector3<F, U, f64> {
    /// Rotate vector into target frame using a Direction Cosine Matrix.
    #[inline(always)]
    pub fn rotate_to<TargetFrame: Frame>(
        self,
        dcm: &DirectionCosineMatrix<F, TargetFrame, f64>,
    ) -> Vector3<TargetFrame, U, f64> {
        dcm.rotate_vector(self)
    }
}

// Mul implementation for chaining DCMs: `DCM<A, B> * DCM<B, C> = DCM<A, C>`
impl<A: Frame, B: Frame, C: Frame> Mul<DirectionCosineMatrix<B, C, f32>>
    for DirectionCosineMatrix<A, B, f32>
{
    type Output = DirectionCosineMatrix<A, C, f32>;
    #[inline(always)]
    fn mul(self, rhs: DirectionCosineMatrix<B, C, f32>) -> Self::Output {
        self.chain(&rhs)
    }
}

impl<A: Frame, B: Frame, C: Frame> Mul<DirectionCosineMatrix<B, C, f64>>
    for DirectionCosineMatrix<A, B, f64>
{
    type Output = DirectionCosineMatrix<A, C, f64>;
    #[inline(always)]
    fn mul(self, rhs: DirectionCosineMatrix<B, C, f64>) -> Self::Output {
        self.chain(&rhs)
    }
}
