//! Unit Quaternion (SO(3) Rotation) implementation with Zero-Cost Frame Typestates.
//!
//! Provides singularity-free attitude representations without gimbal lock issues.

use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::Mul;

use crate::dcm::DirectionCosineMatrix;
use crate::frames::Frame;
use crate::units::Unit;
use crate::vector::Vector3;

/// Unit Quaternion $q = [w, x, y, z]$ representing rotation from `From` frame to `To` frame.
///
/// Memory layout is `#[repr(C)]` matching `[T; 4]` (16 bytes for `f32`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion<From: Frame, To: Frame, T: Copy = f32> {
    /// Scalar real component (w)
    pub w: T,
    /// Vector imaginary x component
    pub x: T,
    /// Vector imaginary y component
    pub y: T,
    /// Vector imaginary z component
    pub z: T,
    _from: PhantomData<From>,
    _to: PhantomData<To>,
}

impl<From: Frame, To: Frame, T: Copy> Quaternion<From, To, T> {
    /// Create a new Quaternion [w, x, y, z].
    #[inline(always)]
    pub const fn new(w: T, x: T, y: T, z: T) -> Self {
        Self {
            w,
            x,
            y,
            z,
            _from: PhantomData,
            _to: PhantomData,
        }
    }
}

impl<From: Frame, To: Frame, T: Copy + core::ops::Neg<Output = T>> Quaternion<From, To, T> {
    /// Conjugate quaternion representing the inverse rotation (`To` -> `From`).
    #[inline(always)]
    pub fn conjugate(&self) -> Quaternion<To, From, T> {
        Quaternion::new(self.w, -self.x, -self.y, -self.z)
    }
}

impl<From: Frame, To: Frame> Quaternion<From, To, f32> {
    /// Identity rotation quaternion [1, 0, 0, 0].
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        _from: PhantomData,
        _to: PhantomData,
    };

    /// Construct rotation quaternion from ZYX (Yaw, Pitch, Roll) Tait-Bryan Euler angles in radians (`std`).
    #[cfg(feature = "std")]
    pub fn from_euler_zyx(yaw: f32, pitch: f32, roll: f32) -> Self {
        let (cy, sy) = ((yaw * 0.5).cos(), (yaw * 0.5).sin());
        let (cp, sp) = ((pitch * 0.5).cos(), (pitch * 0.5).sin());
        let (cr, sr) = ((roll * 0.5).cos(), (roll * 0.5).sin());

        let w = cr * cp * cy + sr * sp * sy;
        let x = sr * cp * cy - cr * sp * sy;
        let y = cr * sp * cy + sr * cp * sy;
        let z = cr * cp * sy - sr * sp * cy;

        Self::new(w, x, y, z)
    }

    /// Construct rotation quaternion from ZYX Euler angles (`no_std`).
    #[cfg(not(feature = "std"))]
    pub fn from_euler_zyx(yaw: f32, pitch: f32, roll: f32) -> Self {
        let (cy, sy) = (libm::cosf(yaw * 0.5), libm::sinf(yaw * 0.5));
        let (cp, sp) = (libm::cosf(pitch * 0.5), libm::sinf(pitch * 0.5));
        let (cr, sr) = (libm::cosf(roll * 0.5), libm::sinf(roll * 0.5));

        let w = cr * cp * cy + sr * sp * sy;
        let x = sr * cp * cy - cr * sp * sy;
        let y = cr * sp * cy + sr * cp * sy;
        let z = cr * cp * sy - sr * sp * cy;

        Self::new(w, x, y, z)
    }

    /// Rotate a vector from `From` frame to `To` frame using the passive frame-transform
    /// sandwich product $v' = q^* v q$, matching the `DirectionCosineMatrix` / `Transform3D` convention.
    #[inline]
    pub fn rotate_vector<U: Unit>(&self, v: Vector3<From, U, f32>) -> Vector3<To, U, f32> {
        let q_vec = Vector3::<From, U, f32>::new(self.x, self.y, self.z);
        let uv = q_vec.cross(v);
        let uuv = q_vec.cross(uv);

        let rotated = v - (uv * (2.0 * self.w)) + (uuv * 2.0);
        Vector3::new(rotated.x, rotated.y, rotated.z)
    }

    /// Convert Quaternion to 3x3 Direction Cosine Matrix (DCM).
    #[inline]
    pub fn to_dcm(&self) -> DirectionCosineMatrix<From, To, f32> {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        DirectionCosineMatrix::from_rows(
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)],
            [2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)],
            [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)],
        )
    }
}

impl<From: Frame, To: Frame> Quaternion<From, To, f64> {
    /// Identity rotation quaternion [1, 0, 0, 0] (`f64`).
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        _from: PhantomData,
        _to: PhantomData,
    };

    /// Construct rotation quaternion from ZYX (Yaw, Pitch, Roll) Tait-Bryan Euler angles in radians (`std`).
    #[cfg(feature = "std")]
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        let (cy, sy) = ((yaw * 0.5).cos(), (yaw * 0.5).sin());
        let (cp, sp) = ((pitch * 0.5).cos(), (pitch * 0.5).sin());
        let (cr, sr) = ((roll * 0.5).cos(), (roll * 0.5).sin());

        let w = cr * cp * cy + sr * sp * sy;
        let x = sr * cp * cy - cr * sp * sy;
        let y = cr * sp * cy + sr * cp * sy;
        let z = cr * cp * sy - sr * sp * cy;

        Self::new(w, x, y, z)
    }

    /// Construct rotation quaternion from ZYX Euler angles (`no_std`).
    #[cfg(not(feature = "std"))]
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        let (cy, sy) = (libm::cos(yaw * 0.5), libm::sin(yaw * 0.5));
        let (cp, sp) = (libm::cos(pitch * 0.5), libm::sin(pitch * 0.5));
        let (cr, sr) = (libm::cos(roll * 0.5), libm::sin(roll * 0.5));

        let w = cr * cp * cy + sr * sp * sy;
        let x = sr * cp * cy - cr * sp * sy;
        let y = cr * sp * cy + sr * cp * sy;
        let z = cr * cp * sy - sr * sp * cy;

        Self::new(w, x, y, z)
    }

    /// Rotate a vector from `From` frame to `To` frame using the passive frame-transform
    /// sandwich product $v' = q^* v q$, matching the `DirectionCosineMatrix` / `Transform3D` convention.
    #[inline]
    pub fn rotate_vector<U: Unit>(&self, v: Vector3<From, U, f64>) -> Vector3<To, U, f64> {
        let q_vec = Vector3::<From, U, f64>::new(self.x, self.y, self.z);
        let uv = q_vec.cross(v);
        let uuv = q_vec.cross(uv);

        let rotated = v - (uv * (2.0 * self.w)) + (uuv * 2.0);
        Vector3::new(rotated.x, rotated.y, rotated.z)
    }

    /// Convert Quaternion to 3x3 Direction Cosine Matrix (DCM).
    #[inline]
    pub fn to_dcm(&self) -> DirectionCosineMatrix<From, To, f64> {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        DirectionCosineMatrix::from_rows(
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)],
            [2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)],
            [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)],
        )
    }
}

/// Quaternion composition multiplication: $Q_{A \to B} \times Q_{B \to C} = Q_{A \to C}$.
impl<A: Frame, B: Frame, C: Frame> Mul<Quaternion<B, C, f32>> for Quaternion<A, B, f32> {
    type Output = Quaternion<A, C, f32>;

    #[inline]
    fn mul(self, rhs: Quaternion<B, C, f32>) -> Self::Output {
        let w = self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z;
        let x = self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y;
        let y = self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x;
        let z = self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w;

        Quaternion::new(w, x, y, z)
    }
}

/// Quaternion composition multiplication: $Q_{A \to B} \times Q_{B \to C} = Q_{A \to C}$ (`f64`).
impl<A: Frame, B: Frame, C: Frame> Mul<Quaternion<B, C, f64>> for Quaternion<A, B, f64> {
    type Output = Quaternion<A, C, f64>;

    #[inline]
    fn mul(self, rhs: Quaternion<B, C, f64>) -> Self::Output {
        let w = self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z;
        let x = self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y;
        let y = self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x;
        let z = self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w;

        Quaternion::new(w, x, y, z)
    }
}
