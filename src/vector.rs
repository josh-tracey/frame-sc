//! Type-safe 3D spatial vector implementation.

use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::frames::Frame;
use crate::scalar::Scalar;
use crate::units::{ConvertUnit, Unit};

/// A 3D spatial vector bound to reference frame `F` and physical unit `U`.
///
/// Guaranteed to have identical `#[repr(C)]` layout to `[T; 3]` or a C struct of 3 elements `T`.
#[repr(C)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(serialize = "T: serde::Serialize", deserialize = "T: serde::de::DeserializeOwned"))
)]
pub struct Vector3<F: Frame, U: Unit, T = f32> {
    pub x: T,
    pub y: T,
    pub z: T,
    #[cfg_attr(feature = "serde", serde(skip))]
    _frame: PhantomData<F>,
    #[cfg_attr(feature = "serde", serde(skip))]
    _unit: PhantomData<U>,
}

impl<F: Frame, U: Unit, T: Copy> Copy for Vector3<F, U, T> {}

impl<F: Frame, U: Unit, T: Copy> Clone for Vector3<F, U, T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame, U: Unit, T: PartialEq> PartialEq for Vector3<F, U, T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }
}

impl<F: Frame, U: Unit, T: Debug> Debug for Vector3<F, U, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Vector3")
            .field("frame", &core::any::type_name::<F>())
            .field("unit", &core::any::type_name::<U>())
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
    }
}

impl<F: Frame, U: Unit, T> Vector3<F, U, T> {
    /// Create a new 3D vector.
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self {
            x,
            y,
            z,
            _frame: PhantomData,
            _unit: PhantomData,
        }
    }

    /// Create a zero vector.
    #[inline(always)]
    pub fn zero() -> Self
    where
        T: Default,
    {
        Self::new(T::default(), T::default(), T::default())
    }

    /// Alias for zero vector.
    #[inline(always)]
    pub fn zeros() -> Self
    where
        T: Default,
    {
        Self::zero()
    }

    /// Create vector from a raw 3-element array `[x, y, z]`.
    #[inline(always)]
    pub const fn from_array(arr: [T; 3]) -> Self
    where
        T: Copy,
    {
        Self::new(arr[0], arr[1], arr[2])
    }

    /// Convert vector components into a raw array `[x, y, z]`.
    #[inline(always)]
    pub fn as_array(self) -> [T; 3]
    where
        T: Copy,
    {
        [self.x, self.y, self.z]
    }
}

impl<F: Frame, U: Unit, T: Default> Default for Vector3<F, U, T> {
    #[inline(always)]
    fn default() -> Self {
        Self::new(T::default(), T::default(), T::default())
    }
}

impl<F: Frame, U: Unit> Vector3<F, U, f32> {
    /// Zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Convert vector components to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Vector3<F, TargetU, f32>
    where
        U: ConvertUnit<TargetU>,
    {
        let s = U::scale_factor_f32();
        Vector3::new(self.x * s, self.y * s, self.z * s)
    }

    /// Compute dot product with another vector in the same frame and unit.
    #[inline(always)]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Compute cross product with another vector in the same frame and unit.
    #[inline(always)]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Squared Euclidean norm / magnitude.
    #[inline(always)]
    pub fn norm_sq(self) -> f32 {
        self.dot(self)
    }

    /// Euclidean norm / magnitude.
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn norm(self) -> Scalar<U, f32> {
        Scalar::new(self.norm_sq().sqrt())
    }

    /// Euclidean norm / magnitude (`no_std` using `libm`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn norm(self) -> Scalar<U, f32> {
        Scalar::new(libm::sqrtf(self.norm_sq()))
    }

    /// Normalize vector to unit length (returns zero vector if norm is 0).
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn normalize(self) -> Self {
        let n = self.norm_sq().sqrt();
        if n > 1e-12 {
            self / n
        } else {
            Self::ZERO
        }
    }

    /// Normalize vector to unit length (`no_std` using `libm`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn normalize(self) -> Self {
        let n = libm::sqrtf(self.norm_sq());
        if n > 1e-12 {
            self / n
        } else {
            Self::ZERO
        }
    }
}

impl<F: Frame, U: Unit> Vector3<F, U, f64> {
    /// Zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Convert vector components to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Vector3<F, TargetU, f64>
    where
        U: ConvertUnit<TargetU>,
    {
        let s = U::scale_factor_f64();
        Vector3::new(self.x * s, self.y * s, self.z * s)
    }

    /// Compute dot product with another vector in the same frame and unit.
    #[inline(always)]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Compute cross product with another vector in the same frame and unit.
    #[inline(always)]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Squared Euclidean norm / magnitude.
    #[inline(always)]
    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    /// Euclidean norm / magnitude.
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn norm(self) -> Scalar<U, f64> {
        Scalar::new(self.norm_sq().sqrt())
    }

    /// Euclidean norm / magnitude (`no_std` using `libm`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn norm(self) -> Scalar<U, f64> {
        Scalar::new(libm::sqrt(self.norm_sq()))
    }

    /// Normalize vector to unit length (returns zero vector if norm is 0).
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn normalize(self) -> Self {
        let n = self.norm_sq().sqrt();
        if n > 1e-15 {
            self / n
        } else {
            Self::ZERO
        }
    }

    /// Normalize vector to unit length (`no_std` using `libm`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn normalize(self) -> Self {
        let n = libm::sqrt(self.norm_sq());
        if n > 1e-15 {
            self / n
        } else {
            Self::ZERO
        }
    }
}

// Operators
impl<F: Frame, U: Unit, T: Add<Output = T>> Add for Vector3<F, U, T> {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<F: Frame, U: Unit, T: Sub<Output = T>> Sub for Vector3<F, U, T> {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl<F: Frame, U: Unit, T: Neg<Output = T>> Neg for Vector3<F, U, T> {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl<F: Frame, U: Unit, T: Mul<Output = T> + Copy> Mul<T> for Vector3<F, U, T> {
    type Output = Self;
    #[inline(always)]
    fn mul(self, scalar: T) -> Self::Output {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl<F: Frame, U: Unit, T: Div<Output = T> + Copy> Div<T> for Vector3<F, U, T> {
    type Output = Self;
    #[inline(always)]
    fn div(self, scalar: T) -> Self::Output {
        Self::new(self.x / scalar, self.y / scalar, self.z / scalar)
    }
}

impl<F: Frame, U: Unit, T: AddAssign> AddAssign for Vector3<F, U, T> {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<F: Frame, U: Unit, T: SubAssign> SubAssign for Vector3<F, U, T> {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl<F: Frame, U: Unit, T: MulAssign + Copy> MulAssign<T> for Vector3<F, U, T> {
    #[inline(always)]
    fn mul_assign(&mut self, scalar: T) {
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

impl<F: Frame, U: Unit, T: DivAssign + Copy> DivAssign<T> for Vector3<F, U, T> {
    #[inline(always)]
    fn div_assign(&mut self, scalar: T) {
        self.x /= scalar;
        self.y /= scalar;
        self.z /= scalar;
    }
}
