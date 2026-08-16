//! Type-safe 3D spatial point (position in affine space).

use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::{Add, AddAssign, Sub, SubAssign};

use crate::frames::Frame;
use crate::scalar::Scalar;
use crate::units::{ConvertUnit, Unit};
use crate::vector::Vector3;

/// A 3D spatial point (position) bound to reference frame `F` and physical unit `U`.
///
/// Follows strict affine space mathematics:
/// - `Point - Point = Vector`
/// - `Point + Vector = Point`
/// - `Point - Vector = Point`
/// - `Point + Point` **does not compile**
#[repr(C)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(serialize = "T: serde::Serialize", deserialize = "T: serde::de::DeserializeOwned"))
)]
pub struct Point3<F: Frame, U: Unit, T = f32> {
    pub x: T,
    pub y: T,
    pub z: T,
    #[cfg_attr(feature = "serde", serde(skip))]
    _frame: PhantomData<F>,
    #[cfg_attr(feature = "serde", serde(skip))]
    _unit: PhantomData<U>,
}

impl<F: Frame, U: Unit, T: Copy> Copy for Point3<F, U, T> {}

impl<F: Frame, U: Unit, T: Copy> Clone for Point3<F, U, T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        *self
    }
}

impl<F: Frame, U: Unit, T: PartialEq> PartialEq for Point3<F, U, T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }
}

impl<F: Frame, U: Unit, T: Debug> Debug for Point3<F, U, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Point3")
            .field("frame", &core::any::type_name::<F>())
            .field("unit", &core::any::type_name::<U>())
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
    }
}

impl<F: Frame, U: Unit, T> Point3<F, U, T> {
    /// Create a new 3D spatial point.
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

    /// Create point at origin (0, 0, 0).
    #[inline(always)]
    pub fn origin() -> Self
    where
        T: Default,
    {
        Self::new(T::default(), T::default(), T::default())
    }

    /// Alias for origin point.
    #[inline(always)]
    pub fn zero() -> Self
    where
        T: Default,
    {
        Self::origin()
    }

    /// Create point from a 3-element array `[x, y, z]`.
    #[inline(always)]
    pub const fn from_array(arr: [T; 3]) -> Self
    where
        T: Copy,
    {
        Self::new(arr[0], arr[1], arr[2])
    }

    /// Convert point coordinates into a raw array `[x, y, z]`.
    #[inline(always)]
    pub fn as_array(self) -> [T; 3]
    where
        T: Copy,
    {
        [self.x, self.y, self.z]
    }

    /// Convert displacement from origin into a displacement vector in the same frame and unit.
    #[inline(always)]
    pub fn to_vector(self) -> Vector3<F, U, T> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl<F: Frame, U: Unit, T: Default> Default for Point3<F, U, T> {
    #[inline(always)]
    fn default() -> Self {
        Self::origin()
    }
}

impl<F: Frame, U: Unit> Point3<F, U, f32> {
    /// Convert point coordinates to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Point3<F, TargetU, f32>
    where
        U: ConvertUnit<TargetU>,
    {
        let s = U::scale_factor_f32();
        Point3::new(self.x * s, self.y * s, self.z * s)
    }

    /// Distance to another point in the same frame and unit.
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn distance_to(self, other: Self) -> Scalar<U, f32> {
        (self - other).norm()
    }

    /// Distance to another point in the same frame and unit (`no_std`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn distance_to(self, other: Self) -> Scalar<U, f32> {
        (self - other).norm()
    }
}

impl<F: Frame, U: Unit> Point3<F, U, f64> {
    /// Convert point coordinates to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Point3<F, TargetU, f64>
    where
        U: ConvertUnit<TargetU>,
    {
        let s = U::scale_factor_f64();
        Point3::new(self.x * s, self.y * s, self.z * s)
    }

    /// Distance to another point in the same frame and unit.
    #[inline(always)]
    #[cfg(feature = "std")]
    pub fn distance_to(self, other: Self) -> Scalar<U, f64> {
        (self - other).norm()
    }

    /// Distance to another point in the same frame and unit (`no_std`).
    #[inline(always)]
    #[cfg(not(feature = "std"))]
    pub fn distance_to(self, other: Self) -> Scalar<U, f64> {
        (self - other).norm()
    }
}

// Affine Operators:
// Point - Point = Vector
impl<F: Frame, U: Unit, T: Sub<Output = T>> Sub<Point3<F, U, T>> for Point3<F, U, T> {
    type Output = Vector3<F, U, T>;
    #[inline(always)]
    fn sub(self, rhs: Point3<F, U, T>) -> Self::Output {
        Vector3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

// Point + Vector = Point
impl<F: Frame, U: Unit, T: Add<Output = T>> Add<Vector3<F, U, T>> for Point3<F, U, T> {
    type Output = Point3<F, U, T>;
    #[inline(always)]
    fn add(self, rhs: Vector3<F, U, T>) -> Self::Output {
        Point3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

// Point - Vector = Point
impl<F: Frame, U: Unit, T: Sub<Output = T>> Sub<Vector3<F, U, T>> for Point3<F, U, T> {
    type Output = Point3<F, U, T>;
    #[inline(always)]
    fn sub(self, rhs: Vector3<F, U, T>) -> Self::Output {
        Point3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

// Point += Vector
impl<F: Frame, U: Unit, T: AddAssign> AddAssign<Vector3<F, U, T>> for Point3<F, U, T> {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Vector3<F, U, T>) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

// Point -= Vector
impl<F: Frame, U: Unit, T: SubAssign> SubAssign<Vector3<F, U, T>> for Point3<F, U, T> {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Vector3<F, U, T>) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}
