//! Typed scalar wrapper for unit safety.

use crate::units::{ConvertUnit, Unit};
use core::marker::PhantomData;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// A scalar magnitude bound to a physical unit `U` (e.g. `Scalar<Meters, f32>`).
#[repr(transparent)]
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(bound(serialize = "T: serde::Serialize", deserialize = "T: serde::de::DeserializeOwned"))
)]
pub struct Scalar<U: Unit, T = f32> {
    pub value: T,
    #[cfg_attr(feature = "serde", serde(skip))]
    _unit: PhantomData<U>,
}

impl<U: Unit, T: Copy> Clone for Scalar<U, T> {
    #[inline(always)]
    fn clone(&self) -> Self {
        *self
    }
}

impl<U: Unit, T: Copy> Copy for Scalar<U, T> {}

impl<U: Unit, T: PartialEq> PartialEq for Scalar<U, T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<U: Unit, T: PartialOrd> PartialOrd for Scalar<U, T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.value.partial_cmp(&other.value)
    }
}

#[cfg(feature = "std")]
impl<U: Unit, T: core::fmt::Display> core::fmt::Display for Scalar<U, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.value)
    }
}

#[cfg(not(feature = "std"))]
impl<U: Unit, T: core::fmt::Display> core::fmt::Display for Scalar<U, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl<U: Unit, T> Scalar<U, T> {
    /// Create a new typed scalar value.
    #[inline(always)]
    pub const fn new(value: T) -> Self {
        Self {
            value,
            _unit: PhantomData,
        }
    }

    /// Extract the raw inner scalar value.
    #[inline(always)]
    pub fn raw(self) -> T {
        self.value
    }
}

impl<U: Unit> Scalar<U, f32> {
    /// Convert scalar value to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Scalar<TargetU, f32>
    where
        U: ConvertUnit<TargetU>,
    {
        Scalar::new(self.value * U::scale_factor_f32())
    }
}

impl<U: Unit> Scalar<U, f64> {
    /// Convert scalar value to target unit.
    #[inline(always)]
    pub fn convert<TargetU: Unit>(self) -> Scalar<TargetU, f64>
    where
        U: ConvertUnit<TargetU>,
    {
        Scalar::new(self.value * U::scale_factor_f64())
    }
}

// Operators for Scalar
impl<U: Unit, T: Add<Output = T>> Add for Scalar<U, T> {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Scalar::new(self.value + rhs.value)
    }
}

impl<U: Unit, T: Sub<Output = T>> Sub for Scalar<U, T> {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Scalar::new(self.value - rhs.value)
    }
}

impl<U: Unit, T: Neg<Output = T>> Neg for Scalar<U, T> {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self::Output {
        Scalar::new(-self.value)
    }
}

impl<U: Unit, T: Mul<Output = T> + Copy> Mul<T> for Scalar<U, T> {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: T) -> Self::Output {
        Scalar::new(self.value * rhs)
    }
}

impl<U: Unit, T: Div<Output = T> + Copy> Div<T> for Scalar<U, T> {
    type Output = Self;
    #[inline(always)]
    fn div(self, rhs: T) -> Self::Output {
        Scalar::new(self.value / rhs)
    }
}

impl<U: Unit, T: AddAssign> AddAssign for Scalar<U, T> {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Self) {
        self.value += rhs.value;
    }
}

impl<U: Unit, T: SubAssign> SubAssign for Scalar<U, T> {
    #[inline(always)]
    fn sub_assign(&mut self, rhs: Self) {
        self.value -= rhs.value;
    }
}

impl<U: Unit, T: MulAssign + Copy> MulAssign<T> for Scalar<U, T> {
    #[inline(always)]
    fn mul_assign(&mut self, rhs: T) {
        self.value *= rhs;
    }
}

impl<U: Unit, T: DivAssign + Copy> DivAssign<T> for Scalar<U, T> {
    #[inline(always)]
    fn div_assign(&mut self, rhs: T) {
        self.value /= rhs;
    }
}
