//! Physical Unit markers and conversion traits for `frame-sc`.

use core::fmt::Debug;

/// Marker trait implemented by all physical unit Zero-Sized Types (ZSTs).
pub trait Unit: 'static + Send + Sync + Copy + Eq + Debug {}

/// Trait for explicit linear conversion factor between two units of the same physical dimension.
pub trait ConvertUnit<TargetUnit: Unit> {
    /// Returns the scale factor `S` such that `val_in_target = val_in_self * S`.
    fn scale_factor_f32() -> f32;
    /// Returns the scale factor `S` as `f64`.
    fn scale_factor_f64() -> f64;
}

/// Unit: Meters (SI unit of length)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Meters;
impl Unit for Meters {}

/// Unit: Millimeters
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Millimeters;
impl Unit for Millimeters {}

/// Unit: Kilometers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Kilometers;
impl Unit for Kilometers {}

/// Unit: Feet
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Feet;
impl Unit for Feet {}

/// Unit: Nautical Miles
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct NauticalMiles;
impl Unit for NauticalMiles {}

/// Unit: Radians (SI unit of angle)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Radians;
impl Unit for Radians {}

/// Unit: Degrees
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Degrees;
impl Unit for Degrees {}

/// Unit: Meters per second (SI unit of velocity)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct MetersPerSecond;
impl Unit for MetersPerSecond {}

/// Unit: Knots
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Knots;
impl Unit for Knots {}

// Identity conversions
impl<U: Unit> ConvertUnit<U> for U {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        1.0
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        1.0
    }
}

// Length conversions
impl ConvertUnit<Millimeters> for Meters {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        1000.0
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        1000.0
    }
}

impl ConvertUnit<Meters> for Millimeters {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        0.001
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        0.001
    }
}

impl ConvertUnit<Kilometers> for Meters {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        0.001
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        0.001
    }
}

impl ConvertUnit<Meters> for Kilometers {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        1000.0
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        1000.0
    }
}

impl ConvertUnit<Feet> for Meters {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        3.280_84
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        3.280839895013123
    }
}

impl ConvertUnit<Meters> for Feet {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        0.3048
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        0.3048
    }
}

// Velocity conversions
impl ConvertUnit<Knots> for MetersPerSecond {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        1.943_844_5
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        1.9438444924406
    }
}

impl ConvertUnit<MetersPerSecond> for Knots {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        0.514_444_4
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        0.51444444444444
    }
}

// Angular conversions
impl ConvertUnit<Degrees> for Radians {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        57.29578
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        57.29577951308232
    }
}

impl ConvertUnit<Radians> for Degrees {
    #[inline(always)]
    fn scale_factor_f32() -> f32 {
        0.017453292
    }
    #[inline(always)]
    fn scale_factor_f64() -> f64 {
        0.017453292519943295
    }
}

/// Macro to define custom physical unit ZSTs implementing the [`Unit`] trait.
#[macro_export]
macro_rules! define_unit {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
        pub struct $name;
        impl $crate::units::Unit for $name {}
    };
    ($name:ident, $doc:expr) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
        pub struct $name;
        impl $crate::units::Unit for $name {}
    };
}
