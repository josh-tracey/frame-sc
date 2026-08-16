//! Helper macros for `frame-sc`.

/// Macro to create a typed spatial vector with explicit frame and unit parameters.
///
/// # Example
/// ```rust
/// use frame_sc::{vec3, BodyFrame, Meters, Vector3};
///
/// let v: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 1.0, 2.0, 3.0];
/// ```
#[macro_export]
macro_rules! vec3 {
    ($frame:ty, $unit:ty, $x:expr, $y:expr, $z:expr) => {
        $crate::vector::Vector3::<$frame, $unit, _>::new($x, $y, $z)
    };
}

/// Macro to create a typed spatial point with explicit frame and unit parameters.
///
/// # Example
/// ```rust
/// use frame_sc::{pt3, LocalNed, Meters, Point3};
///
/// let p: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 10.0, -5.0, 0.0];
/// ```
#[macro_export]
macro_rules! pt3 {
    ($frame:ty, $unit:ty, $x:expr, $y:expr, $z:expr) => {
        $crate::point::Point3::<$frame, $unit, _>::new($x, $y, $z)
    };
}
