//! Reference Frame markers and traits for `frame-sc`.

use core::fmt::Debug;

/// Marker trait implemented by all reference frame Zero-Sized Types (ZSTs).
pub trait Frame: 'static + Send + Sync + Copy + Eq + Debug {}

/// Body Frame (e.g. Forward-Right-Down / Forward-Left-Up attached to a drone/vehicle/aircraft).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BodyFrame;
impl Frame for BodyFrame {}

/// Local North-East-Down (NED) navigation frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LocalNed;
impl Frame for LocalNed {}

/// Local East-North-Up (ENU) navigation frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LocalEnu;
impl Frame for LocalEnu {}

/// Earth-Centered, Earth-Fixed (ECEF) global reference frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Ecef;
impl Frame for Ecef {}

/// Maritime Target / Vessel reference frame (attached to a moving platform/ship).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct MaritimeTargetFrame;
impl Frame for MaritimeTargetFrame {}

/// Camera / Sensor payload reference frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CameraFrame;
impl Frame for CameraFrame {}

/// Macro to define custom reference frame ZSTs implementing the [`Frame`] trait.
///
/// # Example
/// ```rust
/// use frame_sc::define_frame;
///
/// define_frame!(GimbalFrame, "Camera gimbal payload frame");
/// ```
#[macro_export]
macro_rules! define_frame {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
        pub struct $name;
        impl $crate::frames::Frame for $name {}
    };
    ($name:ident, $doc:expr) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
        pub struct $name;
        impl $crate::frames::Frame for $name {}
    };
}
