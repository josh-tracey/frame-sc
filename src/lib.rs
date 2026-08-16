//! # frame-sc: Zero-Cost Spatial Typestates
//!
//! `frame-sc` provides static analysis and type safety for spatial calculations by encoding
//! **Reference Frames** (e.g. Body, Local NED, ECEF, Ship) and **Physical Units** (e.g. Meters, Millimeters)
//! directly into type signatures using Zero-Sized Types (ZSTs) and `PhantomData`.
//!
//! The Rust compiler acts as a static physics engine, preventing cross-frame vector operations,
//! invalid affine geometry (e.g. adding point + point), or mismatched physical units without explicit
//! transformation matrices or scale conversions.
//!
//! All tags disappear entirely at compile time, leaving pure raw scalar floating-point math (`f32`/`f64`)
//! with **zero runtime overhead**.
//!
//! ## Example Usage
//! ```rust
//! use frame_sc::{
//!     BodyFrame, LocalNed, MetersPerSecond, Vector3, DirectionCosineMatrix,
//!     vec3
//! };
//!
//! // Drone velocity in aircraft Body Frame
//! let drone_velocity: Vector3<BodyFrame, MetersPerSecond> = vec3![BodyFrame, MetersPerSecond, 15.0, 0.0, 0.0];
//!
//! // Wind velocity vector in Local NED Frame
//! let wind_vector: Vector3<LocalNed, MetersPerSecond> = vec3![LocalNed, MetersPerSecond, 2.0, -3.0, 0.0];
//!
//! // Direction Cosine Matrix from Local NED to Body Frame (e.g. pitch angle)
//! let ned_to_body = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(0.0, 0.1, 0.0);
//!
//! // COMPILER ERROR if you try to add without rotating:
//! // let err = drone_velocity + wind_vector;
//!
//! // COMPILES SAFELY:
//! let total_effective_velocity = drone_velocity + wind_vector.rotate_to(&ned_to_body);
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod dcm;
pub mod frames;
pub mod macros;
pub mod point;
pub mod pose;
pub mod quaternion;
pub mod scalar;
pub mod transform;
pub mod units;
pub mod vector;
pub mod wgs84;

// Top-level re-exports
pub use dcm::DirectionCosineMatrix;
pub use frames::{BodyFrame, CameraFrame, Ecef, Frame, LocalEnu, LocalNed, MaritimeTargetFrame};
pub use point::Point3;
pub use pose::Pose3D;
pub use quaternion::Quaternion;
pub use scalar::Scalar;
pub use transform::Transform3D;
pub use units::{
    ConvertUnit, Degrees, Feet, Kilometers, Knots, Meters, MetersPerSecond, Millimeters,
    NauticalMiles, Radians, Unit,
};
pub use vector::Vector3;
pub use wgs84::Wgs84Position;
