//! Error types for fallible constructors.

use core::fmt;

/// Error returned by fallible constructors when input fails validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValidationError {
    /// One or more components were not finite (NaN or infinity).
    NonFinite,
    /// WGS-84 latitude is outside the valid range `[-90, 90]` degrees.
    LatitudeOutOfRange,
    /// WGS-84 longitude is outside the valid range `[-180, 180]` degrees.
    LongitudeOutOfRange,
    /// A quaternion does not have unit norm.
    QuaternionNotUnit,
    /// A direction cosine matrix is not orthonormal.
    DcmNotOrthonormal,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msg = match self {
            ValidationError::NonFinite => "value is not finite",
            ValidationError::LatitudeOutOfRange => "latitude out of range [-90, 90] degrees",
            ValidationError::LongitudeOutOfRange => "longitude out of range [-180, 180] degrees",
            ValidationError::QuaternionNotUnit => "quaternion is not unit norm",
            ValidationError::DcmNotOrthonormal => "matrix is not orthonormal",
        };
        f.write_str(msg)
    }
}

impl core::error::Error for ValidationError {}
