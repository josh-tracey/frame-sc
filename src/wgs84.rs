//! WGS-84 Geodetic coordinate (Latitude, Longitude, Altitude) definitions
//! and conversion utilities to/from ECEF and Local NED frames.

use crate::error::ValidationError;
use crate::frames::{Ecef, LocalNed};
use crate::point::Point3;
use crate::units::Meters;
use crate::vector::Vector3;
use core::fmt::Debug;

/// WGS-84 Ellipsoid constants
pub const WGS84_A: f64 = 6_378_137.0; // Semi-major axis (meters)
pub const WGS84_F: f64 = 1.0 / 298.257_223_563; // Flattening
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F); // Semi-minor axis (~6356752.3142 m)
pub const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // First eccentricity squared (~0.00669437999)
pub const WGS84_EP2: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B); // Second eccentricity squared

/// WGS-84 Geodetic Position (Latitude, Longitude, Altitude above ellipsoid).
///
/// Invariants (enforced by `Wgs84Position::try_new` and serde deserialization):
/// `lat_deg ∈ [-90, 90]`, `lon_deg ∈ [-180, 180]`, and all fields finite.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(into = "Wgs84PositionSerde", try_from = "Wgs84PositionSerde")
)]
pub struct Wgs84Position {
    /// Latitude in degrees (-90.0 to +90.0)
    pub lat_deg: f64,
    /// Longitude in degrees (-180.0 to +180.0)
    pub lon_deg: f64,
    /// Altitude above WGS-84 ellipsoid in meters
    pub alt_m: f64,
}

impl Wgs84Position {
    /// Create a new WGS-84 Geodetic Position.
    ///
    /// This const constructor is unchecked; use `try_new` for validated input.
    #[inline(always)]
    pub const fn new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        Self {
            lat_deg,
            lon_deg,
            alt_m,
        }
    }

    /// Fallible constructor: validates latitude, longitude, and altitude.
    pub fn try_new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Result<Self, ValidationError> {
        if !(lat_deg.is_finite() && lon_deg.is_finite() && alt_m.is_finite()) {
            return Err(ValidationError::NonFinite);
        }
        if !(-90.0..=90.0).contains(&lat_deg) {
            return Err(ValidationError::LatitudeOutOfRange);
        }
        if !(-180.0..=180.0).contains(&lon_deg) {
            return Err(ValidationError::LongitudeOutOfRange);
        }
        Ok(Self {
            lat_deg,
            lon_deg,
            alt_m,
        })
    }

    /// Convert WGS-84 Geodetic (Lat, Lon, Alt) to Cartesian 3D ECEF Point.
    #[inline]
    pub fn to_ecef(&self) -> Point3<Ecef, Meters, f64> {
        let lat_rad = self.lat_deg * core::f64::consts::PI / 180.0;
        let lon_rad = self.lon_deg * core::f64::consts::PI / 180.0;

        let (sin_lat, cos_lat) = (libm::sin(lat_rad), libm::cos(lat_rad));
        let (sin_lon, cos_lon) = (libm::sin(lon_rad), libm::cos(lon_rad));

        let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

        let x = (n + self.alt_m) * cos_lat * cos_lon;
        let y = (n + self.alt_m) * cos_lat * sin_lon;
        let z = (n * (1.0 - WGS84_E2) + self.alt_m) * sin_lat;

        Point3::new(x, y, z)
    }

    /// Convert WGS-84 Geodetic position to Local NED relative to a home reference origin.
    #[inline(always)]
    pub fn to_local_ned(&self, home: &Wgs84Position) -> Point3<LocalNed, Meters, f64> {
        let ecef_self = self.to_ecef();
        let ecef_home = home.to_ecef();
        let delta_ecef = ecef_self - ecef_home;

        delta_ecef.ecef_to_local_ned(home.lat_deg, home.lon_deg)
    }

    /// Construct a WGS-84 Position from Local NED displacement relative to a home reference origin.
    #[inline(always)]
    pub fn from_local_ned(ned: Point3<LocalNed, Meters, f64>, home: &Wgs84Position) -> Self {
        let delta_ecef = ned
            .to_vector()
            .local_ned_to_ecef(home.lat_deg, home.lon_deg);
        let ecef_point = home.to_ecef() + delta_ecef;
        ecef_point.to_wgs84()
    }
}

#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct Wgs84PositionSerde {
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
}

#[cfg(feature = "serde")]
impl From<Wgs84Position> for Wgs84PositionSerde {
    #[inline]
    fn from(p: Wgs84Position) -> Self {
        Wgs84PositionSerde {
            lat_deg: p.lat_deg,
            lon_deg: p.lon_deg,
            alt_m: p.alt_m,
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<Wgs84PositionSerde> for Wgs84Position {
    type Error = ValidationError;

    #[inline]
    fn try_from(s: Wgs84PositionSerde) -> Result<Self, Self::Error> {
        Wgs84Position::try_new(s.lat_deg, s.lon_deg, s.alt_m)
    }
}

// Convert ECEF Point to WGS84 Position (Bowring's closed-form algorithm)
impl Point3<Ecef, Meters, f64> {
    /// Convert 3D ECEF Cartesian point to WGS-84 Geodetic (Lat, Lon, Alt).
    #[inline]
    pub fn to_wgs84(&self) -> Wgs84Position {
        let p = libm::sqrt(self.x * self.x + self.y * self.y);
        if p < 1e-6 {
            let lat_deg = if self.z >= 0.0 { 90.0 } else { -90.0 };
            return Wgs84Position::new(lat_deg, 0.0, libm::fabs(self.z) - WGS84_B);
        }

        let theta = libm::atan2(self.z * WGS84_A, p * WGS84_B);
        let sin_th = libm::sin(theta);
        let cos_th = libm::cos(theta);

        let lat_rad = libm::atan2(
            self.z + WGS84_EP2 * WGS84_B * sin_th * sin_th * sin_th,
            p - WGS84_E2 * WGS84_A * cos_th * cos_th * cos_th,
        );
        let lon_rad = libm::atan2(self.y, self.x);

        let sin_lat = libm::sin(lat_rad);
        let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        let alt_m = p / libm::cos(lat_rad) - n;

        Wgs84Position::new(
            lat_rad * 180.0 / core::f64::consts::PI,
            lon_rad * 180.0 / core::f64::consts::PI,
            alt_m,
        )
    }
}

// Vector ECEF <-> Local NED rotation helper based on reference lat/lon
impl Vector3<Ecef, Meters, f64> {
    /// Rotate displacement vector from ECEF to Local NED tangent plane at given reference lat/lon.
    #[inline]
    pub fn ecef_to_local_ned(
        &self,
        ref_lat_deg: f64,
        ref_lon_deg: f64,
    ) -> Point3<LocalNed, Meters, f64> {
        let lat_rad = ref_lat_deg * core::f64::consts::PI / 180.0;
        let lon_rad = ref_lon_deg * core::f64::consts::PI / 180.0;

        let (s_lat, c_lat) = (libm::sin(lat_rad), libm::cos(lat_rad));
        let (s_lon, c_lon) = (libm::sin(lon_rad), libm::cos(lon_rad));

        let north = -s_lat * c_lon * self.x - s_lat * s_lon * self.y + c_lat * self.z;
        let east = -s_lon * self.x + c_lon * self.y;
        let down = -c_lat * c_lon * self.x - c_lat * s_lon * self.y - s_lat * self.z;

        Point3::new(north, east, down)
    }
}

impl Vector3<LocalNed, Meters, f64> {
    /// Rotate displacement vector from Local NED to ECEF frame at given reference lat/lon.
    #[inline]
    pub fn local_ned_to_ecef(
        &self,
        ref_lat_deg: f64,
        ref_lon_deg: f64,
    ) -> Vector3<Ecef, Meters, f64> {
        let lat_rad = ref_lat_deg * core::f64::consts::PI / 180.0;
        let lon_rad = ref_lon_deg * core::f64::consts::PI / 180.0;

        let (s_lat, c_lat) = (libm::sin(lat_rad), libm::cos(lat_rad));
        let (s_lon, c_lon) = (libm::sin(lon_rad), libm::cos(lon_rad));

        let dx = -s_lat * c_lon * self.x - s_lon * self.y - c_lat * c_lon * self.z;
        let dy = -s_lat * s_lon * self.x + c_lon * self.y - c_lat * s_lon * self.z;
        let dz = c_lat * self.x - s_lat * self.z;

        Vector3::new(dx, dy, dz)
    }
}
