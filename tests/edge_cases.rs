//! Deterministic edge-case, NaN/Inf, and validation policy tests.

use frame_sc::{
    pt3, vec3, BodyFrame, DirectionCosineMatrix, LocalNed, Meters, Point3, Quaternion,
    ValidationError, Vector3, Wgs84Position,
};

type V3f = Vector3<BodyFrame, Meters, f32>;
type P3f = Point3<LocalNed, Meters, f32>;
type Qf = Quaternion<LocalNed, BodyFrame, f32>;
type DCMf = DirectionCosineMatrix<LocalNed, BodyFrame, f32>;

#[test]
fn vector_normalize_degenerate() {
    let zero = V3f::ZERO;
    assert_eq!(zero.normalize(), V3f::ZERO);
    assert_eq!(zero.try_normalize(), None);

    let nan = V3f::new(f32::NAN, 0.0, 0.0);
    assert_eq!(nan.normalize(), V3f::ZERO);
    assert_eq!(nan.try_normalize(), None);

    // Inf must NOT collapse to NaN.
    let inf = V3f::new(f32::INFINITY, 0.0, 0.0);
    assert_eq!(inf.normalize(), V3f::ZERO);
    assert_eq!(inf.try_normalize(), None);
}

#[test]
fn vector_point_is_finite() {
    let v: V3f = vec3![BodyFrame, Meters, 1.0, 2.0, 3.0];
    assert!(v.is_finite());
    assert!(!V3f::new(f32::NAN, 0.0, 0.0).is_finite());
    assert!(!V3f::new(f32::INFINITY, 0.0, 0.0).is_finite());

    let p: P3f = pt3![LocalNed, Meters, 1.0, 2.0, 3.0];
    assert!(p.is_finite());
    assert!(!P3f::new(f32::NAN, 0.0, 0.0).is_finite());
}

#[test]
fn quaternion_normalize_and_try_new() {
    // 2.0-scaled identity normalizes to identity.
    let q = Qf::new(2.0, 0.0, 0.0, 0.0);
    assert_eq!(q.normalize(), Qf::IDENTITY);
    assert!(q.normalize().is_normalized());

    // Zero quaternion -> identity.
    let z = Qf::new(0.0, 0.0, 0.0, 0.0);
    assert_eq!(z.normalize(), Qf::IDENTITY);

    // try_new: valid, non-unit, and non-finite.
    assert_eq!(Qf::try_new(1.0, 0.0, 0.0, 0.0).unwrap(), Qf::IDENTITY);
    assert_eq!(
        Qf::try_new(2.0, 0.0, 0.0, 0.0).unwrap_err(),
        ValidationError::QuaternionNotUnit
    );
    assert_eq!(
        Qf::try_new(f32::NAN, 0.0, 0.0, 0.0).unwrap_err(),
        ValidationError::NonFinite
    );
}

#[test]
fn dcm_is_orthonormal() {
    assert!(DCMf::IDENTITY.is_orthonormal());
    let dcm = DCMf::from_euler_zyx(0.3, -0.2, 0.5);
    assert!(dcm.is_orthonormal());

    let bad = DCMf::new([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]]);
    assert!(!bad.is_orthonormal());
    assert!(DCMf::try_new_orthonormal(bad.m).is_err());

    let nan = DCMf::new([[f32::NAN, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    assert!(!nan.is_orthonormal());
}

#[test]
fn wgs84_try_new_validation() {
    assert!(Wgs84Position::try_new(0.0, 0.0, 0.0).is_ok());
    assert_eq!(
        Wgs84Position::try_new(91.0, 0.0, 0.0).unwrap_err(),
        ValidationError::LatitudeOutOfRange
    );
    assert_eq!(
        Wgs84Position::try_new(-91.0, 0.0, 0.0).unwrap_err(),
        ValidationError::LatitudeOutOfRange
    );
    assert_eq!(
        Wgs84Position::try_new(0.0, 181.0, 0.0).unwrap_err(),
        ValidationError::LongitudeOutOfRange
    );
    assert_eq!(
        Wgs84Position::try_new(0.0, 0.0, f64::NAN).unwrap_err(),
        ValidationError::NonFinite
    );
}

#[test]
fn wgs84_poles_and_antimeridian() {
    for (lat, lon) in [
        (90.0, 0.0),
        (-90.0, 0.0),
        (0.0, 180.0),
        (0.0, -180.0),
        (89.9999, 179.9999),
        (-89.9999, -179.9999),
    ] {
        let w = Wgs84Position::new(lat, lon, 100.0);
        let r = w.to_ecef().to_wgs84();
        assert!((r.lat_deg - lat).abs() < 1e-7, "lat {lat} -> {}", r.lat_deg);
        let mut dlon = (r.lon_deg - lon).abs();
        if dlon > 180.0 {
            dlon = 360.0 - dlon;
        }
        assert!(dlon < 1e-7, "lon {lon} -> {}", r.lon_deg);
    }
}
