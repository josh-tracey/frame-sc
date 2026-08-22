//! Property-based (proptest) round-trip and invariant checks.
//!
//! These run under the std test harness regardless of the library's feature set,
//! and exercise the deterministic `libm` math paths.

use frame_sc::{
    pt3, vec3, BodyFrame, DirectionCosineMatrix, Ecef, Feet, LocalNed, MaritimeTargetFrame, Meters,
    Point3, Quaternion, Transform3D, Vector3, Wgs84Position,
};
use proptest::prelude::*;

const PI: f64 = core::f64::consts::PI;

/// Deterministic finite `f64` in `[lo, hi]`.
fn range(lo: f64, hi: f64) -> impl Strategy<Value = f64> {
    (0u64..=1_000_000).prop_map(move |n| lo + (hi - lo) * (n as f64) / 1_000_000.0)
}

proptest! {
    #[test]
    fn ecef_wgs84_roundtrip(
        lat in range(-89.9, 89.9),
        lon in range(-180.0, 180.0),
        alt in range(-500.0, 20_000.0),
    ) {
        let w = Wgs84Position::new(lat, lon, alt);
        let r = w.to_ecef().to_wgs84();

        let dlat = (r.lat_deg - lat).abs();
        let mut dlon = (r.lon_deg - lon).abs();
        if dlon > 180.0 { dlon = 360.0 - dlon; }

        prop_assert!(dlat < 1e-8, "lat err {dlat} for ({lat},{lon},{alt})");
        prop_assert!(dlon < 1e-8, "lon err {dlon} for ({lat},{lon},{alt})");
        prop_assert!((r.alt_m - alt).abs() < 1e-3, "alt err for ({lat},{lon},{alt})");
    }

    #[test]
    fn quaternion_dcm_consistency_f32(
        yaw in range(-PI, PI), pitch in range(-PI, PI), roll in range(-PI, PI),
    ) {
        let (yaw, pitch, roll) = (yaw as f32, pitch as f32, roll as f32);
        let q = Quaternion::<LocalNed, BodyFrame, f32>::from_euler_zyx(yaw, pitch, roll);
        let dcm = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(yaw, pitch, roll);
        let v: Vector3<LocalNed, Meters, f32> = vec3![LocalNed, Meters, 1.0, -2.0, 3.0];

        let via_q = q.rotate_vector(v);
        let via_dcm = dcm.rotate_vector(v);

        prop_assert!((via_q.x - via_dcm.x).abs() < 1e-3);
        prop_assert!((via_q.y - via_dcm.y).abs() < 1e-3);
        prop_assert!((via_q.z - via_dcm.z).abs() < 1e-3);
        prop_assert!(q.is_normalized());
        prop_assert!(dcm.is_orthonormal());
    }

    #[test]
    fn quaternion_dcm_consistency_f64(
        yaw in range(-PI, PI), pitch in range(-PI, PI), roll in range(-PI, PI),
    ) {
        let q = Quaternion::<LocalNed, BodyFrame, f64>::from_euler_zyx(yaw, pitch, roll);
        let dcm = DirectionCosineMatrix::<LocalNed, BodyFrame, f64>::from_euler_zyx(yaw, pitch, roll);
        let v: Vector3<LocalNed, Meters, f64> = vec3![LocalNed, Meters, 1.0, -2.0, 3.0];

        let via_q = q.rotate_vector(v);
        let via_dcm = dcm.rotate_vector(v);

        prop_assert!((via_q.x - via_dcm.x).abs() < 1e-9);
        prop_assert!((via_q.y - via_dcm.y).abs() < 1e-9);
        prop_assert!((via_q.z - via_dcm.z).abs() < 1e-9);
        prop_assert!(q.is_normalized());
        prop_assert!(dcm.is_orthonormal());
    }

    #[test]
    fn quaternion_normalize_is_unit_f32(
        w in range(-10.0, 10.0), x in range(-10.0, 10.0),
        y in range(-10.0, 10.0), z in range(-10.0, 10.0),
    ) {
        let (w, x, y, z) = (w as f32, x as f32, y as f32, z as f32);
        // Skip the all-zero degenerate case (normalize -> identity by design).
        let q = Quaternion::<LocalNed, BodyFrame, f32>::new(w, x, y, z);
        if q.norm_sq() < 1e-8 {
            return Ok(());
        }
        let qn = q.normalize();
        prop_assert!(qn.is_normalized());
    }

    #[test]
    fn transform_inverse_roundtrip_f32(
        yaw in range(-PI, PI), pitch in range(-PI, PI), roll in range(-PI, PI),
        tx in range(-1000.0, 1000.0), ty in range(-1000.0, 1000.0), tz in range(-1000.0, 1000.0),
        px in range(-100.0, 100.0), py in range(-100.0, 100.0), pz in range(-100.0, 100.0),
    ) {
        let (yaw, pitch, roll) = (yaw as f32, pitch as f32, roll as f32);
        let dcm = DirectionCosineMatrix::<MaritimeTargetFrame, Ecef>::from_euler_zyx(yaw, pitch, roll);
        let t: Vector3<Ecef, Meters, f32> = vec3![Ecef, Meters, tx as f32, ty as f32, tz as f32];
        let tf = Transform3D::new(dcm, t);
        let inv = tf.inverse();
        let p: Point3<MaritimeTargetFrame, Meters, f32> =
            pt3![MaritimeTargetFrame, Meters, px as f32, py as f32, pz as f32];

        let p2 = inv.transform_point(tf.transform_point(p));
        prop_assert!((p2.x - p.x).abs() < 1e-2, "x: {} vs {}", p2.x, p.x);
        prop_assert!((p2.y - p.y).abs() < 1e-2);
        prop_assert!((p2.z - p.z).abs() < 1e-2);
    }

    #[test]
    fn unit_convert_roundtrip_f32(v in range(-1000.0, 1000.0)) {
        let v = v as f32;
        let m: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, v, v, v];
        let feet: Vector3<LocalNed, Feet> = m.convert();
        let back: Vector3<LocalNed, Meters> = feet.convert();
        prop_assert!((back.x - v).abs() < 1e-2, "{} vs {}", back.x, v);
    }
}
