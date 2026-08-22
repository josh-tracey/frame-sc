//! Serde round-trip coverage for the optional `serde` feature.
#![cfg(feature = "serde")]

use core::fmt::Debug;
use frame_sc::{
    pt3, vec3, BodyFrame, DirectionCosineMatrix, LocalNed, MaritimeTargetFrame, Meters, Point3,
    Pose3D, Quaternion, Scalar, Transform3D, Vector3, Wgs84Position,
};

/// Serialize to JSON and deserialize back, asserting an exact round-trip.
fn round_trip<T>(value: &T) -> T
where
    T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + Debug,
{
    let json = serde_json::to_string(value).expect("serialize");
    let restored: T = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(&restored, value);
    restored
}

#[test]
fn serde_round_trips_all_types() {
    let v32: Vector3<BodyFrame, Meters, f32> = vec3![BodyFrame, Meters, 1.5, -2.0, 3.25];
    round_trip(&v32);

    let v64: Vector3<BodyFrame, Meters, f64> = vec3![BodyFrame, Meters, 1.0, 2.0, 3.0];
    round_trip(&v64);

    let p: Point3<LocalNed, Meters, f32> = pt3![LocalNed, Meters, 10.0, 20.0, -5.0];
    round_trip(&p);

    let s: Scalar<Meters, f32> = Scalar::new(2.5);
    round_trip(&s);

    let q = Quaternion::<LocalNed, BodyFrame, f32>::from_euler_zyx(0.3, -0.2, 0.1);
    round_trip(&q);

    let dcm = DirectionCosineMatrix::<LocalNed, BodyFrame, f32>::from_euler_zyx(0.3, -0.2, 0.1);
    round_trip(&dcm);

    let tf = Transform3D::<MaritimeTargetFrame, LocalNed, Meters, f32>::new(
        DirectionCosineMatrix::<MaritimeTargetFrame, LocalNed>::IDENTITY,
        vec3![LocalNed, Meters, 1.0, 2.0, 3.0],
    );
    round_trip(&tf);

    let pose = Pose3D::<BodyFrame, LocalNed, Meters, f32>::new(
        pt3![LocalNed, Meters, 50.0, 100.0, -25.0],
        DirectionCosineMatrix::<BodyFrame, LocalNed>::IDENTITY,
    );
    round_trip(&pose);

    let w = Wgs84Position::new(37.7749, -122.4194, 10.0);
    round_trip(&w);
}
