use core::mem::size_of;
use frame_sc::{
    pt3, vec3, BodyFrame, Degrees, DirectionCosineMatrix, Ecef, Feet, LocalNed,
    MaritimeTargetFrame, Meters, Millimeters, Point3, Pose3D, Quaternion, Radians, Scalar,
    Transform3D, Vector3, Wgs84Position,
};

#[test]
fn test_zero_cost_memory_layout() {
    // Verify that Vector3, Point3, and DCM have exact same size as raw float arrays
    assert_eq!(
        size_of::<Vector3<BodyFrame, Meters, f32>>(),
        size_of::<[f32; 3]>()
    );
    assert_eq!(
        size_of::<Point3<LocalNed, Meters, f32>>(),
        size_of::<[f32; 3]>()
    );
    assert_eq!(
        size_of::<DirectionCosineMatrix<LocalNed, BodyFrame, f32>>(),
        size_of::<[[f32; 3]; 3]>()
    );
    assert_eq!(size_of::<Scalar<Meters, f32>>(), size_of::<f32>());

    assert_eq!(
        size_of::<Vector3<BodyFrame, Meters, f64>>(),
        size_of::<[f64; 3]>()
    );
    assert_eq!(
        size_of::<Point3<LocalNed, Meters, f64>>(),
        size_of::<[f64; 3]>()
    );
}

#[test]
fn test_affine_point_vector_arithmetic() {
    let p1: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 10.0, 20.0, -5.0];
    let p2: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 14.0, 16.0, -2.0];

    // Point - Point = Vector
    let disp: Vector3<LocalNed, Meters> = p2 - p1;
    assert_eq!(disp, vec3![LocalNed, Meters, 4.0, -4.0, 3.0]);

    // Point + Vector = Point
    let p3 = p1 + disp;
    assert_eq!(p3, p2);

    // Point - Vector = Point
    let p4 = p2 - disp;
    assert_eq!(p4, p1);

    // Distance
    let dist = p1.distance_to(p2);
    // sqrt(4^2 + (-4)^2 + 3^2) = sqrt(16 + 16 + 9) = sqrt(41) ≈ 6.403124
    let expected = (41.0f32).sqrt();
    assert!((dist.raw() - expected).abs() < 1e-5);
}

#[test]
fn test_vector_algebra() {
    let v1: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 1.0, 2.0, 3.0];
    let v2: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 4.0, 5.0, 6.0];

    assert_eq!(v1 + v2, vec3![BodyFrame, Meters, 5.0, 7.0, 9.0]);
    assert_eq!(v2 - v1, vec3![BodyFrame, Meters, 3.0, 3.0, 3.0]);
    assert_eq!(v1 * 2.0, vec3![BodyFrame, Meters, 2.0, 4.0, 6.0]);
    assert_eq!(v2 / 2.0, vec3![BodyFrame, Meters, 2.0, 2.5, 3.0]);
    assert_eq!(-v1, vec3![BodyFrame, Meters, -1.0, -2.0, -3.0]);

    // Dot product: 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    assert_eq!(v1.dot(v2), 32.0);

    // Cross product: (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    assert_eq!(v1.cross(v2), vec3![BodyFrame, Meters, -3.0, 6.0, -3.0]);
}

#[test]
fn test_unit_conversions() {
    let v_m: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 1.5, -2.0, 0.5];
    let v_mm: Vector3<LocalNed, Millimeters> = v_m.convert();
    assert_eq!(v_mm, vec3![LocalNed, Millimeters, 1500.0, -2000.0, 500.0]);

    let v_feet: Vector3<LocalNed, Feet> = v_m.convert();
    let back_m: Vector3<LocalNed, Meters> = v_feet.convert();
    assert!((back_m.x - 1.5).abs() < 1e-4);

    let angle_deg: Scalar<Degrees, f32> = Scalar::new(90.0);
    let angle_rad: Scalar<Radians, f32> = angle_deg.convert();
    assert!((angle_rad.raw() - core::f32::consts::FRAC_PI_2).abs() < 1e-4);
}

#[test]
fn test_dcm_rotations_and_chaining() {
    // Rotate LocalNed to BodyFrame by 90 deg Yaw (pi/2)
    let dcm_ned_to_body = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(
        core::f32::consts::FRAC_PI_2,
        0.0,
        0.0,
    );

    let north_vel: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 10.0, 0.0, 0.0];

    // North vector in NED rotated by 90 deg yaw corresponds to Left (-Y) or Right in Body Frame depending on convention
    let body_vel = dcm_ned_to_body.rotate_vector(north_vel);
    // [cos(pi/2)*10, sin(pi/2)*10, 0] = [0, 10, 0]
    assert!((body_vel.x - 0.0).abs() < 1e-5);
    assert!((body_vel.y - (-10.0)).abs() < 1e-5);
    assert!((body_vel.z - 0.0).abs() < 1e-5);

    // Test Transpose (Inverse Rotation)
    let dcm_body_to_ned = dcm_ned_to_body.transpose();
    let restored_ned_vel = dcm_body_to_ned.rotate_vector(body_vel);
    assert!((restored_ned_vel.x - north_vel.x).abs() < 1e-5);
    assert!((restored_ned_vel.y - north_vel.y).abs() < 1e-5);

    // Chaining DCMs: R_{A->B} * R_{B->C} = R_{A->C}
    let dcm_body_to_ship = DirectionCosineMatrix::<BodyFrame, MaritimeTargetFrame>::IDENTITY;
    let dcm_ned_to_ship = dcm_ned_to_body * dcm_body_to_ship;
    let ship_vel = dcm_ned_to_ship.rotate_vector(north_vel);
    assert_eq!(
        ship_vel,
        vec3![
            MaritimeTargetFrame,
            Meters,
            body_vel.x,
            body_vel.y,
            body_vel.z
        ]
    );
}

#[test]
fn test_rigid_transform_se3() {
    // Ship origin in ECEF frame
    let ship_origin_ecef: Vector3<Ecef, Meters> = vec3![Ecef, Meters, 1000.0, 2000.0, 3000.0];
    let dcm_ship_to_ecef = DirectionCosineMatrix::<MaritimeTargetFrame, Ecef>::IDENTITY;

    let ship_to_ecef = Transform3D::new(dcm_ship_to_ecef, ship_origin_ecef);

    let point_in_ship: Point3<MaritimeTargetFrame, Meters> =
        pt3![MaritimeTargetFrame, Meters, 10.0, 0.0, -2.0];
    let point_in_ecef = ship_to_ecef.transform_point(point_in_ship);

    assert_eq!(point_in_ecef, pt3![Ecef, Meters, 1010.0, 2000.0, 2998.0]);

    // Inverse transform
    let ecef_to_ship = ship_to_ecef.inverse();
    let restored_ship_pt = ecef_to_ship.transform_point(point_in_ecef);
    assert!((restored_ship_pt.x - point_in_ship.x).abs() < 1e-4);
    assert!((restored_ship_pt.y - point_in_ship.y).abs() < 1e-4);
    assert!((restored_ship_pt.z - point_in_ship.z).abs() < 1e-4);
}

#[test]
fn test_wgs84_conversions() {
    // San Francisco Home Base: ~37.7749 deg N, -122.4194 deg W, 10m alt
    let home = Wgs84Position::new(37.7749, -122.4194, 10.0);

    // ECEF roundtrip
    let ecef_home = home.to_ecef();
    let restored_home = ecef_home.to_wgs84();

    assert!((restored_home.lat_deg - home.lat_deg).abs() < 1e-7);
    assert!((restored_home.lon_deg - home.lon_deg).abs() < 1e-7);
    assert!((restored_home.alt_m - home.alt_m).abs() < 1e-4);

    // Local NED relative displacement: 100m North, 50m East, -20m Down (up 20m)
    let ned_offset: Point3<LocalNed, Meters, f64> = pt3![LocalNed, Meters, 100.0, 50.0, -20.0];
    let drone_wgs84 = Wgs84Position::from_local_ned(ned_offset, &home);

    // Convert back from drone_wgs84 to local NED
    let restored_ned = drone_wgs84.to_local_ned(&home);
    assert!((restored_ned.x - 100.0).abs() < 1e-3);
    assert!((restored_ned.y - 50.0).abs() < 1e-3);
    assert!((restored_ned.z - (-20.0)).abs() < 1e-3);
}

#[test]
fn test_quaternion_and_pose() {
    assert_eq!(
        size_of::<Quaternion<LocalNed, BodyFrame, f32>>(),
        size_of::<[f32; 4]>()
    );

    // Create Quaternion for 90 deg Yaw
    let q_ned_to_body = Quaternion::<LocalNed, BodyFrame, f32>::from_euler_zyx(
        core::f32::consts::FRAC_PI_2,
        0.0,
        0.0,
    );

    let north_vel: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 10.0, 0.0, 0.0];
    let body_vel = q_ned_to_body.rotate_vector(north_vel);

    // A NED "north" vector, viewed in a body frame yawed +90 deg, points out the
    // body's left (-Y) side under the passive frame-transform convention.
    assert!((body_vel.x - 0.0).abs() < 1e-4);
    assert!((body_vel.y - (-10.0)).abs() < 1e-4);
    assert!((body_vel.z - 0.0).abs() < 1e-4);

    // Quaternion rotation must agree with its own DCM form.
    let body_vel_dcm = q_ned_to_body.to_dcm().rotate_vector(north_vel);
    assert!((body_vel.x - body_vel_dcm.x).abs() < 1e-5);
    assert!((body_vel.y - body_vel_dcm.y).abs() < 1e-5);
    assert!((body_vel.z - body_vel_dcm.z).abs() < 1e-5);

    // Invert the NED -> Body quaternion into a Body -> NED DCM for the pose.
    let dcm_body_to_ned = q_ned_to_body.to_dcm().transpose();
    let drone_pos_ned: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 50.0, 100.0, -25.0];
    let drone_pose = Pose3D::new(drone_pos_ned, dcm_body_to_ned);

    let pt_body: Point3<BodyFrame, Meters> = pt3![BodyFrame, Meters, 5.0, 0.0, 0.0];
    let pt_ned = drone_pose.transform_point(pt_body);

    assert_eq!(pt_ned, pt3![LocalNed, Meters, 50.0, 105.0, -25.0]);
}

#[test]
fn test_quaternion_dcm_convention_consistency() {
    let (yaw, pitch, roll) = (1.2f32, -0.4f32, 0.7f32);

    let dcm = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(yaw, pitch, roll);
    let q = Quaternion::<LocalNed, BodyFrame>::from_euler_zyx(yaw, pitch, roll);

    let v: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 3.0, -2.0, 7.0];

    let via_dcm = dcm.rotate_vector(v);
    let via_q = q.rotate_vector(v);
    let via_q_dcm = q.to_dcm().rotate_vector(v);

    assert!((via_q.x - via_dcm.x).abs() < 1e-5);
    assert!((via_q.y - via_dcm.y).abs() < 1e-5);
    assert!((via_q.z - via_dcm.z).abs() < 1e-5);
    assert!((via_q_dcm.x - via_dcm.x).abs() < 1e-5);
    assert!((via_q_dcm.y - via_dcm.y).abs() < 1e-5);
    assert!((via_q_dcm.z - via_dcm.z).abs() < 1e-5);

    // Round-trip through the inverse DCM restores the original vector.
    let restored = dcm.transpose().rotate_vector(via_dcm);
    assert!((restored.x - v.x).abs() < 1e-4);
    assert!((restored.y - v.y).abs() < 1e-4);
    assert!((restored.z - v.z).abs() < 1e-4);
}

#[test]
fn test_quaternion_f64() {
    let q = Quaternion::<LocalNed, BodyFrame, f64>::from_euler_zyx(0.9, -0.3, 0.2);
    let dcm = q.to_dcm();

    let v: Vector3<LocalNed, Meters, f64> = vec3![LocalNed, Meters, 5.0, 1.0, -8.0];

    let via_q = q.rotate_vector(v);
    let via_dcm = dcm.rotate_vector(v);

    assert!((via_q.x - via_dcm.x).abs() < 1e-9);
    assert!((via_q.y - via_dcm.y).abs() < 1e-9);
    assert!((via_q.z - via_dcm.z).abs() < 1e-9);

    // Composition must match sequential application.
    let q1 = Quaternion::<LocalNed, MaritimeTargetFrame, f64>::from_euler_zyx(0.5, 0.0, 0.0);
    let q2 = Quaternion::<MaritimeTargetFrame, BodyFrame, f64>::from_euler_zyx(0.0, 0.3, 0.0);
    let seq = q2.rotate_vector(q1.rotate_vector(v));
    let composed = (q1 * q2).rotate_vector(v);
    assert!((seq.x - composed.x).abs() < 1e-9);
    assert!((seq.y - composed.y).abs() < 1e-9);
    assert!((seq.z - composed.z).abs() < 1e-9);
}
