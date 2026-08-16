//! Example 3: Camera Gimbal Ground Target Tracking
//!
//! Demonstrates multi-frame transformation chaining:
//! Optical Camera Frame -> Gimbal Body Frame -> Drone Body Frame -> Local NED -> ECEF.

use frame_sc::{
    pt3, vec3, BodyFrame, CameraFrame, DirectionCosineMatrix, Ecef, LocalNed, Meters, Point3,
    Transform3D, Vector3,
};

fn main() {
    println!("=== frame-sc: Multi-Frame Camera Gimbal Target Tracking ===\n");

    // 1. Target optical detection in Camera Frame (X = Right, Y = Down, Z = Forward/Optical Axis)
    // Detected ground vehicle 45 meters ahead along optical axis, slightly left (-2m) and down (+5m)
    let target_camera_frame: Point3<CameraFrame, Meters, f32> =
        pt3![CameraFrame, Meters, -2.0, 5.0, 45.0];
    println!("[1] Detected Target in Optical Camera Frame:");
    println!(
        "    X (Right): {:.2} m, Y (Down): {:.2} m, Z (Optical Axis): {:.2} m\n",
        target_camera_frame.x, target_camera_frame.y, target_camera_frame.z
    );

    // 2. Camera-to-Body Transformation (Gimbal pitch down by 30 deg = 0.5236 rad)
    let pitch_rad = 30.0f32.to_radians();
    // Rotation DCM converting optical camera vectors to aircraft Body Frame (Forward, Right, Down)
    let dcm_cam_to_body =
        DirectionCosineMatrix::<CameraFrame, BodyFrame>::from_euler_zyx(0.0, pitch_rad, 0.0);
    let camera_offset_body: Vector3<BodyFrame, Meters, f32> =
        vec3![BodyFrame, Meters, 0.2, 0.0, 0.1]; // Gimbal mounted 0.2m forward, 0.1m down
    let tf_cam_to_body = Transform3D::new(dcm_cam_to_body, camera_offset_body);

    let target_body_frame = tf_cam_to_body.transform_point(target_camera_frame);
    println!("[2] Target Location in Aircraft Body Frame (FRD):");
    println!(
        "    Forward: {:.2} m, Right: {:.2} m, Down: {:.2} m\n",
        target_body_frame.x, target_body_frame.y, target_body_frame.z
    );

    // 3. Body-to-NED Transformation (Drone attitude: 5 deg Roll, 10 deg Pitch, 45 deg Yaw)
    let roll_rad = 5.0f32.to_radians();
    let pitch_drone = 10.0f32.to_radians();
    let yaw_drone = 45.0f32.to_radians();

    let dcm_ned_to_body = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(
        yaw_drone,
        pitch_drone,
        roll_rad,
    );
    let dcm_body_to_ned = dcm_ned_to_body.transpose(); // Transpose = inverse rotation

    let drone_pos_ned: Vector3<LocalNed, Meters, f32> =
        vec3![LocalNed, Meters, 500.0, 300.0, -100.0]; // Drone at 100m AGL
    let tf_body_to_ned = Transform3D::new(dcm_body_to_ned, drone_pos_ned);

    let target_ned_frame = tf_body_to_ned.transform_point(target_body_frame);
    println!("[3] Target Location in Local NED Frame:");
    println!(
        "    North: {:.2} m, East: {:.2} m, Down: {:.2} m (Elevation {:.2} m AGL)\n",
        target_ned_frame.x, target_ned_frame.y, target_ned_frame.z, -target_ned_frame.z
    );

    // 4. Local NED to ECEF Absolute Coordinate Transform
    let ned_origin_ecef: Vector3<Ecef, Meters, f32> =
        vec3![Ecef, Meters, 3_821_000.0, 312_000.0, 5_082_000.0];
    let dcm_ned_to_ecef = DirectionCosineMatrix::<LocalNed, Ecef>::IDENTITY;
    let tf_ned_to_ecef = Transform3D::new(dcm_ned_to_ecef, ned_origin_ecef);

    let target_ecef_frame = tf_ned_to_ecef.transform_point(target_ned_frame);
    println!("[4] Target Global Coordinates in ECEF Frame:");
    println!(
        "    X: {:.1} m, Y: {:.1} m, Z: {:.1} m",
        target_ecef_frame.x, target_ecef_frame.y, target_ecef_frame.z
    );

    println!("\n[SUCCESS] Chained 4-frame transformation (Camera -> Body -> Local NED -> ECEF) with 0 runtime overhead!");
}
