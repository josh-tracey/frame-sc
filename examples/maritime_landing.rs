//! Real-World Aerospace & Maritime Example:
//! Autonomous Quadrotor Deck Landing on a Dynamically Moving Ship.
//!
//! Demonstrates multi-frame navigation:
//! - ECEF (Global Geodetic Earth Frame)
//! - Local NED (Local Navigation Frame)
//! - Ship Frame (Maritime Target Platform in choppy seas)
//! - Drone Body Frame (Aircraft Body Centric)

use frame_sc::{
    pt3, vec3, BodyFrame, DirectionCosineMatrix, Ecef, LocalNed, MaritimeTargetFrame, Meters,
    Point3, Transform3D, Vector3,
};

fn main() {
    println!("=== frame-sc: Autonomous Maritime Quadrotor Deck Landing Simulation ===");

    // 1. Reference Frame Origins in ECEF
    let ned_origin_ecef: Point3<Ecef, Meters> =
        pt3![Ecef, Meters, 3_821_000.0, 312_000.0, 5_082_000.0];

    // DCM from ECEF to Local NED (fixed local tangent plane)
    let ecef_to_ned_dcm = DirectionCosineMatrix::<Ecef, LocalNed>::new([
        [-0.150, 0.988, 0.000],
        [-0.612, -0.093, 0.785],
        [-0.776, -0.118, -0.619],
    ]);

    let ecef_to_ned = Transform3D::new(
        ecef_to_ned_dcm,
        ecef_to_ned_dcm.rotate_vector(-ned_origin_ecef.to_vector()),
    );

    // 2. Ship Telemetry in Local NED
    // Ship is cruising North at 12 knots (~6.17 m/s) with pitch/roll from sea swells
    let ship_pos_ned: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 250.0, 120.0, 0.0];
    let ship_vel_ned: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 6.17, 0.5, 0.1];

    // Ship Attitude (Roll 5 deg, Pitch -3 deg, Heading 15 deg)
    let ship_attitude_dcm = DirectionCosineMatrix::<LocalNed, MaritimeTargetFrame>::from_euler_zyx(
        15.0f32.to_radians(),
        -3.0f32.to_radians(),
        5.0f32.to_radians(),
    );

    let ship_transform = Transform3D::new(ship_attitude_dcm.transpose(), ship_pos_ned.to_vector());

    // 3. Drone Telemetry in Local NED
    let drone_pos_ned: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 230.0, 110.0, -15.0];
    let drone_vel_ned: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 12.0, -1.0, 1.5];

    // Drone Attitude (Pitch down 8 deg heading towards ship)
    let drone_attitude_dcm = DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(
        20.0f32.to_radians(),
        8.0f32.to_radians(),
        0.0f32.to_radians(),
    );

    // 4. MULTI-FRAME SPATIAL CALCULATIONS (Type-Checked by Compiler!)

    // A) Relative position vector from Drone to Ship Helideck in Local NED Frame
    let relative_pos_ned: Vector3<LocalNed, Meters> = ship_pos_ned - drone_pos_ned;
    println!("\n[1] Relative Position in Local NED Frame:");
    println!(
        "    North: {:.2} m, East: {:.2} m, Down: {:.2} m",
        relative_pos_ned.x, relative_pos_ned.y, relative_pos_ned.z
    );

    // B) Convert relative target position into Drone Body Frame for flight controller guidance
    let relative_pos_body: Vector3<BodyFrame, Meters> =
        relative_pos_ned.rotate_to(&drone_attitude_dcm);
    println!("\n[2] Target Intercept Vector in Drone Body Frame:");
    println!(
        "    Forward: {:.2} m, Right: {:.2} m, Down: {:.2} m",
        relative_pos_body.x, relative_pos_body.y, relative_pos_body.z
    );

    // C) Relative Velocity in Local NED
    let relative_vel_ned: Vector3<LocalNed, Meters> = ship_vel_ned - drone_vel_ned;

    // D) Transform Relative Velocity into Ship Frame to check deck impact velocity
    let relative_vel_ship: Vector3<MaritimeTargetFrame, Meters> =
        relative_vel_ned.rotate_to(&ship_attitude_dcm);
    println!("\n[3] Closing Velocity in Ship Helideck Frame:");
    println!(
        "    Surge (Bow): {:.2} m/s, Sway (Port): {:.2} m/s, Heave (Deck Touchdown): {:.2} m/s",
        relative_vel_ship.x, relative_vel_ship.y, relative_vel_ship.z
    );

    // E) Helideck touchdown point in Ship Frame (0, 0, -2.5m deck height above waterline)
    let helideck_ship: Point3<MaritimeTargetFrame, Meters> =
        pt3![MaritimeTargetFrame, Meters, 0.0, 0.0, -2.5];

    // Transform deck touchdown target to ECEF global coordinates for satellite link logging
    let helideck_ned = ship_transform.transform_point(helideck_ship);
    let ned_to_ecef = ecef_to_ned.inverse();
    let helideck_ecef = ned_to_ecef.transform_point(helideck_ned);

    println!("\n[4] Global Helideck Position (ECEF Frame):");
    println!(
        "    X: {:.1} m, Y: {:.1} m, Z: {:.1} m",
        helideck_ecef.x, helideck_ecef.y, helideck_ecef.z
    );

    println!("\n[SUCCESS] Multi-frame vector & point operations compiled and executed with 0 runtime tag overhead!");
}
