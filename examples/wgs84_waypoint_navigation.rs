//! Example 2: WGS-84 Geodetic Waypoint Navigation Pipeline
//!
//! Demonstrates how `frame-sc` enables safe conversion between global WGS-84
//! (Latitude, Longitude, Altitude) waypoints and local Cartesian Local NED
//! coordinates for real-time UAV navigation loops.

use frame_sc::{pt3, LocalNed, Meters, Point3, Vector3, Wgs84Position};

fn main() {
    println!("=== frame-sc: WGS-84 Geodetic Waypoint Navigation ===\n");

    // 1. Define Launch Pad Home Location in WGS-84 Geodetic Coordinates
    // (San Francisco Bay Base: 37.7749° N, -122.4194° W, 15.0m MSL Altitude)
    let home_base = Wgs84Position::new(37.774900, -122.419400, 15.0);
    println!("[1] Home Base WGS-84 Location:");
    println!(
        "    Lat: {:.6}°, Lon: {:.6}°, Alt: {:.1} m\n",
        home_base.lat_deg, home_base.lon_deg, home_base.alt_m
    );

    // 2. Define Mission Waypoints in WGS-84 (from Ground Control Station / QGroundControl)
    let waypoints = [
        Wgs84Position::new(37.775800, -122.418200, 50.0), // WP1: ~100m North, ~105m East, 50m Alt
        Wgs84Position::new(37.776500, -122.420500, 60.0), // WP2: ~177m North, ~97m West, 60m Alt
        Wgs84Position::new(37.774100, -122.421000, 30.0), // WP3: ~89m South, ~140m West, 30m Alt
    ];

    // Current Drone Position relative to Home Base in Local NED
    let drone_curr_ned: Point3<LocalNed, Meters, f64> = pt3![LocalNed, Meters, 0.0, 0.0, -15.0]; // 15m AGL (Down = -15)

    println!("[2] Processing Mission Waypoints in Flight Controller:");
    for (i, wp) in waypoints.iter().enumerate() {
        // Convert WGS-84 Waypoint to Local NED Point relative to Home Base
        let wp_ned: Point3<LocalNed, Meters, f64> = wp.to_local_ned(&home_base);

        // Compute Relative Displacement Vector from Current Position to Waypoint
        let dist_vector: Vector3<LocalNed, Meters, f64> = wp_ned - drone_curr_ned;
        let distance_m = dist_vector.norm().raw();

        // Calculate Bearing Angle (Yaw in Local NED: atan2(East, North))
        let bearing_deg = dist_vector.y.atan2(dist_vector.x).to_degrees();

        println!("    Waypoint {}:", i + 1);
        println!(
            "      Target WGS-84 : Lat {:.6}°, Lon {:.6}°, Alt {:.1} m",
            wp.lat_deg, wp.lon_deg, wp.alt_m
        );
        println!(
            "      Target Local NED: North: {:.2} m, East: {:.2} m, Down: {:.2} m",
            wp_ned.x, wp_ned.y, wp_ned.z
        );
        println!(
            "      Flight Command : Range: {:.1} m, Bearing: {:.1}°, Alt Diff: {:.1} m\n",
            distance_m,
            bearing_deg,
            -wp_ned.z - (-drone_curr_ned.z)
        );
    }

    // 3. Convert Local NED Offset back to Global WGS-84 for Telemetry Broadcasting
    let simulated_offset: Point3<LocalNed, Meters, f64> =
        pt3![LocalNed, Meters, 250.0, -120.0, -75.0];
    let drone_telemetry_wgs84 = Wgs84Position::from_local_ned(simulated_offset, &home_base);

    println!("[3] Telemetry Output (Converted back to WGS-84 for Ground Station):");
    println!(
        "    Estimated Position: Lat {:.6}°, Lon {:.6}°, Alt {:.1} m",
        drone_telemetry_wgs84.lat_deg, drone_telemetry_wgs84.lon_deg, drone_telemetry_wgs84.alt_m
    );
    println!("\n[SUCCESS] WGS-84 <-> Local NED geodetic conversions completed with zero-cost type safety!");
}
