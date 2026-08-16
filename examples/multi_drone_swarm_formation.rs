//! Example 4: Multi-Drone Autonomous Swarm Formation Flight
//!
//! Demonstrates relative SE(3) rigid transformations between Leader
//! and Follower drones in a multi-vehicle autonomous swarm formation.

use frame_sc::{pt3, BodyFrame, DirectionCosineMatrix, LocalNed, Meters, Point3, Transform3D};

fn main() {
    println!("=== frame-sc: Multi-Drone Swarm Formation Flight ===\n");

    // Leader Drone Position in Local NED Frame
    let leader_pos_ned: Point3<LocalNed, Meters, f32> = pt3![LocalNed, Meters, 200.0, 150.0, -50.0];
    let leader_yaw_rad = 30.0f32.to_radians(); // Leader heading 30 deg East of North

    // Leader Body -> Local NED DCM
    let dcm_ned_to_leader =
        DirectionCosineMatrix::<LocalNed, BodyFrame>::from_euler_zyx(leader_yaw_rad, 0.0, 0.0);
    let dcm_leader_to_ned = dcm_ned_to_leader.transpose();

    // SE(3) Transform: Leader Body Frame -> Local NED
    let tf_leader_to_ned = Transform3D::new(dcm_leader_to_ned, leader_pos_ned.to_vector());

    println!("[1] Swarm Leader State (Local NED):");
    println!(
        "    Position: North {:.1} m, East {:.1} m, Down {:.1} m",
        leader_pos_ned.x, leader_pos_ned.y, leader_pos_ned.z
    );
    println!("    Heading : {:.1}° Yaw\n", leader_yaw_rad.to_degrees());

    // Swarm Formation Offset Specification in Leader Body Frame (FRD):
    // Follower 1 (Starboard Wingman): 10m Behind, 15m Right, 0m Altitude offset
    let follower_1_offset_body: Point3<BodyFrame, Meters, f32> =
        pt3![BodyFrame, Meters, -10.0, 15.0, 0.0];

    // Follower 2 (Port Wingman): 10m Behind, 15m Left (-15m), 5m Below (+5m)
    let follower_2_offset_body: Point3<BodyFrame, Meters, f32> =
        pt3![BodyFrame, Meters, -10.0, -15.0, 5.0];

    // Compute Global Local NED Position for Followers using SE(3) Leader Transform
    let follower_1_pos_ned = tf_leader_to_ned.transform_point(follower_1_offset_body);
    let follower_2_pos_ned = tf_leader_to_ned.transform_point(follower_2_offset_body);

    println!("[2] Swarm Formation Commands (Computed via SE(3) Rigid Transforms):");
    println!("    Follower 1 (Starboard Wingman):");
    println!(
        "      Desired Local NED: North {:.2} m, East {:.2} m, Down {:.2} m",
        follower_1_pos_ned.x, follower_1_pos_ned.y, follower_1_pos_ned.z
    );

    println!("    Follower 2 (Port Wingman):");
    println!(
        "      Desired Local NED: North {:.2} m, East {:.2} m, Down {:.2} m\n",
        follower_2_pos_ned.x, follower_2_pos_ned.y, follower_2_pos_ned.z
    );

    // Verify Relative Distance between Followers in Local NED vs Body Offset
    let rel_dist_ned = follower_1_pos_ned.distance_to(follower_2_pos_ned);
    let rel_dist_body = follower_1_offset_body.distance_to(follower_2_offset_body);

    println!("[3] Formation Rigidity Check:");
    println!(
        "    Relative Wingman Distance in Body Frame: {:.2} m",
        rel_dist_body.raw()
    );
    println!(
        "    Relative Wingman Distance in Local NED : {:.2} m",
        rel_dist_ned.raw()
    );

    assert!((rel_dist_ned.raw() - rel_dist_body.raw()).abs() < 1e-4);
    println!("\n[SUCCESS] Multi-vehicle swarm formation transforms verified mathematically rigid!");
}
