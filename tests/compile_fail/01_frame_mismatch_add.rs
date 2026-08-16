use frame_sc::{vec3, BodyFrame, LocalNed, Meters, Vector3};

fn main() {
    let drone_vel: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 10.0, 0.0, 0.0];
    let wind_vel: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 2.0, -3.0, 0.0];

    // Mismatched reference frames: BodyFrame vs LocalNed
    let _effective = drone_vel + wind_vel;
}
