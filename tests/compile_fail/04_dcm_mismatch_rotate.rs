use frame_sc::{vec3, BodyFrame, DirectionCosineMatrix, Ecef, LocalNed, Meters, Vector3};

fn main() {
    let ned_vel: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 10.0, 0.0, 0.0];
    let dcm_body_to_ecef = DirectionCosineMatrix::<BodyFrame, Ecef>::IDENTITY;

    // Invalid DCM input frame: Expected BodyFrame, found LocalNed
    let _invalid = ned_vel.rotate_to(&dcm_body_to_ecef);
}
