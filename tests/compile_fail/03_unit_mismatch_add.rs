use frame_sc::{vec3, BodyFrame, Meters, Millimeters, Vector3};

fn main() {
    let v_m: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 1.0, 0.0, 0.0];
    let v_mm: Vector3<BodyFrame, Millimeters> = vec3![BodyFrame, Millimeters, 500.0, 0.0, 0.0];

    // Mismatched units: Meters vs Millimeters without explicit convert()
    let _invalid = v_m + v_mm;
}
