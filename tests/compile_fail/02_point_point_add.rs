use frame_sc::{pt3, LocalNed, Meters, Point3};

fn main() {
    let p1: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 10.0, 20.0, 0.0];
    let p2: Point3<LocalNed, Meters> = pt3![LocalNed, Meters, 5.0, 15.0, 0.0];

    // Invalid affine operation: Point + Point is forbidden
    let _invalid = p1 + p2;
}
