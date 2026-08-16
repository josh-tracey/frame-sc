# frame-sc: Zero-Cost Spatial Typestates

> Static analysis and compile-time type safety for spatial reference frames and physical units in Rust.

---

## The Problem

In multi-axis control environments—such as landing autonomous vehicles on dynamically moving maritime platforms, satellite docking, WGS-84 waypoint navigation, or optical target tracking—engineers must constantly translate between different spatial frames of reference:

- **ECEF** (Earth-Centered Earth-Fixed Cartesian)
- **WGS-84** (Geodetic Latitude, Longitude, Altitude)
- **Local NED** (Local North-East-Down Tangent Plane)
- **Ship / Target Frame** (Attached to moving vessel)
- **Body Frame** (Forward-Right-Down / Aircraft Centric)
- **Camera Frame** (Optical Axis Payload Frame)

A common, catastrophic error in aviation and robotics software is mathematically combining vectors or points from two different reference frames without rotating or translating them first.

---

## The Solution

`frame-sc` encodes both the **Reference Frame** and **Physical Unit** directly into the type signature using Zero-Sized Types (ZSTs) and `PhantomData`.

The Rust compiler acts as a static physics engine, refusing to compile any operation that mixes incompatible spatial topologies, unit mismatches, or invalid affine geometry (e.g. adding two positions together).

### Key Design Principles

1. **Dual Frame + Unit Typestates**: Every vector/point encodes both `Frame` AND `Unit` (`Vector3<Frame, Unit>`).
2. **Zero Runtime Overhead**: 100% `#[repr(C)]` layout matching raw scalar arrays (`size_of::<Vector3<F, U, f32>>() == 12` bytes, `size_of::<Quaternion>() == 16` bytes).
3. **Hard Real-Time `#![no_std]`**: Pure lightweight Rust with zero dynamic heap allocations (`no_alloc`) and zero heavy third-party dependencies.

---

## Features & Mathematical Primitives

- **`#![no_std]` Compatible**: Ready for bare-metal flight control microcontrollers (`libm` integration).
- **Dual-Tag Typestates**: Types track both `Frame` and `Unit` (`Meters`, `Millimeters`, `Feet`, `Knots`, `MetersPerSecond`, `Radians`).
- **`Quaternion<From, To>`**: Singularity-free $SO(3)$ unit quaternions with ZYX Tait-Bryan Euler angle constructors and $v' = q v q^*$ vector rotations.
- **`Pose3D<From, To, Unit>`**: High-level engineering pose combining 3D origin position (`Point3`) and attitude orientation (`DirectionCosineMatrix` / `Quaternion`).
- **`Transform3D<From, To, Unit>`**: $SE(3)$ rigid body transformation matrix for multi-frame composition.
- **WGS-84 Geodetic Pipeline**: Bi-directional closed-form conversions between WGS-84 $(\text{Lat}, \text{Lon}, \text{Alt})$, ECEF $(X,Y,Z)$, and Local NED $(N,E,D)$.

---

## Conceptual Syntax & Compile Safety

```rust
use frame_sc::{
    vec3, pt3, BodyFrame, DirectionCosineMatrix, LocalNed, MetersPerSecond, Pose3D, Quaternion, Vector3
};

// Drone velocity in aircraft Body Frame
let drone_velocity: Vector3<BodyFrame, MetersPerSecond> = vec3![BodyFrame, MetersPerSecond, 15.0, 0.0, 0.0];

// Wind vector in Local NED Navigation Frame
let wind_vector: Vector3<LocalNed, MetersPerSecond> = vec3![LocalNed, MetersPerSecond, 2.0, -3.0, 0.0];

// COMPILER ERROR: Expected `BodyFrame`, found `LocalNed`
// let effective_velocity = drone_velocity + wind_vector;

// Attitude Quaternion (Local NED to Body Frame)
let q_ned_to_body = Quaternion::<LocalNed, BodyFrame>::from_euler_zyx(0.0, 0.1, 0.0);

// COMPILES SAFELY (Rotated into target frame):
let effective_velocity = drone_velocity + q_ned_to_body.rotate_vector(wind_vector);
```

---

## Examples & Demonstrations

`frame-sc` includes four real-world flight control examples:

```bash
# 1. Maritime Deck Landing Simulation (ECEF -> Local NED -> Ship Frame -> Body Frame)
cargo run --example maritime_landing

# 2. WGS-84 Geodetic Waypoint Navigation Pipeline
cargo run --example wgs84_waypoint_navigation

# 3. Optical Camera Gimbal Ground Target Tracking
cargo run --example camera_gimbal_target_tracking

# 4. Multi-Vehicle Swarm Formation Flight via SE(3) Transforms
cargo run --example multi_drone_swarm_formation
```

---

## Verification & Benchmarks

```bash
# Run integration & trybuild compile-fail safety tests
cargo test

# Run zero-cost Criterion benchmarks
cargo bench
```

---

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
