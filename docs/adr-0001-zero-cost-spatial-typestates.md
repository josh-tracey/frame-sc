# ADR: Zero-Cost Spatial Typestates for Compile-Time Reference Frame & Unit Safety

**Author:** Joshua Tracey
**Date:** 2026-08-15
**TRL Phase:** TRL 2 (Concept & Architecture)
**Status:** Accepted
**Lead Sign-off:** Joshua Tracey
---

## 1. Problem & Success Criteria

### Problem

In multi-axis flight control and autonomous navigation environments—such as landing autonomous vehicles on dynamically moving maritime platforms, WGS-84 waypoint navigation, optical camera target tracking, or multi-vehicle swarm formation flight—software engineers must constantly translate between different spatial frames of reference (e.g., Earth-Centered Earth-Fixed `ECEF`, Local North-East-Down `LocalNed`, Ship / Target Frame `MaritimeTargetFrame`, Aircraft `BodyFrame`, and Optical `CameraFrame`).

A common, catastrophic error in aviation and robotics software is mathematically combining vectors or positions from two different reference frames without rotating or translating them first (or mixing incompatible units such as meters and millimeters). In runtime-checked or untyped math libraries, these errors silently pass through control loops and produce erroneous control commands, leading to vehicle crashes, sensor drift, or flight termination.

### Success Criteria

1. **Mathematical Safety**: Absolute compile-time prohibition of invalid cross-frame arithmetic, cross-unit operations, and invalid affine geometry (e.g. adding two absolute positions).
2. **Zero Runtime Cost**: Zero memory, CPU, or allocation overhead compared to raw `[f32; 3]` or `[f64; 3]` scalar math (`size_of::<Vector3<F, U, f32>>() == 12 bytes`, `size_of::<Quaternion>() == 16 bytes`).
3. **WGS-84 Geodetic Integration**: Type-safe, closed-form conversion pipeline between global WGS-84 coordinates (Latitude, Longitude, Altitude) and local Cartesian frames (`LocalNed`, `Ecef`).
4. **Hard Real-Time & Determinism**: `#![no_std]` core compatibility with zero dynamic memory allocation (`no_alloc`), fully deterministic execution time for microsecond flight control loops.

---

## 2. Proposed Architecture & Solution

We propose `frame-sc`, a Rust static analysis library that encodes **Reference Frame** and **Physical Unit** topologies directly into Rust type signatures using Zero-Sized Types (ZSTs) and `PhantomData<T>`.

### Data Flow & Type Hierarchy

1. **Zero-Sized Marker Traits (`ZST`)**:
   - `Frame`: `BodyFrame`, `LocalNed`, `LocalEnu`, `Ecef`, `MaritimeTargetFrame`, `CameraFrame`
   - `Unit`: `Meters`, `Millimeters`, `Feet`, `Kilometers`, `Radians`, `Degrees`, `MetersPerSecond`, `Knots`

2. **Core Typestate Structures**:
   - `Vector3<F: Frame, U: Unit, T = f32>`: 3D spatial vector with `#[repr(C)]` layout matching `[T; 3]`.
   - `Point3<F: Frame, U: Unit, T = f32>`: 3D spatial position enforcing strict affine space rules (`Point - Point = Vector`, `Point + Vector = Point`, `Point + Point` fails compilation).
   - `Quaternion<From: Frame, To: Frame, T = f32>`: Singularity-free $SO(3)$ unit quaternion representation with $v' = q^* v q$ vector rotation and ZYX Tait-Bryan Euler constructors (`#[repr(C)]` matching `[T; 4]`).
   - `DirectionCosineMatrix<From: Frame, To: Frame, T = f32>`: $SO(3)$ rotation matrix that explicitly transforms `Vector3<From, U, T>` to `Vector3<To, U, T>` and chains matrix multiplications ($R_{A \to B} \times R_{B \to C} = R_{A \to C}$).
   - `Pose3D<From: Frame, To: Frame, U: Unit, T = f32>`: High-level engineering pose combining 3D origin position (`Point3`) and attitude orientation (`DirectionCosineMatrix` / `Quaternion`).
   - `Transform3D<From: Frame, To: Frame, U: Unit, T = f32>`: $SE(3)$ rigid body transformation combining rotation DCM and translational origin offset.
   - `Wgs84Position`: Geodetic coordinate container ($\text{Lat}^\circ, \text{Lon}^\circ, \text{Alt}_m$) with bi-directional conversion to `Point3<Ecef, Meters, f64>` and relative `Point3<LocalNed, Meters, f64>` relative to a reference home base.

### Conceptual Control Loop Integration

```rust
// WGS-84 Waypoint Navigation -> Local NED Flight Command
let home = Wgs84Position::new(37.7749, -122.4194, 10.0);
let target_wgs84 = Wgs84Position::new(37.7758, -122.4182, 50.0);

// Convert WGS-84 Waypoint to Local NED Cartesian Point relative to Home
let target_ned: Point3<LocalNed, Meters, f64> = target_wgs84.to_local_ned(&home);

// Telemetry inputs
let drone_velocity: Vector3<BodyFrame, MetersPerSecond> = get_body_velocity();
let wind_vector: Vector3<LocalNed, MetersPerSecond> = get_wind_vector();
let q_ned_to_body: Quaternion<LocalNed, BodyFrame> = get_attitude_quaternion();

// COMPILES SAFELY (Rotated into target frame):
let total_velocity = drone_velocity + q_ned_to_body.rotate_vector(wind_vector);
```

---

## 3. Technical Constraints & Safety

### Geodetic Math & Determinism

- **WGS-84 Ellipsoid Precision:** Conversions use closed-form equations ($a = 6,378,137\text{ m}$, $f = 1/298.257223563$) and Bowring's method for ECEF $\to$ LLA to guarantee sub-millimeter precision without iterative loops.
- **Performance & Footprint:** `#[repr(C)]` memory layout matching raw float arrays (`size_of::<Vector3>() == 12` bytes for `f32`, `24` bytes for `f64`).
- **Zero Allocations:** No heap allocation (`alloc`) required. All operations execute on stack/registers.
- **Embedded Compatibility:** Full `#![no_std]` compatibility using `libm` for scalar floating-point math (`sqrt`, `sin`, `cos`, `atan2`).

---

## 4. Real-World Demonstrations & Examples

The codebase includes four complete end-to-end flight control examples:

1. `examples/maritime_landing.rs`: Autonomous quadrotor vessel deck landing in rough seas (`ECEF` $\to$ `LocalNed` $\to$ `MaritimeTargetFrame` $\to$ `BodyFrame`).
2. `examples/wgs84_waypoint_navigation.rs`: Mission waypoint execution (`Wgs84Position` $\leftrightarrow$ `LocalNed` $\leftrightarrow$ `ECEF`).
3. `examples/camera_gimbal_target_tracking.rs`: Optical payload target tracking (`CameraFrame` $\to$ `BodyFrame` $\to$ `LocalNed` $\to$ `ECEF`).
4. `examples/multi_drone_swarm_formation.rs`: Multi-UAV swarm formation flight using $SE(3)$ rigid transforms.

---

## 5. Alternatives Evaluated & Justification

### Evaluation of Existing Crates

We conducted a technical audit of existing Rust crates across four core requirements:

1. **Spatial Frame Typestates**: Compile-time prohibition of cross-frame operations and $SO(3)$ / $SE(3)$ compositions.
2. **Physical Unit Typestates**: Compile-time enforcement of physical units (`Meters` vs `Millimeters` vs `Knots`).
3. **Affine Space Geometry**: Compile-time enforcement of affine rules (`Point - Point = Vector`, `Point + Point` fails).
4. **Bare-Metal `#![no_std]` & Zero Cost**: 100% `[T; 3]` primitive memory layout (`#[repr(C)]`) with zero heap allocations (`no_alloc`).

| Library / Pattern                      |   Spatial Frame Typestates?    |     Physical Unit Safety?     |     Affine Geometry Enforcement?      |         Bare-Metal `#![no_std]` Zero Cost?          | Rationale for Selection / Rejection                                                                                                                                              |
| :------------------------------------- | :----------------------------: | :---------------------------: | :-----------------------------------: | :-------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`euclid`** (Mozilla)                 |  **Partial** (Generic spaces)  |     **No** (Raw scalars)      |                **Yes**                |            **Partial** (Graphics focus)             | Focuses on 2D/3D graphics viewports; lacks physical unit typestates and aerospace $SO(3)$ Euler/DCM math.                                                                        |
| **`sguaba`**                           |    **Yes** (ECEF, NED, FRD)    | **No** (Implicit meters/rad)  |                **Yes**                |           **No** (Heavy `nalgebra` tree)            | Provides spatial frame coordinate systems, but lacks physical unit typestates (`Meters` vs `Millimeters`) and requires heavy linear algebra dependencies (`nalgebra`).           |
| **`uom`** (Units)                      |    **No** (Frame agnostic)     |     **Yes** (Dimensions)      |         **No** (Scalars only)         |                       **Yes**                       | Handles scalar physical units, but cannot track 3D spatial frames or vector rotations.                                                                                           |
| **Untyped Math (`nalgebra` / `glam`)** |     **No** (Untyped math)      |     **No** (Raw scalars)      |                **No**                 |        **Partial** (Heavy dependency trees)         | Requires manual ZST wrapper boilerplate per project, introducing compile overhead.                                                                                               |
| **`frame-sc` (Proposed)**              | **Yes** (Type-safe ZST frames) | **Yes** (Type-safe ZST units) | **Yes** (Affine `Point3` / `Vector3`) | **Yes** (`#![no_std]`, `#[repr(C)]`, 0.0% overhead) | **SELECTED**: Simultaneously satisfies frame typestates, unit typestates, affine geometry, high-level pose/quaternion domain abstractions, and `#![no_std]` zero-cost execution. |

### Detailed Evaluation Rationale

1. **`euclid`**:
   - _Strengths:_ Excellent phantom type tracking for 2D/3D graphics spaces (`Point3D<T, Space>`, `Vector3D<T, Space>`).
   - _Why Rejected:_ Lacks physical unit typestates (`Meters` vs `Feet` vs `Millimeters`) and focuses on 4x4 homogeneous matrices for graphics rather than aerospace Direction Cosine Matrices ($SO(3)$) or quaternions.

2. **`sguaba`**:
   - _Strengths:_ Provides strongly-typed spatial math and coordinate reference frames (`ECEF`, `NED`, `FRD`, `WGS84`).
   - _Why Rejected:_ `sguaba` does **not** track physical unit typestates (`Meters` vs `Millimeters` vs `Knots`) within type signatures to catch scaling errors. Additionally, its underlying math relies on `nalgebra` abstractions rather than lightweight, zero-dependency `#![no_std]` array wrappers.

3. **`uom`**:
   - _Strengths:_ Standard for scalar dimensional analysis in Rust.
   - _Why Rejected:_ `uom` tracks _dimensions_ (Length, Time, Mass), but is completely agnostic to _spatial reference frames_. A `Length` in `uom` can be added to any other `Length` regardless of frame orientation (`BodyFrame` vs `LocalNed`), providing zero protection against frame mixing bugs.

---
