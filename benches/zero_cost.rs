use criterion::{black_box, criterion_group, criterion_main, Criterion};
use frame_sc::{vec3, BodyFrame, DirectionCosineMatrix, LocalNed, Meters, Vector3};

// Raw f32 array implementations
fn raw_vector_add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn raw_vector_dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn raw_dcm_rotate(dcm: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        dcm[0][0] * v[0] + dcm[0][1] * v[1] + dcm[0][2] * v[2],
        dcm[1][0] * v[0] + dcm[1][1] * v[1] + dcm[1][2] * v[2],
        dcm[2][0] * v[0] + dcm[2][1] * v[1] + dcm[2][2] * v[2],
    ]
}

fn bench_addition(c: &mut Criterion) {
    let mut group = c.benchmark_group("Vector Addition");

    let raw_a = [12.5f32, -3.4, 8.1];
    let raw_b = [-4.2f32, 9.8, 1.2];

    let typed_a: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 12.5, -3.4, 8.1];
    let typed_b: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, -4.2, 9.8, 1.2];

    group.bench_function("raw_f32_array", |b| {
        b.iter(|| raw_vector_add(black_box(raw_a), black_box(raw_b)))
    });

    group.bench_function("frame_sc_vector3", |b| {
        b.iter(|| black_box(typed_a) + black_box(typed_b))
    });

    group.finish();
}

fn bench_dot_product(c: &mut Criterion) {
    let mut group = c.benchmark_group("Vector Dot Product");

    let raw_a = [12.5f32, -3.4, 8.1];
    let raw_b = [-4.2f32, 9.8, 1.2];

    let typed_a: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, 12.5, -3.4, 8.1];
    let typed_b: Vector3<BodyFrame, Meters> = vec3![BodyFrame, Meters, -4.2, 9.8, 1.2];

    group.bench_function("raw_f32_array", |b| {
        b.iter(|| raw_vector_dot(black_box(raw_a), black_box(raw_b)))
    });

    group.bench_function("frame_sc_vector3", |b| {
        b.iter(|| black_box(typed_a).dot(black_box(typed_b)))
    });

    group.finish();
}

fn bench_dcm_rotation(c: &mut Criterion) {
    let mut group = c.benchmark_group("DCM Frame Rotation");

    let raw_dcm = [
        [0.8660254f32, 0.5, 0.0],
        [-0.5, 0.8660254, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let raw_v = [100.0f32, -50.0, 25.0];

    let typed_dcm = DirectionCosineMatrix::<LocalNed, BodyFrame>::new(raw_dcm);
    let typed_v: Vector3<LocalNed, Meters> = vec3![LocalNed, Meters, 100.0, -50.0, 25.0];

    group.bench_function("raw_f32_array", |b| {
        b.iter(|| raw_dcm_rotate(black_box(raw_dcm), black_box(raw_v)))
    });

    group.bench_function("frame_sc_vector3", |b| {
        b.iter(|| typed_dcm.rotate_vector(black_box(typed_v)))
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_addition,
    bench_dot_product,
    bench_dcm_rotation
);
criterion_main!(benches);
