//! Vectors in plain numbers, `[f64; N]`: for what is only ever a free
//! choice or a picture — a section placed along a spline, a sample to draw,
//! a point to suggest — never for a result the kernel reasons about, which
//! is a [`crate::vector::Vector`] of scalars.
//!
//! Where a direction has no length, [`normalize`] gives none; what to do
//! then is the caller's choice, made where it is called.

/// `[x, y]`.
pub type P2 = [f64; 2];

/// `[x, y, z]`.
pub type P3 = [f64; 3];

pub fn add<const N: usize>(a: [f64; N], b: [f64; N]) -> [f64; N] {
    std::array::from_fn(|i| a[i] + b[i])
}

pub fn sub<const N: usize>(a: [f64; N], b: [f64; N]) -> [f64; N] {
    std::array::from_fn(|i| a[i] - b[i])
}

pub fn scale<const N: usize>(a: [f64; N], s: f64) -> [f64; N] {
    a.map(|x| x * s)
}

pub fn dot<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    (1..N).fold(a[0] * b[0], |sum, i| sum + a[i] * b[i])
}

/// The length of `a`, `sqrt(a . a)`.
pub fn norm<const N: usize>(a: [f64; N]) -> f64 {
    dot(a, a).sqrt()
}

/// How far `a` is from `b`.
pub fn distance<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    norm(sub(a, b))
}

/// `a` scaled to unit length, or `None` if it has no finite, non-zero one.
pub fn normalize<const N: usize>(a: [f64; N]) -> Option<[f64; N]> {
    let n = norm(a);
    (n > 0.0 && n.is_finite()).then(|| scale(a, 1.0 / n))
}

/// The cross product `a x b`.
pub fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The 2-D cross product `a.x b.y - a.y b.x`: positive where `b` lies
/// counter-clockwise of `a`.
pub fn cross2(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Twice the signed area of the closed polygon `poly` (shoelace formula):
/// positive where it runs counter-clockwise.
pub fn signed_area2(poly: &[P2]) -> f64 {
    (0..poly.len())
        .map(|i| cross2(poly[i], poly[(i + 1) % poly.len()]))
        .sum()
}

/// The solution `x` of `m x = r` for the 2x2 matrix `m`, by Cramer's rule:
/// not finite where `m` is singular.
pub fn solve2(m: [P2; 2], r: P2) -> P2 {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    [
        (m[1][1] * r[0] - m[0][1] * r[1]) / det,
        (m[0][0] * r[1] - m[1][0] * r[0]) / det,
    ]
}

/// The inverse of the 2x2 matrix `m`: not finite where it is singular.
pub fn inverse2(m: [P2; 2]) -> [P2; 2] {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    [
        [m[1][1] / det, -m[0][1] / det],
        [-m[1][0] / det, m[0][0] / det],
    ]
}

/// The angle `a`, turned by whole turns to within half a turn of
/// `reference`: in `[reference - π, reference + π)`, how an angle measured
/// in `(-π, π]` is followed on from the one before.
pub fn unwrap_angle(a: f64, reference: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    reference + (a - reference + PI).rem_euclid(TAU) - PI
}

/// The length of the diagonal of the box from `lo` to `hi`: how far apart
/// any two points in it can be.
pub fn diagonal<const N: usize>(lo: [f64; N], hi: [f64; N]) -> f64 {
    (0..N).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt()
}
