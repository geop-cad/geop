//! [`P2`]: a point, or a vector, in a sketch's plane, as plain numbers —
//! what a sketch stores its positions as and draws its curves with — and
//! the arithmetic on it. The solver's own differentiable geometry is
//! [`crate::geometry`].

/// `[x, y]`.
pub type P2 = [f64; 2];

pub fn add(a: P2, b: P2) -> P2 {
    [a[0] + b[0], a[1] + b[1]]
}

pub fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub fn scale(a: P2, s: f64) -> P2 {
    [a[0] * s, a[1] * s]
}

pub fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub fn dist(a: P2, b: P2) -> f64 {
    let d = sub(a, b);
    d[0].hypot(d[1])
}

/// The point a fraction `t` of the way from `a` to `b`.
pub fn lerp(a: P2, b: P2, t: f64) -> P2 {
    add(a, scale(sub(b, a), t))
}
