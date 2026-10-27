//! Plain `f64` vectors and quaternions, for what the window draws and
//! where the camera is: the kernel's own interval types are for geometry,
//! not for pixels.

pub type V3 = [f64; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn length(a: V3) -> f64 {
    dot(a, a).sqrt()
}

pub fn distance(a: V3, b: V3) -> f64 {
    length(sub(a, b))
}

/// `a` of unit length, or `fallback` where it has none.
pub fn normalized_or(a: V3, fallback: V3) -> V3 {
    let l = length(a);
    if l > 0.0 && l.is_finite() {
        scale(a, 1.0 / l)
    } else {
        fallback
    }
}

pub fn normalized(a: V3) -> V3 {
    normalized_or(a, [0.0, 0.0, 1.0])
}

pub fn lerp(a: V3, b: V3, t: f64) -> V3 {
    add(a, scale(sub(b, a), t))
}

pub fn neg(a: V3) -> V3 {
    [-a[0], -a[1], -a[2]]
}

/// `p` turned by `angle` radians about the unit `axis` through the origin
/// (Rodrigues).
pub fn rotate(p: V3, axis: V3, angle: f64) -> V3 {
    let (s, c) = angle.sin_cos();
    add(
        add(scale(p, c), scale(cross(axis, p), s)),
        scale(axis, dot(axis, p) * (1.0 - c)),
    )
}

/// A rotation, as a unit quaternion `(x, y, z, w)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat(pub [f64; 4]);

impl Quat {
    /// The rotation taking the camera's own axes — `x` right, `y` up, `z`
    /// back towards the eye — onto `right`, `up`, `back`.
    pub fn from_basis(right: V3, up: V3, back: V3) -> Self {
        let (m00, m01, m02) = (right[0], up[0], back[0]);
        let (m10, m11, m12) = (right[1], up[1], back[1]);
        let (m20, m21, m22) = (right[2], up[2], back[2]);
        let trace = m00 + m11 + m22;
        let q = if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            [(m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s]
        } else if m00 > m11 && m00 > m22 {
            let s = 2.0 * (1.0 + m00 - m11 - m22).sqrt();
            [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]
        } else if m11 > m22 {
            let s = 2.0 * (1.0 + m11 - m00 - m22).sqrt();
            [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s]
        } else {
            let s = 2.0 * (1.0 + m22 - m00 - m11).sqrt();
            [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s]
        };
        Quat(q).normalized()
    }

    fn normalized(self) -> Self {
        let l = self.0.iter().map(|x| x * x).sum::<f64>().sqrt();
        Quat(self.0.map(|x| x / l))
    }

    /// `v` turned by the rotation.
    pub fn apply(&self, v: V3) -> V3 {
        let [x, y, z, w] = self.0;
        let u = [x, y, z];
        let t = scale(cross(u, v), 2.0);
        add(add(v, scale(t, w)), cross(u, t))
    }

    /// The rotation `t` of the way from `self` to `to`, the short way.
    pub fn slerp(&self, to: &Quat, t: f64) -> Quat {
        let a = self.0;
        let mut b = to.0;
        let mut cos = (0..4).map(|k| a[k] * b[k]).sum::<f64>();
        if cos < 0.0 {
            b = b.map(|x| -x);
            cos = -cos;
        }
        if cos > 0.9995 {
            return Quat(std::array::from_fn(|k| a[k] + (b[k] - a[k]) * t)).normalized();
        }
        let theta = cos.acos();
        let (wa, wb) = (
            ((1.0 - t) * theta).sin() / theta.sin(),
            (t * theta).sin() / theta.sin(),
        );
        Quat(std::array::from_fn(|k| a[k] * wa + b[k] * wb)).normalized()
    }
}

/// A 4 × 4 matrix, column-major as the GPU takes it.
pub type Mat4 = [[f32; 4]; 4];

/// `a · b`.
pub fn mul(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for (c, column) in out.iter_mut().enumerate() {
        for (r, x) in column.iter_mut().enumerate() {
            *x = (0..4).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

pub fn to_f32(m: &[[f64; 4]; 4]) -> Mat4 {
    m.map(|c| c.map(|x| x as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: V3, b: V3) -> bool {
        distance(a, b) < 1e-9
    }

    #[test]
    fn a_basis_is_turned_onto_by_its_quaternion() {
        let right = normalized([1.0, 1.0, 0.0]);
        let back = [0.0, 0.0, 1.0];
        let up = cross(back, right);
        let q = Quat::from_basis(right, up, back);
        assert!(near(q.apply([1.0, 0.0, 0.0]), right));
        assert!(near(q.apply([0.0, 1.0, 0.0]), up));
        let half =
            Quat::from_basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]).slerp(&q, 0.5);
        assert!(near(
            half.apply([1.0, 0.0, 0.0]),
            normalized([1.0, (22.5f64).to_radians().tan(), 0.0])
        ));
    }

    #[test]
    fn rotating_about_an_axis_keeps_what_lies_on_it() {
        assert!(near(
            rotate([0.0, 0.0, 2.0], [0.0, 0.0, 1.0], 1.0),
            [0.0, 0.0, 2.0]
        ));
        assert!(near(
            rotate(
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                std::f64::consts::FRAC_PI_2
            ),
            [0.0, 1.0, 0.0]
        ));
    }
}
