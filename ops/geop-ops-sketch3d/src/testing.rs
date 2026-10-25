//! The 3-D sketches other crates' tests pick from (feature `testing`).

use geop_core_math::{scalars::Scalar, vector::Vector3};

use crate::AddSketch3dArgs;
use geop_ops_model::sketch3d::Sketch3d;

/// The 3-D sketch `route`: lines from `(1, 1, 1)` to `(3, 1, 1)` and on to
/// `(3, 2, 2)` — its points `p0`, `p1` and `p3`, its lines `c2` and `c4`.
pub fn route() -> AddSketch3dArgs {
    let mut s = Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
    let a = s.add_point(at([1.0, 1.0, 1.0]));
    let b = s.add_point(at([3.0, 1.0, 1.0]));
    s.add_line(a, b);
    let c = s.add_point(at([3.0, 2.0, 2.0]));
    s.add_line(b, c);
    AddSketch3dArgs {
        sketch: s,
        references: Vec::new(),
    }
}
