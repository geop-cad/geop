//! Benchmark `intersection::curve_curve_bisect` / `curve_surface_bisect` (hull tests +
//! bisection, coincidence read from hitting `max_solutions`) against the
//! wrappers in `curve_curve` / `curve_surface` (fat line clipping,
//! coincidence found directly) on the same pairs, transversal and
//! coincident.
//!
//! Run with `cargo run --release -p geop-core-geometry --example intersection_bench`.

use std::hint::black_box;
use std::time::{Duration, Instant};

use geop_core_geometry::{
    intersection::{self, Intersections},
    nurb_curve::NurbCurve,
    nurb_surface::NurbSurface,
};
use geop_core_math::{
    geop_error::GeopResult,
    scalars::{ScalInF64, ScalInFPA64, Scalar},
    vector::{Vector2, Vector4},
};

const MAX_NODES: usize = 20000;
const MAX_SOLUTIONS: usize = 16;
const MIN_SIZE: f64 = 1e-6;
const REPS: usize = 10;

/// Homogeneous control point for Cartesian `(x, y, z)` with weight `w`.
fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
    Vector4::from_array([
        S::from_f64(x * w),
        S::from_f64(y * w),
        S::from_f64(z * w),
        S::from_f64(w),
    ])
}

fn clamped_uniform_knots<S: Scalar>(n_pts: usize, degree: usize) -> Vec<S> {
    let spans = n_pts - degree;
    let mut knots = vec![S::ZERO; degree + 1];
    for i in 1..spans {
        knots.push(S::from_f64(i as f64 / spans as f64));
    }
    knots.extend(std::iter::repeat_n(S::ONE, degree + 1));
    knots
}

fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve<S, 4> {
    NurbCurve::try_new(
        1,
        vec![pt(a[0], a[1], a[2], 1.), pt(b[0], b[1], b[2], 1.)],
        clamped_uniform_knots(2, 1),
    )
    .unwrap()
}

/// Cubic B-spline through `pts` (unit weights, clamped uniform knots).
fn cubic<S: Scalar>(pts: &[[f64; 3]]) -> NurbCurve<S, 4> {
    NurbCurve::try_new(
        3,
        pts.iter().map(|p| pt(p[0], p[1], p[2], 1.)).collect(),
        clamped_uniform_knots(pts.len(), 3),
    )
    .unwrap()
}

fn quarter_circle<S: Scalar>() -> NurbCurve<S, 4> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    NurbCurve::try_new(
        2,
        vec![pt(1., 0., 0., 1.), pt(1., 1., 0., w), pt(0., 1., 0., 1.)],
        clamped_uniform_knots(3, 2),
    )
    .unwrap()
}

fn lifted<S: Scalar>() -> NurbSurface<S, 4> {
    NurbSurface::try_new(
        1,
        1,
        vec![
            pt(0., 0., 0., 1.),
            pt(0., 1., 0., 1.),
            pt(1., 0., 0., 1.),
            pt(1., 1., 1., 1.),
        ],
        clamped_uniform_knots(2, 1),
        clamped_uniform_knots(2, 1),
    )
    .unwrap()
}

fn sphere_octant<S: Scalar>() -> NurbSurface<S, 4> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let circle = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
    NurbSurface::try_new(
        2,
        2,
        circle
            .iter()
            .flat_map(|&(x, y, wu)| {
                circle
                    .iter()
                    .map(move |&(r, z, wv)| pt(x * r, y * r, z, wu * wv))
            })
            .collect(),
        clamped_uniform_knots(3, 2),
        clamped_uniform_knots(3, 2),
    )
    .unwrap()
}

/// Rational bicubic, 8×8 net (5×5 spans), wavy in z over [0, 7]².
fn wavy<S: Scalar>() -> NurbSurface<S, 4> {
    NurbSurface::try_new(
        3,
        3,
        (0..8)
            .flat_map(|i| {
                (0..8).map(move |j| {
                    let z = ((i * 3 + j * 5) % 7) as f64 / 7.0 - 0.5;
                    pt(i as f64, j as f64, z, 1.0 + 0.25 * ((i + j) % 3) as f64)
                })
            })
            .collect(),
        clamped_uniform_knots(8, 3),
        clamped_uniform_knots(8, 3),
    )
    .unwrap()
}

/// Planar cubic wiggling across `y = 0` along x ∈ [0, 7].
fn wiggle<S: Scalar>(sign: f64) -> NurbCurve<S, 4> {
    cubic(
        &(0..8)
            .map(|i| [i as f64, sign * if i % 2 == 0 { 1. } else { -1. }, 0.])
            .collect::<Vec<_>>(),
    )
}

/// Radius-1 quarter circle in the plane y = 0.3, centred at (0, 0.3, -0.2):
/// meets the unit sphere once, at x ≈ 0.946, z ≈ 0.125.
fn tilted_arc<S: Scalar>() -> NurbCurve<S, 4> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    NurbCurve::try_new(
        2,
        vec![
            pt(1., 0.3, -0.2, 1.),
            pt(1., 0.3, 0.8, w),
            pt(0., 0.3, 0.8, 1.),
        ],
        clamped_uniform_knots(3, 2),
    )
    .unwrap()
}

/// The unit sphere octant's `u = 0` boundary: the quarter meridian in the
/// xz-plane from the equator to the pole.
fn meridian<S: Scalar>() -> NurbCurve<S, 4> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    NurbCurve::try_new(
        2,
        vec![pt(1., 0., 0., 1.), pt(1., 0., 1., w), pt(0., 0., 1., 1.)],
        clamped_uniform_knots(3, 2),
    )
    .unwrap()
}

/// A 90° arc of the unit equator starting at `start` degrees.
fn equator_arc_from<S: Scalar>(start: f64) -> NurbCurve<S, 4> {
    let (a0, am, a1) = (
        start.to_radians(),
        (start + 45.).to_radians(),
        (start + 90.).to_radians(),
    );
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let r = std::f64::consts::SQRT_2;
    NurbCurve::try_new(
        2,
        vec![
            pt(a0.cos(), a0.sin(), 0., 1.),
            pt(r * am.cos(), r * am.sin(), 0., w),
            pt(a1.cos(), a1.sin(), 0., 1.),
        ],
        clamped_uniform_knots(3, 2),
    )
    .unwrap()
}

/// What a pair's correct answer is.
#[derive(Clone, Copy, PartialEq)]
enum Expect {
    /// `Found` with exactly this many crossings.
    Hits(usize),
    /// `Found`, count not pinned down.
    Some,
    /// `Coincident`.
    Overlap,
}

/// One pair to intersect: whether the result was `Coincident`, and the
/// widest parameter width of every returned box.
type Query<'a> = Box<dyn Fn() -> GeopResult<(bool, Vec<f64>)> + 'a>;

struct Outcome {
    time: Duration,
    /// `Ok(coincident, count, avg box width)`, or the error text.
    result: Result<(bool, usize, f64), String>,
}

fn measure(q: &Query) -> Outcome {
    let result = match q() {
        Ok((coincident, widths)) => {
            let avg = if widths.is_empty() {
                0.0
            } else {
                widths.iter().sum::<f64>() / widths.len() as f64
            };
            Ok((coincident, widths.len(), avg))
        }
        Err(e) => Err(format!("{e}")),
    };
    let start = Instant::now();
    for _ in 0..REPS {
        let _ = black_box(q());
    }
    Outcome {
        time: start.elapsed() / REPS as u32,
        result,
    }
}

/// `"3"` for `Found` with 3, `"C16"` for `Coincident` with 16, `"err"`.
fn describe(o: &Outcome) -> (String, String) {
    match &o.result {
        Ok((c, n, w)) => (
            format!("{}{n}", if *c { "C" } else { "" }),
            format!("{w:.1e}"),
        ),
        Err(_) => ("err".into(), "-".into()),
    }
}

fn correct(o: &Outcome, expect: Expect) -> bool {
    match (&o.result, expect) {
        (Ok((false, n, _)), Expect::Hits(e)) => *n == e,
        (Ok((false, _, _)), Expect::Some) => true,
        (Ok((true, _, _)), Expect::Overlap) => true,
        _ => false,
    }
}

type CurveCurveFn<S> =
    fn(&NurbCurve<S, 4>, &NurbCurve<S, 4>, usize, usize, S) -> GeopResult<Intersections<(S, S)>>;

type CurveSurfaceFn<S> = fn(
    &NurbCurve<S, 4>,
    &NurbSurface<S, 4>,
    usize,
    usize,
    S,
) -> GeopResult<Intersections<(S, Vector2<S>)>>;

fn cc_query<S: Scalar>(
    f: CurveCurveFn<S>,
    a: NurbCurve<S, 4>,
    b: NurbCurve<S, 4>,
) -> Query<'static> {
    Box::new(move || {
        let r = f(&a, &b, MAX_SOLUTIONS, MAX_NODES, S::from_f64(MIN_SIZE))?;
        let widths = r
            .as_slice()
            .iter()
            .map(|(s, t)| s.width().to_f64().max(t.width().to_f64()))
            .collect();
        Ok((r.is_coincident(), widths))
    })
}

fn cs_query<S: Scalar>(
    f: CurveSurfaceFn<S>,
    c: NurbCurve<S, 4>,
    s: NurbSurface<S, 4>,
) -> Query<'static> {
    Box::new(move || {
        let r = f(&c, &s, MAX_SOLUTIONS, MAX_NODES, S::from_f64(MIN_SIZE))?;
        let widths = r
            .as_slice()
            .iter()
            .map(|(t, uv)| {
                t.width()
                    .to_f64()
                    .max(uv[0].width().to_f64())
                    .max(uv[1].width().to_f64())
            })
            .collect();
        Ok((r.is_coincident(), widths))
    })
}

/// `(name, expected, old, new)`.
fn cases<S: Scalar>() -> Vec<(&'static str, Expect, Query<'static>, Query<'static>)> {
    use intersection::{curve_curve, curve_curve_bisect, curve_surface, curve_surface_bisect};
    let mut v = Vec::new();
    let mut cc = |name, e, a: NurbCurve<S, 4>, b: NurbCurve<S, 4>| {
        v.push((
            name,
            e,
            cc_query(
                curve_curve_bisect::curve_curve_intersect::<S, 4, 3>,
                a.clone(),
                b.clone(),
            ),
            cc_query(curve_curve::curve_curve_intersect::<S, 4, 3>, a, b),
        ));
    };
    let arc = quarter_circle::<S>();
    cc(
        "cc: crossing lines",
        Expect::Hits(1),
        line([0., 0., 0.], [1., 1., 0.]),
        line([0., 1., 0.], [1., 0., 0.]),
    );
    cc(
        "cc: shared endpoint",
        Expect::Hits(1),
        line([0., 0., 0.], [1., 0., 0.]),
        line([1., 0., 0.], [1., 1., 1.]),
    );
    cc(
        "cc: arc x chord (2)",
        Expect::Hits(2),
        arc.clone(),
        line([1., 0.15, 0.], [0.15, 1., 0.]),
    );
    cc(
        "cc: arc, ends on chord (2)",
        Expect::Hits(2),
        arc.clone(),
        line([1., 0., 0.], [0., 1., 0.]),
    );
    cc(
        "cc: cubic x line (7)",
        Expect::Hits(7),
        wiggle(1.),
        line([-1., 0.05, 0.], [8., 0.05, 0.]),
    );
    cc(
        "cc: cubic x cubic (7)",
        Expect::Hits(7),
        wiggle(1.),
        wiggle(-1.),
    );
    cc(
        "cc: skew, 1e-3 apart",
        Expect::Hits(0),
        line([0., 0., 0.], [1., 1., 0.]),
        line([0., 1., 1e-3], [1., 0., 1e-3]),
    );
    cc(
        "cc: identical lines",
        Expect::Overlap,
        line([0., 0., 0.], [1., 1., 0.]),
        line([0., 0., 0.], [1., 1., 0.]),
    );
    cc(
        "cc: lines, partial overlap",
        Expect::Overlap,
        line([0., 0., 0.], [1., 0., 0.]),
        line([0.5, 0., 0.], [1.5, 0., 0.]),
    );
    cc(
        "cc: arc pieces, overlap",
        Expect::Overlap,
        arc.split(S::from_f64(0.75)).unwrap().0,
        arc.split(S::from_f64(0.25)).unwrap().1,
    );
    cc(
        "cc: cubic, reversed piece",
        Expect::Overlap,
        wiggle(1.),
        wiggle::<S>(1.)
            .sub_curve(S::from_f64(0.2), S::from_f64(0.9))
            .unwrap()
            .reverse(),
    );

    let mut cs = |name, e, c: NurbCurve<S, 4>, s: NurbSurface<S, 4>| {
        v.push((
            name,
            e,
            cs_query(
                curve_surface_bisect::curve_surface_intersect,
                c.clone(),
                s.clone(),
            ),
            cs_query(curve_surface::curve_surface_intersect, c, s),
        ));
    };
    cs(
        "cs: line x bilinear",
        Expect::Hits(1),
        line([0.2, 0.7, -1.], [0.9, 0.3, 2.]),
        lifted(),
    );
    cs(
        "cs: line ends on bilinear",
        Expect::Hits(1),
        line([0.3, 0.6, 1.], [0.3, 0.6, 0.18]),
        lifted(),
    );
    cs(
        "cs: line x sphere",
        Expect::Hits(1),
        line([0., 0., 0.], [1., 1., 1.]),
        sphere_octant(),
    );
    cs(
        "cs: tilted arc x sphere",
        Expect::Hits(1),
        tilted_arc(),
        sphere_octant(),
    );
    cs(
        "cs: line x bicubic",
        Expect::Hits(1),
        line([2.3, 4.1, -2.], [2.6, 3.7, 2.]),
        wavy(),
    );
    cs(
        "cs: cubic x bicubic",
        Expect::Some,
        cubic(&[
            [0.5, 0.5, -1.],
            [2., 6., 1.],
            [4., 1., -1.],
            [5., 6., 1.],
            [6.5, 3., -1.],
        ]),
        wavy(),
    );
    cs(
        "cs: near miss sphere",
        Expect::Hits(0),
        line([1.001, 0., 0.1], [1.001, 0.5, 0.1]),
        sphere_octant(),
    );
    cs(
        "cs: equator on sphere",
        Expect::Overlap,
        arc.clone(),
        sphere_octant(),
    );
    cs(
        "cs: diagonal ends on bilinear (2)",
        Expect::Hits(2),
        line([0., 0., 0.], [1., 1., 1.]),
        lifted(),
    );
    cs(
        "cs: bilinear ruling line",
        Expect::Overlap,
        line([0.3, 0., 0.], [0.3, 1., 0.3]),
        lifted(),
    );
    cs(
        "cs: line along patch edge",
        Expect::Overlap,
        line([0.2, 0., 0.], [0.8, 0., 0.]),
        lifted(),
    );
    cs(
        "cs: meridian along sphere seam",
        Expect::Overlap,
        meridian(),
        sphere_octant(),
    );
    cs(
        "cs: arc half on sphere",
        Expect::Overlap,
        equator_arc_from(-45.),
        sphere_octant(),
    );
    v
}

fn bench<S: Scalar>(scalar_name: &str) {
    println!(
        "\n=== {scalar_name} (max_nodes={MAX_NODES}, max_solutions={MAX_SOLUTIONS}, \
         min_subdivision_size={MIN_SIZE:e}) ==="
    );
    println!(
        "{:<31} {:>11} {:>11} {:>9}   {:>11} {:>19}",
        "pair", "old [µs]", "new [µs]", "speedup", "result o/n", "avg width o/n"
    );
    let mut totals = [[Duration::ZERO; 2]; 2];
    for (name, expect, old, new) in cases::<S>() {
        let (o, n) = (measure(&old), measure(&new));
        let (or, ow) = describe(&o);
        let (nr, nw) = describe(&n);
        assert!(correct(&n, expect), "{name}: new result {nr} is wrong");
        let group = usize::from(expect == Expect::Overlap);
        totals[group][0] += o.time;
        totals[group][1] += n.time;
        let mark = |ok: bool| if ok { "" } else { "!" };
        println!(
            "{:<31} {:>11.1} {:>11.1} {:>8.1}x   {:>11} {:>19}",
            name,
            o.time.as_secs_f64() * 1e6,
            n.time.as_secs_f64() * 1e6,
            o.time.as_secs_f64() / n.time.as_secs_f64(),
            format!("{or}{}/{nr}", mark(correct(&o, expect))),
            format!("{ow}/{nw}"),
        );
    }
    for (label, [old, new]) in ["transversal", "coincident"].into_iter().zip(totals) {
        println!(
            "total {label}: old {:.2} ms, new {:.2} ms, speedup {:.1}x",
            old.as_secs_f64() * 1e3,
            new.as_secs_f64() * 1e3,
            old.as_secs_f64() / new.as_secs_f64()
        );
    }
    println!("(`C` = Coincident, `!` = old result wrong)");
}

fn main() {
    bench::<ScalInF64>("ScalInF64");
    bench::<ScalInFPA64>("ScalInFPA64");
}
