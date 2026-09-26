//! Benchmark `contains::curve_bisect` (hull test + bisection) against
//! `contains::curve` (fat line clipping) on the same queries.
//!
//! Run with `cargo run --release -p geop-core-geometry --example curve_contains_bench`.

use std::hint::black_box;
use std::time::{Duration, Instant};

use geop_core_geometry::{contains, nurb_curve::NurbCurve};
use geop_core_math::{
    scalars::{ScalInF64, ScalInFPA64, Scalar},
    vector::{Vector3, Vector4},
};

const MAX_NODES: usize = 2000;
const MIN_SIZE: f64 = 1e-6;
const REPS: usize = 20;

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

fn curves<S: Scalar>() -> Vec<(&'static str, NurbCurve<S, 4>)> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let line = NurbCurve::try_new(
        1,
        vec![pt(0., 0., 0., 1.), pt(1., 0.3, 0.2, 1.)],
        clamped_uniform_knots(2, 1),
    )
    .unwrap();
    let quarter_circle = NurbCurve::try_new(
        2,
        vec![pt(1., 0., 0., 1.), pt(1., 1., 0., w), pt(0., 1., 0., 1.)],
        clamped_uniform_knots(3, 2),
    )
    .unwrap();
    let cubic = NurbCurve::try_new(
        3,
        vec![
            pt(0., 0., 0., 1.),
            pt(1., 2., 0., 1.),
            pt(2., -1., 1., 1.),
            pt(3., 2., 0., 1.),
            pt(4., 0., -1., 1.),
            pt(5., 1., 0., 1.),
        ],
        clamped_uniform_knots(6, 3),
    )
    .unwrap();
    // Helix-like cubic B-spline with many spans.
    let helix_pts: Vec<_> = (0..24)
        .map(|i| {
            let a = i as f64 * 0.6;
            pt(a.cos(), a.sin(), i as f64 * 0.1, 1.0 + 0.3 * (i % 3) as f64)
        })
        .collect();
    let helix = NurbCurve::try_new(3, helix_pts, clamped_uniform_knots(24, 3)).unwrap();
    vec![
        ("line", line),
        ("quarter circle", quarter_circle),
        ("cubic 3 spans", cubic),
        ("rational helix 21 spans", helix),
    ]
}

/// `(kind, point, parameter it was derived from)` queries for `curve`.
fn queries<S: Scalar>(curve: &NurbCurve<S, 4>) -> Vec<(&'static str, Vector3<S>, S)> {
    let mut out = Vec::new();
    for i in 0..=16 {
        let t = S::from_f64(i as f64 / 16.0);
        let p = curve.evaluate(t).unwrap();
        out.push(("on", p, t));
        // Pushed off the curve by 1e-3 along z and y: a near miss.
        let near = Vector3::from_array([
            p[0],
            p[1].add(S::from_f64(1e-3)),
            p[2].add(S::from_f64(1e-3)),
        ]);
        out.push(("near", near, t));
        let far = Vector3::from_array([
            p[0].add(S::from_f64(0.3)),
            p[1].sub(S::from_f64(0.2)),
            p[2].add(S::from_f64(0.5)),
        ]);
        out.push(("far", far, t));
    }
    out
}

type ContainsFn<S> = fn(
    &NurbCurve<S, 4>,
    &Vector3<S>,
    usize,
    S,
) -> geop_core_math::geop_error::GeopResult<Option<S>>;

struct Stats {
    time: Duration,
    found: usize,
    width_sum: f64,
}

fn run<S: Scalar>(
    f: ContainsFn<S>,
    curve: &NurbCurve<S, 4>,
    qs: &[(&str, Vector3<S>, S)],
    kind: &str,
) -> Stats {
    let min_size = S::from_f64(MIN_SIZE);
    let mut stats = Stats {
        time: Duration::ZERO,
        found: 0,
        width_sum: 0.0,
    };
    for (_, p, t_true) in qs.iter().filter(|q| q.0 == kind) {
        let res = f(curve, p, MAX_NODES, min_size).unwrap();
        if kind == "on" {
            let t = res.expect("point on the curve not found");
            assert!(
                t.could_be_equal(*t_true),
                "enclosure {t:?} misses t={t_true:?}"
            );
        }
        if let Some(t) = res {
            stats.found += 1;
            stats.width_sum += t.width().to_f64();
        }
        let start = Instant::now();
        for _ in 0..REPS {
            black_box(f(black_box(curve), black_box(p), MAX_NODES, min_size).unwrap());
        }
        stats.time += start.elapsed() / REPS as u32;
    }
    stats
}

fn bench<S: Scalar>(scalar_name: &str) {
    println!("\n=== {scalar_name} (max_nodes={MAX_NODES}, min_subdivision_size={MIN_SIZE:e}) ===");
    println!(
        "{:<24} {:<5} {:>12} {:>12} {:>8}   {:>9} {:<5} {:>19}",
        "curve", "query", "hull [µs]", "clip [µs]", "speedup", "found h/c", "", "avg width h/c"
    );
    let mut total_hull = Duration::ZERO;
    let mut total_clip = Duration::ZERO;
    for (name, curve) in curves::<S>() {
        let qs = queries(&curve);
        for kind in ["on", "near", "far"] {
            let n = qs.iter().filter(|q| q.0 == kind).count();
            let h = run::<S>(
                contains::curve_bisect::curve_could_contain,
                &curve,
                &qs,
                kind,
            );
            let c = run::<S>(contains::curve::curve_could_contain, &curve, &qs, kind);
            total_hull += h.time;
            total_clip += c.time;
            let avg = |s: &Stats| {
                if s.found == 0 {
                    0.0
                } else {
                    s.width_sum / s.found as f64
                }
            };
            println!(
                "{:<24} {:<5} {:>12.1} {:>12.1} {:>7.1}x   {:>9} {:<5} {:>9.1e}/{:<9.1e}",
                name,
                kind,
                h.time.as_secs_f64() * 1e6 / n as f64,
                c.time.as_secs_f64() * 1e6 / n as f64,
                h.time.as_secs_f64() / c.time.as_secs_f64(),
                format!("{}/{}", h.found, c.found),
                format!("of {n}"),
                avg(&h),
                avg(&c),
            );
        }
    }
    println!(
        "total: hull {:.2} ms, clip {:.2} ms, speedup {:.1}x",
        total_hull.as_secs_f64() * 1e3,
        total_clip.as_secs_f64() * 1e3,
        total_hull.as_secs_f64() / total_clip.as_secs_f64()
    );
}

fn main() {
    bench::<ScalInF64>("ScalInF64");
    bench::<ScalInFPA64>("ScalInFPA64");
}
