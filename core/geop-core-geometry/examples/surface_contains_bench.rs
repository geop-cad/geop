//! Benchmark `contains::surface_bisect` (hull test + bisection) against
//! `contains::surface` (fat line clipping) on the same queries.
//!
//! Run with `cargo run --release -p geop-core-geometry --example surface_contains_bench`.

use std::hint::black_box;
use std::time::{Duration, Instant};

use geop_core_geometry::{contains, nurb_surface::NurbSurface};
use geop_core_math::{
    geop_error::GeopResult,
    scalars::{ScalInF64, ScalInFPA64, Scalar},
    vector::{Vector3, Vector4},
};

const MAX_NODES: usize = 2000;
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

fn surfaces<S: Scalar>() -> Vec<(&'static str, NurbSurface<S, 4>)> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let circle = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];

    let lifted = NurbSurface::try_new(
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
    .unwrap();

    let cylinder = NurbSurface::try_new(
        2,
        1,
        circle
            .iter()
            .flat_map(|&(x, y, wi)| [pt(x, y, 0., wi), pt(x, y, 1., wi)])
            .collect(),
        clamped_uniform_knots(3, 2),
        clamped_uniform_knots(2, 1),
    )
    .unwrap();

    let sphere = NurbSurface::try_new(
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
    .unwrap();

    // Bicubic 8×8 net (5×5 spans), wavy in z, non-uniform weights.
    let wavy = NurbSurface::try_new(
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
    .unwrap();

    vec![
        ("bilinear lifted", lifted),
        ("quarter cylinder", cylinder),
        ("sphere octant (pole)", sphere),
        ("rational bicubic 5x5 spans", wavy),
    ]
}

/// `(kind, point, (u, v) it was derived from)` queries on a 5×5 grid.
fn queries<S: Scalar>(s: &NurbSurface<S, 4>) -> Vec<(&'static str, Vector3<S>, (S, S))> {
    let mut out = Vec::new();
    for i in 0..=4 {
        for j in 0..=4 {
            let (u, v) = (S::from_f64(i as f64 / 4.0), S::from_f64(j as f64 / 4.0));
            let p = s.evaluate(u, v).unwrap();
            out.push(("on", p, (u, v)));
            // 1e-3 off the surface along its normal: a near miss. The normal
            // is undefined at the sphere's pole; skip those.
            if let Ok(n) = s.normal(u, v) {
                out.push(("near", p.add(&n.prod_scalar(S::from_f64(1e-3))), (u, v)));
            }
            let far = Vector3::from_array([
                p[0].add(S::from_f64(0.3)),
                p[1].sub(S::from_f64(0.2)),
                p[2].add(S::from_f64(0.5)),
            ]);
            out.push(("far", far, (u, v)));
        }
    }
    out
}

type ContainsFn<S> = fn(&NurbSurface<S, 4>, &Vector3<S>, usize, S) -> GeopResult<Option<(S, S)>>;

struct Stats {
    time: Duration,
    found: usize,
    width_sum: f64,
}

fn run<S: Scalar>(
    f: ContainsFn<S>,
    s: &NurbSurface<S, 4>,
    qs: &[(&str, Vector3<S>, (S, S))],
    kind: &str,
) -> Stats {
    let min_size = S::from_f64(MIN_SIZE);
    let mut stats = Stats {
        time: Duration::ZERO,
        found: 0,
        width_sum: 0.0,
    };
    for (_, p, (u_true, v_true)) in qs.iter().filter(|q| q.0 == kind) {
        let res = f(s, p, MAX_NODES, min_size).unwrap();
        if kind == "on" {
            let (u, v) = res.expect("point on the surface not found");
            assert!(
                u.could_be_equal(*u_true) && v.could_be_equal(*v_true),
                "({u:?}, {v:?}) misses ({u_true:?}, {v_true:?})"
            );
        }
        if let Some((u, v)) = res {
            stats.found += 1;
            stats.width_sum += u.width().to_f64().max(v.width().to_f64());
        }
        let start = Instant::now();
        for _ in 0..REPS {
            black_box(f(black_box(s), black_box(p), MAX_NODES, min_size).unwrap());
        }
        stats.time += start.elapsed() / REPS as u32;
    }
    stats
}

fn bench<S: Scalar>(scalar_name: &str) {
    println!("\n=== {scalar_name} (max_nodes={MAX_NODES}, min_subdivision_size={MIN_SIZE:e}) ===");
    println!(
        "{:<27} {:<5} {:>12} {:>12} {:>8}   {:>9} {:<5} {:>19}",
        "surface", "query", "hull [µs]", "clip [µs]", "speedup", "found h/c", "", "avg width h/c"
    );
    let mut total_hull = Duration::ZERO;
    let mut total_clip = Duration::ZERO;
    for (name, s) in surfaces::<S>() {
        let qs = queries(&s);
        for kind in ["on", "near", "far"] {
            let n = qs.iter().filter(|q| q.0 == kind).count();
            let h = run::<S>(
                contains::surface_bisect::surface_could_contain,
                &s,
                &qs,
                kind,
            );
            let c = run::<S>(contains::surface::surface_could_contain, &s, &qs, kind);
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
                "{:<27} {:<5} {:>12.1} {:>12.1} {:>7.1}x   {:>9} {:<5} {:>9.1e}/{:<9.1e}",
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
