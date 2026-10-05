//! Network surfaces through curves standing on their own — each a wire of
//! its own — picked in any order and running either way.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::{NurbCurve, NurbCurve3D},
};
use geop_core_math::{
    scalars::{ScalInF64 as S, Scalar},
    vector::{Vector3, Vector4},
};
use geop_core_topology::build::{BodySpec, EdgeSpec};
use geop_ops::{BodyNames, NoFiles, Part};

use super::*;
use crate::boundary::tests::{assert_valid, refused};
use crate::{BoundarySurface, BoundarySurfaceArgs, Thicken, ThickenArgs, ThickenSide};

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |c, i| c * (n - i) as f64 / (i + 1) as f64)
}

/// The Bézier curve on `[0, 1]` whose coordinates are the polynomials with
/// the monomial coefficients `coords`, lowest power first.
fn polynomial(coords: [&[f64]; 3], degree: usize) -> NurbCurve3D<S> {
    let coefficient = |a: &[f64], j: usize| a.get(j).copied().unwrap_or(0.0);
    let control_points = (0..=degree)
        .map(|i| {
            let [x, y, z] = coords.map(|a| {
                (0..=i)
                    .map(|j| binomial(i, j) / binomial(degree, j) * coefficient(a, j))
                    .sum::<f64>()
            });
            Vector4::from_array([x, y, z, 1.0].map(S::from_f64))
        })
        .collect();
    let mut knots = vec![S::ZERO; degree + 1];
    knots.extend(vec![S::ONE; degree + 1]);
    NurbCurve::try_new(degree, control_points, knots).unwrap()
}

/// `curve` added to `part` as a wire of its own, its edge named `name`.
fn wire(part: &mut Part<S>, name: &str, curve: NurbCurve3D<S>) -> EntityRef {
    let (t0, t1) = curve.domain();
    let spec = BodySpec {
        vertices: vec![curve.evaluate(t0).unwrap(), curve.evaluate(t1).unwrap()],
        edges: vec![EdgeSpec {
            curve,
            start: 0,
            end: 1,
        }],
        faces: Vec::new(),
        shells: Vec::new(),
        solid: false,
    };
    let names = BodyNames {
        vertices: vec![format!("{name}.start"), format!("{name}.end")],
        edges: vec![name.into()],
        faces: Vec::new(),
        solid: None,
    };
    part.build_body(spec, names).unwrap();
    EntityRef::Edge { name: name.into() }
}

/// Curves on `z = lift (0.3 x² - 0.2 y² + 0.1 x y)`: along `x` from 0 to
/// 2 at `y = 0, 1, 2`, the middle one at a speed of its own
/// (`x = t + t²`); and along `y`, from -0.5 on past the grid to 2.5, at
/// `x = 0, 0.7, 2`.
fn grid_curves(lift: f64) -> (Vec<NurbCurve3D<S>>, Vec<NurbCurve3D<S>>) {
    let z = |coefficients: &[f64]| coefficients.iter().map(|c| c * lift).collect::<Vec<_>>();
    let u = [0.0, 1.0, 2.0]
        .iter()
        .map(|&y| {
            if y == 1.0 {
                let height = z(&[-0.2 * y * y, 0.1 * y, 0.3 + 0.1 * y, 0.6, 0.3]);
                polynomial([&[0., 1., 1.], &[y], &height], 4)
            } else {
                polynomial([&[0., 2.], &[y], &z(&[-0.2 * y * y, 0.2 * y, 1.2])], 2)
            }
        })
        .collect();
    // y = -0.5 + 3 s.
    let v = [0.0, 0.7, 2.0]
        .iter()
        .map(|&x| {
            let height = z(&[
                0.3 * x * x - 0.2 * 0.25 - 0.1 * x * 0.5,
                -0.2 * 2. * -0.5 * 3. + 0.1 * x * 3.,
                -0.2 * 9.,
            ]);
            polynomial([&[x], &[-0.5, 3.], &height], 2)
        })
        .collect();
    (u, v)
}

/// A part with the grid's curves as wires, `u0`… and `v0`…, each turned
/// round where `turned` names it, and the references to them in the order
/// `u_order`, `v_order`.
fn grid(
    lift: f64,
    turned: &[&str],
    u_order: [usize; 3],
    v_order: [usize; 3],
) -> (Part<S>, Vec<EntityRef>, Vec<EntityRef>) {
    let mut part = Part::new();
    let (u, v) = grid_curves(lift);
    let mut add = |prefix: &str, curves: Vec<NurbCurve3D<S>>| -> Vec<EntityRef> {
        curves
            .into_iter()
            .enumerate()
            .map(|(k, c)| {
                let name = format!("{prefix}{k}");
                let c = if turned.contains(&name.as_str()) {
                    c.reverse()
                } else {
                    c
                };
                wire(&mut part, &name, c)
            })
            .collect()
    };
    let u = add("u", u);
    let v = add("v", v);
    (
        part,
        u_order.map(|k| u[k].clone()).to_vec(),
        v_order.map(|k| v[k].clone()).to_vec(),
    )
}

fn span(part: Part<S>, u: Vec<EntityRef>, v: Vec<EntityRef>) -> GeopResult<Part<S>> {
    let args = NetworkSurfaceArgs {
        u_curves: u,
        v_curves: v,
    };
    NetworkSurface.apply(part, "n", &args, &NoFiles)
}

fn surface_of(part: &Part<S>, face: &str) -> NurbSurface3D<S> {
    let face = part.face_id(face).unwrap();
    part.topology().get_face(face).unwrap().surface.clone()
}

/// Whether `p` is a point of `surface`: located by a containment search,
/// refined by Newton, and the point there `p`'s enclosure — or what of
/// that failed.
fn on_surface(surface: &NurbSurface3D<S>, p: &Vector3<S>) -> Result<(), String> {
    let Some((u, v)) =
        surface_could_contain(surface, p, MAX_NODES, min_subdivision_size()).unwrap()
    else {
        return Err("the containment search finds it nowhere".into());
    };
    let (u, v) = surface.project(*p, u.sharpen(), v.sharpen(), 20).unwrap();
    let q = surface.evaluate(u, v).unwrap();
    if q.could_be_equal(p) {
        Ok(())
    } else {
        Err(format!(
            "refined to ({u:?}, {v:?}), the surface is at {q:?}"
        ))
    }
}

/// A 3 × 3 network of curved lines — one at a speed of its own, the
/// others running on past the grid — runs along every curve, between the
/// outer ones, and is a valid sheet bounded by them.
#[test]
fn network_of_three_by_three_runs_along_every_curve() {
    let (part, u, v) = grid(1.0, &[], [0, 1, 2], [0, 1, 2]);
    let part = span(part, u, v).unwrap();
    assert_valid(&part);
    let surface = surface_of(&part, "network(n)");
    let (u, v) = grid_curves(1.0);
    for (k, curve) in u.iter().enumerate() {
        for i in 0..=8 {
            let p = curve.evaluate(S::from_ratio(i, 8).unwrap()).unwrap();
            on_surface(&surface, &p).unwrap_or_else(|e| panic!("u{k} at {i}/8: {p:?}: {e}"));
        }
    }
    for (k, curve) in v.iter().enumerate() {
        // Within the grid: y from 0 to 2.
        for i in 0..=8 {
            let s = S::from_f64((0.5 + 2.0 * i as f64 / 8.0) / 3.0);
            let p = curve.evaluate(s).unwrap();
            on_surface(&surface, &p).unwrap_or_else(|e| panic!("v{k} at {i}/8: {p:?}: {e}"));
        }
    }
    // Bounded by the outer curves, cut to the grid.
    for name in [
        "network(n,u0)",
        "network(n,u2)",
        "network(n,v0)",
        "network(n,v2)",
    ] {
        part.edge_id(name).unwrap();
    }
    let corner = part.vertex_id("network(n,u0,v0)").unwrap();
    let p = part.topology().get_vertex(corner).unwrap().point;
    assert!(p.could_be_equal(&Vector3::from_array([0.0, 0.0, 0.0].map(S::from_f64))));
}

/// Picked in another order, some of them running the other way: the same
/// surface.
#[test]
fn network_picks_in_any_order_and_direction() {
    let (part, u, v) = grid(1.0, &[], [0, 1, 2], [0, 1, 2]);
    let ordered = surface_of(&span(part, u, v).unwrap(), "network(n)");
    let (part, u, v) = grid(1.0, &["u1", "v0", "v2"], [2, 0, 1], [1, 2, 0]);
    let part = span(part, u, v).unwrap();
    assert_valid(&part);
    let shuffled = surface_of(&part, "network(n)");
    for (a, b) in [(&ordered, &shuffled), (&shuffled, &ordered)] {
        for i in 0..=4 {
            for j in 0..=4 {
                let (u, v) = (S::from_ratio(i, 4).unwrap(), S::from_ratio(j, 4).unwrap());
                let p = a.evaluate(u, v).unwrap();
                on_surface(b, &p).unwrap_or_else(|e| panic!("at ({i}/4, {j}/4): {p:?}: {e}"));
            }
        }
    }
}

/// Four curves meeting at their ends: the boundary surface's Coons patch
/// of the loop they make.
#[test]
fn network_of_two_by_two_is_the_coons_patch() {
    let mut part = Part::new();
    let bottom = NurbCurve::try_new(
        3,
        [
            [0., 0., 0.],
            [0.3, -0.2, 0.4],
            [0.6, 0.3, -0.2],
            [1., 0., 0.],
        ]
        .map(|[x, y, z]| Vector4::from_array([x, y, z, 1.].map(S::from_f64)))
        .to_vec(),
        [0., 0., 0., 0., 1., 1., 1., 1.].map(S::from_f64).to_vec(),
    )
    .unwrap();
    let u = vec![
        wire(&mut part, "bottom", bottom),
        wire(
            &mut part,
            "top",
            polynomial([&[1., -1.], &[1.], &[1., 0., 0.3]], 2),
        ),
    ];
    let v = vec![
        wire(
            &mut part,
            "left",
            polynomial([&[0.], &[0., 1.], &[0., 1.6, -0.3]], 2),
        ),
        wire(
            &mut part,
            "right",
            polynomial([&[1.], &[0., 1.], &[0., 1.]], 1),
        ),
    ];
    let edges: Vec<EntityRef> = u.iter().chain(&v).cloned().collect();
    let part = span(part, u, v).unwrap();
    assert_valid(&part);
    let args = BoundarySurfaceArgs {
        edges,
        tangent: Vec::new(),
    };
    let part = BoundarySurface.apply(part, "b", &args, &NoFiles).unwrap();
    let network = surface_of(&part, "network(n)");
    let coons = surface_of(&part, "boundary(b)");
    for i in 0..=6 {
        for j in 0..=6 {
            let (u, v) = (S::from_ratio(i, 6).unwrap(), S::from_ratio(j, 6).unwrap());
            let p = network.evaluate(u, v).unwrap();
            on_surface(&coons, &p).unwrap_or_else(|e| panic!("at ({i}/6, {j}/6): {p:?}: {e}"));
        }
    }
}

/// A `v` curve lifted off the grid misses the `u` curves: refused, naming
/// a pair and how far apart they pass.
#[test]
fn network_refuses_curves_that_miss() {
    let mut part = Part::new();
    let (u, mut v) = grid_curves(1.0);
    v[1] = v[1].translate(Vector3::from_array([0.0, 0.0, 0.01].map(S::from_f64)));
    let u: Vec<_> = u
        .into_iter()
        .enumerate()
        .map(|(k, c)| wire(&mut part, &format!("u{k}"), c))
        .collect();
    let v: Vec<_> = v
        .into_iter()
        .enumerate()
        .map(|(k, c)| wire(&mut part, &format!("v{k}"), c))
        .collect();
    let error = refused(span(part, u, v));
    let prefix = "u curve u0 and v curve v1 do not cross: they pass ";
    let at = error.find(prefix).unwrap_or_else(|| panic!("{error}")) + prefix.len();
    let apart: f64 = error[at..].split(' ').next().unwrap().parse().unwrap();
    // Lifted 0.01 off a grid sloping where they pass.
    assert!(0.005 < apart && apart <= 0.01, "{error}");
}

/// Too few curves, and a curve picked in both directions, are refused.
#[test]
fn network_refuses_too_few_curves_and_curves_picked_twice() {
    let (part, u, v) = grid(1.0, &[], [0, 1, 2], [0, 1, 2]);
    let error = refused(span(part.clone(), u[..1].to_vec(), v.clone()));
    assert!(error.contains("two curves at least"), "{error}");
    let mut twice = v.clone();
    twice.push(u[0].clone());
    let error = refused(span(part, u, twice));
    assert!(error.contains("is picked twice"), "{error}");
}

/// A flat network's sheet thickens into a valid solid. (Thickening
/// offsets its faces, which only planes and surfaces of revolution do
/// exactly yet.)
#[test]
fn flat_network_sheet_thickens() {
    let (part, u, v) = grid(0.0, &[], [0, 1, 2], [0, 1, 2]);
    let part = span(part, u, v).unwrap();
    let args = ThickenArgs {
        face: "network(n)".into(),
        thickness: 0.05,
        side: ThickenSide::Both,
    };
    let part = Thicken.apply(part, "t", &args, &NoFiles).unwrap();
    assert_valid(&part);
    part.solid_id("thicken(t)").unwrap();
}

/// A small generator of numbers in `[0, 1)`: the same every run.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// `curve`, run the other way for half the values of `coin`.
fn turned(curve: NurbCurve3D<S>, coin: f64) -> NurbCurve3D<S> {
    if coin < 0.5 { curve.reverse() } else { curve }
}

/// Random networks: `m` × `n` grids of heights, the curves through them
/// cubic splines along `x` and along `y`, each through the heights where
/// it crosses the others and more points between, and run either way.
/// The sheet is valid and runs through every crossing.
#[test]
#[ignore = "slow: random networks — run with `cargo test -- --ignored`"]
fn random_networks_are_valid() {
    let mut random = Lcg(7);
    for case in 0..60 {
        let (m, n) = (2 + case % 3, 2 + (case / 3) % 3);
        let xs: Vec<f64> = (0..n).map(|i| i as f64 + 0.3 * random.next()).collect();
        let ys: Vec<f64> = (0..m).map(|j| j as f64 + 0.3 * random.next()).collect();
        let z: Vec<Vec<f64>> = (0..m)
            .map(|_| (0..n).map(|_| random.next() - 0.5).collect())
            .collect();
        let point = |x: f64, y: f64, h: f64| Vector3::from_array([x, y, h].map(S::from_f64));
        let mut part = Part::new();
        let u: Vec<EntityRef> = (0..m)
            .map(|j| {
                let mut points = Vec::new();
                for i in 0..n {
                    points.push(point(xs[i], ys[j], z[j][i]));
                    if i + 1 < n {
                        let x = 0.5 * (xs[i] + xs[i + 1]);
                        points.push(point(x, ys[j], random.next() - 0.5));
                    }
                }
                wire(
                    &mut part,
                    &format!("u{j}"),
                    turned(NurbCurve3D::interpolate(&points, 3).unwrap(), random.next()),
                )
            })
            .collect();
        let v: Vec<EntityRef> = (0..n)
            .map(|i| {
                let mut points = Vec::new();
                for j in 0..m {
                    points.push(point(xs[i], ys[j], z[j][i]));
                    if j + 1 < m {
                        let y = 0.5 * (ys[j] + ys[j + 1]);
                        points.push(point(xs[i], y, random.next() - 0.5));
                    }
                }
                wire(
                    &mut part,
                    &format!("v{i}"),
                    turned(NurbCurve3D::interpolate(&points, 3).unwrap(), random.next()),
                )
            })
            .collect();
        let part = span(part, u, v).unwrap_or_else(|e| panic!("case {case}: {e}"));
        assert_valid(&part);
        let surface = surface_of(&part, "network(n)");
        for j in 0..m {
            for i in 0..n {
                let p = point(xs[i], ys[j], z[j][i]);
                on_surface(&surface, &p)
                    .unwrap_or_else(|e| panic!("case {case}: crossing ({j}, {i}): {e}"));
            }
        }
    }
}

/// Two `v` curves crossing each other between the `u` curves cross them in
/// different orders: no grid, refused naming them.
#[test]
fn network_refuses_curves_of_one_direction_that_meet() {
    let mut part = Part::new();
    let line = |a: [f64; 2], b: [f64; 2]| {
        polynomial([&[a[0], b[0] - a[0]], &[a[1], b[1] - a[1]], &[0.]], 1)
    };
    let u = vec![
        wire(&mut part, "u0", line([-1., 0.], [2., 0.])),
        wire(&mut part, "u1", line([-1., 1.], [2., 1.])),
    ];
    let v = vec![
        wire(&mut part, "v0", line([0., -0.5], [1., 1.5])),
        wire(&mut part, "v1", line([1., -0.5], [0., 1.5])),
    ];
    let error = refused(span(part, u, v));
    assert!(
        error.contains("v curve v0 and v curve v1 meet between the curves they cross"),
        "{error}"
    );
}
