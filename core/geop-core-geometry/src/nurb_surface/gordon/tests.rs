use geop_core_math::{
    for_all_scalars,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |c, i| c * (n - i) as f64 / (i + 1) as f64)
}

/// The Bézier curve on `[0, 1]` whose coordinates are the polynomials with
/// the monomial coefficients `coords` (lowest power first), all of degree
/// `degree`.
fn polynomial<S: Scalar>(coords: [&[f64]; 3], degree: usize) -> NurbCurve3D<S> {
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

/// Curves on the surface `z = 0.3 x² - 0.2 y² + 0.1 x y`: along `x` at
/// `y = 0, 1, 2`, the middle one run at a speed of its own (`x = 2 t²`),
/// and along `y` at `x = 0, 0.5, 2`. Their crossings, as
/// [`NurbSurface3D::gordon`] takes them.
#[allow(clippy::type_complexity)]
fn network<S: Scalar>() -> (
    Vec<NurbCurve3D<S>>,
    Vec<NurbCurve3D<S>>,
    Vec<Vec<S>>,
    Vec<Vec<S>>,
) {
    let f = S::from_f64;
    let u = [0.0, 1.0, 2.0]
        .iter()
        .enumerate()
        .map(|(j, &y)| {
            if j == 1 {
                polynomial(
                    [&[0., 0., 2.], &[y], &[-0.2 * y * y, 0., 0.2 * y, 0., 1.2]],
                    4,
                )
            } else {
                polynomial([&[0., 2.], &[y], &[-0.2 * y * y, 0.2 * y, 1.2]], 2)
            }
        })
        .collect();
    let v = [0.0, 0.5, 2.0]
        .iter()
        .map(|&x| polynomial([&[x], &[0., 2.], &[0.3 * x * x, 0.2 * x, -0.8]], 2))
        .collect();
    let u_crossings = (0..3)
        .map(|j| {
            if j == 1 {
                vec![f(0.), f(0.5), f(1.)]
            } else {
                vec![f(0.), f(0.25), f(1.)]
            }
        })
        .collect();
    let v_crossings = vec![vec![f(0.), f(0.5), f(1.)]; 3];
    (u, v, u_crossings, v_crossings)
}

/// `t` taken piecewise linearly from `from` onto `to`.
fn mapped<S: Scalar>(t: S, from: &[S], to: &[S]) -> S {
    let i = (0..from.len() - 2)
        .find(|&i| t.to_f64() <= from[i + 1].to_f64())
        .unwrap_or(from.len() - 2);
    let scale = to[i + 1].sub(to[i]).div(from[i + 1].sub(from[i])).unwrap();
    to[i].add(t.sub(from[i]).mul(scale))
}

/// The surface through a 3 × 3 network, one curve of which runs at a
/// different speed, runs along every curve.
fn check_gordon_runs_along_every_curve<S: Scalar>() {
    let (u, v, u_crossings, v_crossings) = network::<S>();
    let surface = NurbSurface3D::gordon(&u, &v, &u_crossings, &v_crossings).unwrap();
    // The common parameters, as `common_parameters` chooses them.
    let middle = S::from_f64((0.25 + 0.5 + 0.25) / 3.0).sharpen();
    let u_params = [S::ZERO, middle, S::ONE];
    let v_params = [S::ZERO, S::from_f64(0.5), S::ONE];
    for k in 0..=16 {
        let t = S::from_ratio(k, 16).unwrap();
        for (j, curve) in u.iter().enumerate() {
            let on = surface
                .evaluate(mapped(t, &u_crossings[j], &u_params), v_params[j])
                .unwrap();
            let p = curve.evaluate(t).unwrap();
            assert!(
                on.could_be_equal(&p),
                "u curve {j} at {t:?}: {on:?} vs {p:?}"
            );
        }
        for (i, curve) in v.iter().enumerate() {
            let on = surface.evaluate(u_params[i], t).unwrap();
            let p = curve.evaluate(t).unwrap();
            assert!(
                on.could_be_equal(&p),
                "v curve {i} at {t:?}: {on:?} vs {p:?}"
            );
        }
    }
}
#[test]
fn gordon_runs_along_every_curve() {
    for_all_scalars!(check_gordon_runs_along_every_curve);
}

fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve3D<S> {
    polynomial(
        [
            &[a[0], b[0] - a[0]],
            &[a[1], b[1] - a[1]],
            &[a[2], b[2] - a[2]],
        ],
        1,
    )
}

/// The quarter circle in the plane `x = 0` from the origin round the
/// center `(0, 0, 1)` to `(0, 1, 1)`.
fn rising_arc<S: Scalar>() -> NurbCurve3D<S> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let f = S::from_f64;
    NurbCurve::try_new(
        2,
        vec![
            Vector4::from_array([0., 0., 0., 1.].map(f)),
            Vector4::from_array([0., w, 0., w].map(f)),
            Vector4::from_array([0., 1., 1., 1.].map(f)),
        ],
        [0., 0., 0., 1., 1., 1.].map(f).to_vec(),
    )
    .unwrap()
}

/// Two curves each way, crossing at their ends — a cubic, a line and a
/// rational arc among them: the Coons patch of the loop they make.
fn check_gordon_of_two_by_two_is_the_coons_patch<S: Scalar>() {
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
    let top = line::<S>([0., 1., 1.], [1., 1., 1.]);
    let left = rising_arc::<S>();
    let right = line::<S>([1., 0., 0.], [1., 1., 1.]);
    let ends = || vec![vec![S::ZERO, S::ONE]; 2];
    let gordon = NurbSurface3D::gordon(
        &[bottom.clone(), top.clone()],
        &[left.clone(), right.clone()],
        &ends(),
        &ends(),
    )
    .unwrap();
    let coons = NurbSurface3D::coons([&bottom, &right, &top.reverse(), &left.reverse()]).unwrap();
    for i in 0..=6 {
        for j in 0..=6 {
            let (u, v) = (S::from_ratio(i, 6).unwrap(), S::from_ratio(j, 6).unwrap());
            let (a, b) = (
                gordon.evaluate(u, v).unwrap(),
                coons.evaluate(u, v).unwrap(),
            );
            assert!(a.could_be_equal(&b), "at ({u:?}, {v:?}): {a:?} vs {b:?}");
        }
    }
}
#[test]
fn gordon_of_two_by_two_is_the_coons_patch() {
    for_all_scalars!(check_gordon_of_two_by_two_is_the_coons_patch);
}

/// A rational arc crossed in its middle: the surface still runs along it,
/// every point of it on its circle.
fn check_gordon_runs_along_an_arc_crossed_inside<S: Scalar>() {
    // Arcs in the planes `y = 0, 1, 2`, round `(1, y, 0)` from `(0, y, 0)`
    // over `(1, y, 1)` to `(2, y, 0)`, crossed by lines along `y` at
    // `x = 0`, half way up the first quarter, and at `x = 2`.
    let f = S::from_f64;
    let arc = |y: f64| {
        NurbCurve::try_new(
            2,
            vec![
                Vector4::from_array([0., y, 0., 1.].map(f)),
                Vector4::from_array([0., y, 1., 1.].map(f)).prod_scalar(f(0.5).sqrt().unwrap()),
                Vector4::from_array([1., y, 1., 1.].map(f)),
                Vector4::from_array([2., y, 1., 1.].map(f)).prod_scalar(f(0.5).sqrt().unwrap()),
                Vector4::from_array([2., y, 0., 1.].map(f)),
            ],
            [0., 0., 0., 0.5, 0.5, 1., 1., 1.].map(f).to_vec(),
        )
        .unwrap()
    };
    let u: Vec<_> = [0.0, 1.0, 2.0].iter().map(|&y| arc(y)).collect();
    // The middle line through the arcs' points at a quarter of their
    // domain, as the first arc has it: a line no sharp coordinates put
    // exactly through them.
    let quarter = u[0].evaluate(f(0.25)).unwrap();
    let across = |x: S, z: S| {
        let at = |y: f64| Vector4::from_array([x, f(y), z, S::ONE]);
        NurbCurve::try_new(1, vec![at(0.), at(2.)], [0., 0., 1., 1.].map(f).to_vec()).unwrap()
    };
    let v = vec![
        across(f(0.), f(0.)),
        across(quarter[0], quarter[2]),
        across(f(2.), f(0.)),
    ];
    let u_crossings = vec![vec![f(0.), f(0.25), f(1.)]; 3];
    let v_crossings = vec![vec![f(0.), f(0.5), f(1.)]; 3];
    let surface = NurbSurface3D::gordon(&u, &v, &u_crossings, &v_crossings).unwrap();
    for (j, y) in [0.0, 1.0, 2.0].into_iter().enumerate() {
        let v_param = S::from_ratio(j as i64, 2).unwrap();
        for k in 0..=16 {
            let p = surface
                .evaluate(S::from_ratio(k, 16).unwrap(), v_param)
                .unwrap();
            let from_center = p.sub(&Vector3::from_array([1., y, 0.].map(f)));
            assert!(
                from_center.norm_sq().could_be_equal(S::ONE),
                "arc {j} at {k}/16: {p:?}"
            );
        }
    }
}
#[test]
fn gordon_runs_along_an_arc_crossed_inside() {
    for_all_scalars!(check_gordon_runs_along_an_arc_crossed_inside);
}

/// Curves said to cross where they do not are refused.
fn check_gordon_refuses_curves_that_do_not_cross<S: Scalar>() {
    let (u, v, mut u_crossings, v_crossings) = network::<S>();
    u_crossings[0][1] = S::from_f64(0.3);
    let error = NurbSurface3D::gordon(&u, &v, &u_crossings, &v_crossings).unwrap_err();
    assert!(
        error.to_string().contains("where it crosses v curve 1"),
        "{error}"
    );
}
#[test]
fn gordon_refuses_curves_that_do_not_cross() {
    for_all_scalars!(check_gordon_refuses_curves_that_do_not_cross);
}
