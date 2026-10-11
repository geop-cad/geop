//! The generic system on numbers and on poses: solved, enclosed, measured
//! for freedom, and its conflicts named — in every scalar.

use super::*;
use crate::{dual::Dual, for_all_scalars, primitives::Pose, scalars::Scalar, vector::Vector3};

fn n<S: Scalar>(v: f64) -> S {
    S::from_f64(v)
}

/// The derivatives a term of dual numbers carries by the slots `slots` of
/// its gradient: per block, one row per residual.
fn linearized<S: Scalar, const N: usize>(
    residuals: &[Dual<S, N>],
    slots: &[(usize, std::ops::Range<usize>)],
) -> Linearized<S> {
    Linearized {
        values: residuals.iter().map(|r| r.v).collect(),
        jacobian: (slots.iter())
            .map(|(block, range)| {
                let rows = (residuals.iter())
                    .map(|r| range.clone().map(|k| r.d[k]).collect())
                    .collect();
                (*block, rows)
            })
            .collect(),
    }
}

/// Lines `a x + b y = c` over two numbers, each a constraint.
struct Lines<S: Scalar>(Vec<[S; 3]>);

impl<S: Scalar> Equations for Lines<S> {
    type Value = S;

    fn count(&self) -> (usize, usize) {
        (self.0.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[S], which: &[Term]) -> Vec<Option<Linearized<S>>> {
        (which.iter())
            .map(|term| {
                let Term::Constraint(c) = term else {
                    return None;
                };
                let [a, b, c] = self.0[*c].map(Dual::<S, 2>::cst);
                let (x, y) = (Dual::var(values[0], 0), Dual::var(values[1], 1));
                let r = a.mul(x).add(b.mul(y)).sub(c);
                Some(linearized(&[r], &[(0, 0..1), (1, 1..2)]))
            })
            .collect()
    }
}

/// `x + y = 5` and `x - y = 1`: `(3, 2)`, solved, held and enclosed.
fn lines_are_solved_and_enclosed<S: Scalar>() {
    let lines = Lines(vec![[n(1.0), n(1.0), n(5.0)], [n(1.0), n(-1.0), n(1.0)]]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: S::ONE,
    };
    let solved = system.solve(&[S::ZERO, S::ZERO]).unwrap();
    assert!(system.held(&solved.values).unwrap().converged);
    assert!(solved.enclosure.failures.is_empty());
    for (found, exact) in solved.enclosure.boxes.iter().zip([3.0, 2.0]) {
        let found = found.unwrap();
        assert!(
            n::<S>(exact).is_subset_of(found),
            "{found:?} misses {exact}"
        );
    }
    assert_eq!(solved.enclosure.total, 0);
}

#[test]
fn lines_are_solved_and_enclosed_in_every_scalar() {
    for_all_scalars!(lines_are_solved_and_enclosed);
}

/// A third line through neither: no solution to enclose, and the
/// conflict is named — the line that cannot hold and those it depends on.
fn conflicting_lines_are_named<S: Scalar>() {
    let lines = Lines(vec![
        [n(1.0), n(1.0), n(5.0)],
        [n(1.0), n(1.0), n(6.0)],
        [n(1.0), n(-1.0), n(1.0)],
    ]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: S::ONE,
    };
    let at = [n::<S>(3.0), n::<S>(2.0)];
    let enclosure = system.enclose(&at).unwrap();
    let (_, error) = &enclosure.failures[0];
    assert!(matches!(error, EncloseError::Conflict { .. }), "{error:?}");
    assert_eq!(system.conflicting(&at).unwrap(), [0, 1]);
}

#[test]
fn conflicting_lines_are_named_in_every_scalar() {
    for_all_scalars!(conflicting_lines_are_named);
}

/// A number or a pose: the values of an assembly's state.
#[derive(Clone, Debug, PartialEq)]
enum Value<S: Scalar> {
    Number(S),
    Pose(Pose<S>),
}

impl<S: Scalar + PartialEq> Variable for Value<S> {
    type Scalar = S;

    fn dimension(&self) -> usize {
        match self {
            Value::Number(v) => v.dimension(),
            Value::Pose(p) => p.dimension(),
        }
    }

    fn retract(&self, step: &[S]) -> GeopResult<Self> {
        Ok(match self {
            Value::Number(v) => Value::Number(v.retract(step)?),
            Value::Pose(p) => Value::Pose(p.retract(step)?),
        })
    }

    fn retract_over(&self, step: &[S]) -> GeopResult<Self> {
        Ok(match self {
            Value::Number(v) => Value::Number(v.retract_over(step)?),
            Value::Pose(p) => Value::Pose(p.retract_over(step)?),
        })
    }

    fn differential(&self, step: &[S]) -> GeopResult<Option<Vec<Vec<S>>>> {
        match self {
            Value::Number(v) => v.differential(step),
            Value::Pose(p) => p.differential(step),
        }
    }

    fn stay(&self, at: &Self, scale: S) -> GeopResult<Rows<S>> {
        match (self, at) {
            (Value::Number(v), Value::Number(at)) => v.stay(at, scale),
            (Value::Pose(p), Value::Pose(at)) => p.stay(at, scale),
            _ => Err(GeopError::new("values of different kinds")),
        }
    }

    fn same(&self, other: &Self) -> bool {
        self == other
    }
}

type D<S> = Dual<S, 13>;

/// A body's pose, moved by a step of its own variables from `slot` on —
/// as a solver differentiates it.
fn stepped<S: Scalar>(pose: &Pose<S>, slot: usize) -> Pose<D<S>> {
    let var = |k: usize| D::<S>::var(S::ZERO, slot + k);
    let step = Pose::exp(
        &Vector3::from_array([var(0), var(1), var(2)]),
        &Vector3::from_array([var(3), var(4), var(5)]),
    )
    .unwrap();
    pose.map(D::cst).compose(&step)
}

fn point<S: Scalar>(p: [f64; 3]) -> Vector3<D<S>> {
    Vector3::from_array(p.map(|c| D::cst(n(c))))
}

/// Three points of a body: where it is, all of it.
const MARKS: [[f64; 3]; 3] = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];

/// Constraints on a state of numbers and poses, each a function of the
/// values by dual numbers whose gradient has the blocks' variables one
/// after another — `layout` saying where each block's start.
struct Mechanism<S: Scalar> {
    layout: Vec<usize>,
    #[allow(clippy::type_complexity)]
    constraints: Vec<Box<dyn Fn(&[Value<S>], &dyn Fn(usize) -> Var<S>) -> Vec<D<S>>>>,
}

/// A block as a constraint reads it.
enum Var<S: Scalar> {
    Number(D<S>),
    Pose(Pose<D<S>>),
}

impl<S: Scalar> Var<S> {
    fn number(self) -> D<S> {
        match self {
            Var::Number(v) => v,
            Var::Pose(_) => panic!("not a number"),
        }
    }

    fn pose(self) -> Pose<D<S>> {
        match self {
            Var::Pose(p) => p,
            Var::Number(_) => panic!("not a pose"),
        }
    }
}

impl<S: Scalar + PartialEq> Equations for Mechanism<S> {
    type Value = Value<S>;

    fn count(&self) -> (usize, usize) {
        (self.constraints.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[Value<S>], which: &[Term]) -> Vec<Option<Linearized<S>>> {
        let read = |block: usize| match &values[block] {
            Value::Number(v) => Var::Number(D::var(*v, self.layout[block])),
            Value::Pose(p) => Var::Pose(stepped(p, self.layout[block])),
        };
        let slots: Vec<(usize, std::ops::Range<usize>)> = (values.iter().enumerate())
            .map(|(block, v)| {
                let start = self.layout[block];
                (block, start..start + v.dimension())
            })
            .collect();
        (which.iter())
            .map(|term| {
                let Term::Constraint(c) = term else {
                    return None;
                };
                let residuals = (self.constraints[*c])(values, &read);
                Some(linearized(&residuals, &slots))
            })
            .collect()
    }
}

/// The residuals of `pose` putting the body's marks where `target` does.
fn at<S: Scalar>(pose: &Pose<D<S>>, target: &Pose<D<S>>) -> Vec<D<S>> {
    MARKS
        .iter()
        .flat_map(|&m| {
            pose.apply(&point(m))
                .sub(&target.apply(&point(m)))
                .to_array()
        })
        .collect()
}

/// The size of the pose systems: a fixed point scalar resolves a turn only
/// to `2^-32`, so their constraints hold to a part in a billion of a size
/// some way above the bodies' own, which are about one.
fn size<S: Scalar>() -> S {
    S::from_i64(100)
}

/// A turn by `radians` about `z`, at `position`.
fn turned<S: Scalar>(radians: f64, position: [f64; 3]) -> Pose<S> {
    Pose::rotation_about(
        &Vector3::zero(),
        &Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
        n(radians),
    )
    .unwrap()
    .with_position(Vector3::from_array(position.map(n)))
}

/// A body pinned to a frame turned a quarter about `z` and moved: three
/// marks, nine residuals over six variables — three of them redundant,
/// which the rows chosen over a box leave out. The enclosure holds the
/// body where the frame is, to every digit it resolves, and nothing is
/// free.
fn a_pinned_body_is_enclosed<S: Scalar + PartialEq>() {
    let frame = turned::<S>(std::f64::consts::FRAC_PI_2, [1.0, 2.0, -0.5]);
    let mechanism = Mechanism {
        layout: vec![0],
        constraints: vec![Box::new(move |_, read| {
            at(&read(0).pose(), &frame.map(D::cst))
        })],
    };
    let system = System {
        equations: &mechanism,
        mobility: vec![Mobility::Held],
        scale: size(),
    };
    let start = [Value::Pose(turned::<S>(0.3, [0.5, 1.0, 0.0]))];
    let solved = system.solve(&start).unwrap();
    let held = system.held(&solved.values).unwrap();
    assert!(held.converged, "{held:?} after {:?}", solved.phases);
    assert!(
        solved.enclosure.failures.is_empty(),
        "{:?}",
        solved.enclosure.failures
    );
    let Some(Value::Pose(enclosed)) = &solved.enclosure.boxes[0] else {
        panic!("the body is not enclosed");
    };
    // The quarter turn takes `x` to `y` and `y` to `-x`, exactly.
    for (mark, exact) in MARKS
        .iter()
        .zip([[1.0, 2.0, -0.5], [1.0, 3.0, -0.5], [0.0, 2.0, -0.5]])
    {
        let moved = enclosed.apply(&Vector3::from_array(mark.map(n)));
        for k in 0..3 {
            assert!(
                n::<S>(exact[k]).is_subset_of(moved[k]),
                "{mark:?} goes to {moved:?}, not {exact:?}"
            );
            assert!(moved[k].width().to_f64() < 1e-6, "{moved:?}");
        }
    }
    assert_eq!(solved.enclosure.freedom, [0]);
}

#[test]
fn a_pinned_body_is_enclosed_in_every_scalar() {
    for_all_scalars!(a_pinned_body_is_enclosed);
}

/// A revolute chain: an arm turning about `z` at the origin by a joint's
/// angle, a second link fixed to its end. Its axis is said by six
/// residuals for its two freedoms, its end twice: the rows chosen over a
/// box find the one freedom the joint's angle leaves, and with the angle
/// driven, none; the second link is enclosed where the angle puts it.
fn a_revolute_chain_is_enclosed<S: Scalar + PartialEq>() {
    let z = |p: &Pose<D<S>>| {
        p.apply(&point([0.0, 0.0, 1.0]))
            .sub(&p.apply(&point([0.0; 3])))
    };
    let axis = move |values: &[Value<S>], read: &dyn Fn(usize) -> Var<S>| {
        let _ = values;
        let arm = read(1).pose();
        let mut r: Vec<D<S>> = arm.apply(&point([0.0; 3])).to_array().to_vec();
        r.extend(z(&arm).sub(&point([0.0, 0.0, 1.0])).to_array());
        r
    };
    let angle = move |_: &[Value<S>], read: &dyn Fn(usize) -> Var<S>| {
        let (angle, arm) = (read(0).number(), read(1).pose());
        let x = arm
            .apply(&point([1.0, 0.0, 0.0]))
            .sub(&arm.apply(&point([0.0; 3])));
        vec![x[0].sub(angle.cos()), x[1].sub(angle.sin())]
    };
    let end = turned::<D<S>>(0.0, [2.0, 0.0, 0.0]);
    let link = move |_: &[Value<S>], read: &dyn Fn(usize) -> Var<S>| {
        let (arm, link) = (read(1).pose(), read(2).pose());
        at(&link, &arm.compose(&end))
    };
    let drive =
        |_: &[Value<S>], read: &dyn Fn(usize) -> Var<S>| vec![read(0).number().sub(D::cst(n(0.5)))];
    let start = [
        Value::Number(n(0.3)),
        Value::Pose(turned::<S>(0.3, [0.0; 3])),
        Value::Pose(turned::<S>(0.2, [1.9, 0.6, 0.1])),
    ];
    let mobility = vec![Mobility::Follows, Mobility::Held, Mobility::Held];
    let layout = vec![0, 1, 7];

    // Undriven, where every constraint holds: one freedom, the turn about
    // the axis.
    let free = Mechanism {
        layout: layout.clone(),
        constraints: vec![Box::new(axis), Box::new(angle), Box::new(link)],
    };
    let system = System {
        equations: &free,
        mobility: mobility.clone(),
        scale: size(),
    };
    let arm = turned::<S>(0.3, [0.0; 3]);
    let holding = [
        Value::Number(n(0.3)),
        Value::Pose(arm),
        Value::Pose(arm.compose(&turned::<S>(0.0, [2.0, 0.0, 0.0]))),
    ];
    assert!(system.held(&holding).unwrap().converged);
    let freedom = system.freedom(&holding).unwrap();
    assert_eq!(freedom.total, 1, "{freedom:?}");
    assert_eq!(freedom.blocks, [1, 1, 1], "{freedom:?}");

    // Driven to half a radian: none, and the link where the angle puts it.
    let driven = Mechanism {
        layout,
        constraints: vec![
            Box::new(axis),
            Box::new(angle),
            Box::new(link),
            Box::new(drive),
        ],
    };
    let system = System {
        equations: &driven,
        mobility,
        scale: size(),
    };
    let solved = system.solve(&start).unwrap();
    let held = system.held(&solved.values).unwrap();
    assert!(held.converged, "{held:?} after {:?}", solved.phases);
    assert!(
        solved.enclosure.failures.is_empty(),
        "{:?}",
        solved.enclosure.failures
    );
    assert_eq!(solved.enclosure.total, 0);
    let Some(Value::Pose(link)) = &solved.enclosure.boxes[2] else {
        panic!("the link is not enclosed");
    };
    let origin = link.apply(&Vector3::zero());
    let half = n::<S>(0.5);
    let exact = [S::TWO.mul(half.cos()), S::TWO.mul(half.sin()), S::ZERO];
    for k in 0..3 {
        assert!(origin[k].could_be_equal(exact[k]), "{origin:?}");
        assert!(origin[k].width().to_f64() < 1e-6, "{origin:?}");
    }
}

#[test]
fn a_revolute_chain_is_enclosed_in_every_scalar() {
    for_all_scalars!(a_revolute_chain_is_enclosed);
}

/// A body pinned to two frames apart: nothing to enclose, and both pins
/// are named as what conflicts.
fn two_pins_conflict<S: Scalar + PartialEq>() {
    let pin = |to: [f64; 3]| {
        let frame = turned::<S>(0.0, to);
        Box::new(move |_: &[Value<S>], read: &dyn Fn(usize) -> Var<S>| {
            at(&read(0).pose(), &frame.map(D::cst))
        }) as Box<dyn Fn(&[Value<S>], &dyn Fn(usize) -> Var<S>) -> Vec<D<S>>>
    };
    let mechanism = Mechanism {
        layout: vec![0],
        constraints: vec![pin([1.0, 0.0, 0.0]), pin([0.0, 2.0, 0.0])],
    };
    let system = System {
        equations: &mechanism,
        mobility: vec![Mobility::Held],
        scale: size(),
    };
    let start = [Value::Pose(Pose::identity())];
    let solved = system.solve(&start).unwrap();
    assert!(!system.held(&solved.values).unwrap().converged);
    assert!(!solved.enclosure.failures.is_empty());
    assert_eq!(system.conflicting(&start).unwrap(), [0, 1]);
}

#[test]
fn two_pins_conflict_in_every_scalar() {
    for_all_scalars!(two_pins_conflict);
}

/// The right Jacobian of the exponential map, enclosed over a box of
/// twists around zero and away from it, holds the derivative a finite
/// difference measures, and is the identity at zero.
fn the_exponential_differential_is_enclosed<S: Scalar>() {
    let twist = [0.3, -0.2, 0.5, 1.0, 0.4, -0.7];
    let v3 = |a: &[f64]| Vector3::from_array([a[0], a[1], a[2]].map(n::<S>));
    let jacobian = Pose::<S>::exp_differential(&v3(&twist[..3]), &v3(&twist[3..])).unwrap();
    let pose = |t: &[f64; 6]| Pose::<ScalInF64>::exp(&v3f(&t[..3]), &v3f(&t[3..])).unwrap();
    let base = pose(&twist);
    let h = 1e-6;
    for k in 0..6 {
        let mut ahead = twist;
        ahead[k] += h;
        // The twist from `exp(ξ)` to `exp(ξ + h e_k)`, to first order.
        let step = base.inverse().compose(&pose(&ahead));
        let r = step.rotation();
        let measured = [
            2.0 * r.x.to_f64() / h,
            2.0 * r.y.to_f64() / h,
            2.0 * r.z.to_f64() / h,
            step.position()[0].to_f64() / h,
            step.position()[1].to_f64() / h,
            step.position()[2].to_f64() / h,
        ];
        for i in 0..6 {
            let found = jacobian[i][k].to_f64();
            assert!(
                (found - measured[i]).abs() < 1e-4,
                "J[{i}][{k}] {found} vs {measured:?}"
            );
        }
    }
    // Over a box around zero, the identity is inside.
    let tiny = n::<S>(1e-6);
    let around = tiny.neg().union(tiny);
    let boxed = Vector3::from_array([around; 3]);
    let jacobian = Pose::<S>::exp_differential(&boxed, &boxed).unwrap();
    for (i, row) in jacobian.iter().enumerate() {
        for (k, e) in row.iter().enumerate() {
            let identity = if i == k { S::ONE } else { S::ZERO };
            assert!(identity.is_subset_of(*e), "J[{i}][{k}] = {e:?}");
        }
    }
}

use crate::scalars::ScalInF64;

fn v3f(a: &[f64]) -> Vector3<ScalInF64> {
    Vector3::from_array([a[0], a[1], a[2]].map(ScalInF64::from_f64))
}

#[test]
fn the_exponential_differential_is_enclosed_in_every_scalar() {
    for_all_scalars!(the_exponential_differential_is_enclosed);
}

/// Constraints on a few numbers, each a function of all of them in dual
/// numbers: what the closed-form tests below solve.
type Formula<S> = Box<dyn Fn(&[Dual<S, 4>]) -> Vec<Dual<S, 4>>>;

struct Formulas<S: Scalar>(Vec<Formula<S>>);

impl<S: Scalar> Equations for Formulas<S> {
    type Value = S;

    fn count(&self) -> (usize, usize) {
        (self.0.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[S], which: &[Term]) -> Vec<Option<Linearized<S>>> {
        let vars: Vec<Dual<S, 4>> = (values.iter().enumerate())
            .map(|(k, &v)| Dual::var(v, k))
            .collect();
        let slots: Vec<(usize, std::ops::Range<usize>)> =
            (0..values.len()).map(|k| (k, k..k + 1)).collect();
        (which.iter())
            .map(|term| match term {
                Term::Constraint(c) => Some(linearized(&(self.0[*c])(&vars), &slots)),
                Term::Preference(_) => None,
            })
            .collect()
    }
}

/// The system of `formulas` over numbers, each held where it is.
fn numbers<S: Scalar>(formulas: &Formulas<S>, count: usize) -> System<'_, Formulas<S>> {
    System {
        equations: formulas,
        mobility: vec![Mobility::Held; count],
        scale: S::ONE,
    }
}

/// `found` holds `exact` — enclosed itself, so overlapping it — and is
/// tight: an enclosure, not merely a box somewhere near.
fn holds<S: Scalar>(found: Option<S>, exact: S) {
    let found = found.expect("enclosed");
    assert!(found.could_be_equal(exact), "{found:?} misses {exact:?}");
    assert!(found.width().to_f64() < 1e-6, "{found:?} is not tight");
}

/// The unit circle `x² + y² = r²` as a constraint.
fn circle<S: Scalar>(r: f64) -> Formula<S> {
    Box::new(move |v| vec![v[0].mul(v[0]).add(v[1].mul(v[1])).sub(Dual::cst(n(r * r)))])
}

/// The circle `x² + y² = 25` and the line `y = x + 1` cross at `(3, 4)` and
/// `(-4, -3)`: each enclosed from near it, exactly.
fn a_circle_and_a_line_cross_where_known<S: Scalar>() {
    let formulas = Formulas::<S>(vec![
        circle(5.0),
        Box::new(|v| vec![v[1].sub(v[0]).sub(Dual::cst(S::ONE))]),
    ]);
    let system = numbers(&formulas, 2);
    for (start, exact) in [([2.5, 3.6], [3.0, 4.0]), ([-3.5, -3.2], [-4.0, -3.0])] {
        let solved = system.solve(&start.map(n::<S>)).unwrap();
        let enclosure = &solved.enclosure;
        assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
        for (found, exact) in enclosure.boxes.iter().zip(exact) {
            assert!(
                n::<S>(exact).is_subset_of(found.unwrap()),
                "{found:?} misses {exact}"
            );
            holds(*found, n(exact));
        }
        assert_eq!(enclosure.total, 0);
        // The rows the solve ended at, a Newton step or two, a box, and its
        // check: a drag pays a few evaluations to enclose, not dozens.
        assert!(
            enclosure.evaluations <= 5,
            "enclosed in {} evaluations",
            enclosure.evaluations
        );
    }
}

#[test]
fn a_circle_and_a_line_cross_where_known_in_every_scalar() {
    for_all_scalars!(a_circle_and_a_line_cross_where_known);
}

/// A two-link arm, links 2 and 1.5 long, its end put where the angles 0.7
/// and 1.1 put it: those angles are enclosed — not the elbow's other way.
fn a_two_link_arm_reaches_with_known_angles<S: Scalar>() {
    let (a, b) = (n::<S>(0.7), n::<S>(1.1));
    let (l1, l2) = (n::<S>(2.0), n::<S>(1.5));
    let end = [
        l1.mul(a.cos()).add(l2.mul(a.add(b).cos())),
        l1.mul(a.sin()).add(l2.mul(a.add(b).sin())),
    ];
    let reach = move |v: &[Dual<S, 4>], k: usize| {
        let (l1, l2) = (Dual::cst(l1), Dual::cst(l2));
        let both = v[0].add(v[1]);
        let at = match k {
            0 => l1.mul(v[0].cos()).add(l2.mul(both.cos())),
            _ => l1.mul(v[0].sin()).add(l2.mul(both.sin())),
        };
        vec![at.sub(Dual::cst(end[k]))]
    };
    let formulas = Formulas::<S>(vec![
        Box::new(move |v| reach(v, 0)),
        Box::new(move |v| reach(v, 1)),
    ]);
    let system = numbers(&formulas, 2);
    let solved = system.solve(&[n(0.5), n(1.3)]).unwrap();
    let enclosure = &solved.enclosure;
    assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
    holds(enclosure.boxes[0], a);
    holds(enclosure.boxes[1], b);
}

#[test]
fn a_two_link_arm_reaches_with_known_angles_in_every_scalar() {
    for_all_scalars!(a_two_link_arm_reaches_with_known_angles);
}

/// One line `x + y = 5` over two numbers: one of them is free, and stays
/// exactly as the solve left it; the other is enclosed where the line puts
/// it.
fn an_underdetermined_system_keeps_its_free_directions<S: Scalar + PartialEq>() {
    let lines = Lines(vec![[n(1.0), n(1.0), n(5.0)]]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: S::ONE,
    };
    let solved = system.solve(&[n(1.0), n(1.0)]).unwrap();
    let enclosure = &solved.enclosure;
    assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
    assert_eq!(enclosure.total, 1);
    assert_eq!(enclosure.freedom, [1, 1]);
    let boxes: Vec<S> = enclosure.boxes.iter().map(|b| b.unwrap()).collect();
    let free: Vec<usize> = (0..2)
        .filter(|&k| boxes[k].is_sharp() && boxes[k] == solved.values[k])
        .collect();
    // Both, where the line is met exactly.
    let Some(&free) = free.first() else {
        panic!("no value is free: {boxes:?} for {:?}", solved.values);
    };
    holds(Some(boxes[1 - free]), n::<S>(5.0).sub(boxes[free]));
}

#[test]
fn an_underdetermined_system_keeps_its_free_directions_in_every_scalar() {
    for_all_scalars!(an_underdetermined_system_keeps_its_free_directions);
}

/// The unit circle and its tangent `y = 1`, a point on both: the one
/// solution, `(0, 1)`, is singular, and no solve ends exactly there. Not
/// proven, from wherever the solve ends — neither a box claimed, nor a
/// conflict for constraints that hold.
fn a_tangency_is_not_proven<S: Scalar>() {
    let formulas = Formulas::<S>(vec![
        circle(1.0),
        Box::new(|v| vec![v[1].sub(Dual::cst(S::ONE))]),
    ]);
    let system = numbers(&formulas, 2);
    for x in [0.3, 1e-4, 1e-6, 1e-8] {
        let enclosure = system.enclose(&[n(x), S::ONE]).unwrap();
        let [(_, error)] = &enclosure.failures[..] else {
            panic!("a tangency is proven at {x}: {enclosure:?}");
        };
        assert!(matches!(error, EncloseError::Unproven(_)), "{x}: {error:?}");
        assert!(enclosure.boxes.iter().all(Option::is_none));
    }
}

#[test]
fn a_tangency_is_not_proven_in_every_scalar() {
    for_all_scalars!(a_tangency_is_not_proven);
}

/// The unit circle and the line `y = 1.5`, which misses it, a point on
/// both: a conflict, named — the two constraints.
fn a_line_missing_a_circle_conflicts<S: Scalar>() {
    let formulas = Formulas::<S>(vec![
        circle(1.0),
        Box::new(|v| vec![v[1].sub(Dual::cst(n(1.5)))]),
    ]);
    let system = numbers(&formulas, 2);
    let start = [n::<S>(0.3), n::<S>(0.9)];
    let solved = system.solve(&start).unwrap();
    assert!(!system.held(&solved.values).unwrap().converged);
    let [(_, error)] = &solved.enclosure.failures[..] else {
        panic!("{:?}", solved.enclosure);
    };
    assert!(matches!(error, EncloseError::Conflict { .. }), "{error:?}");
    assert_eq!(system.conflicting(&start).unwrap(), [0, 1]);
}

#[test]
fn a_line_missing_a_circle_conflicts_in_every_scalar() {
    for_all_scalars!(a_line_missing_a_circle_conflicts);
}

/// A body pinned to a frame turned by `radians` about `axis`, from `start`:
/// the marks of the enclosed pose are where the frame's are.
fn pinned<S: Scalar + PartialEq>(start: Pose<S>, axis: [f64; 3], radians: f64, position: [f64; 3]) {
    let direction = Vector3::from_array(axis.map(n::<S>)).normalize().unwrap();
    let frame = Pose::rotation_about(&Vector3::zero(), &direction, n(radians))
        .unwrap()
        .with_position(Vector3::from_array(position.map(n)));
    let mechanism = Mechanism {
        layout: vec![0],
        constraints: vec![Box::new(move |_, read| {
            at(&read(0).pose(), &frame.map(D::cst))
        })],
    };
    let system = System {
        equations: &mechanism,
        mobility: vec![Mobility::Held],
        scale: size(),
    };
    let solved = system.solve(&[Value::Pose(start)]).unwrap();
    let held = system.held(&solved.values).unwrap();
    assert!(
        held.converged,
        "{radians}: {held:?} after {:?}",
        solved.phases
    );
    let enclosure = &solved.enclosure;
    assert!(
        enclosure.failures.is_empty(),
        "{radians}: {:?}",
        enclosure.failures
    );
    let Some(Value::Pose(enclosed)) = &enclosure.boxes[0] else {
        panic!("the body is not enclosed");
    };
    for mark in MARKS {
        let mark = Vector3::from_array(mark.map(n::<S>));
        let (found, exact) = (enclosed.apply(&mark), frame.apply(&mark));
        for k in 0..3 {
            assert!(
                found[k].could_be_equal(exact[k]),
                "{radians}: {found:?} misses {exact:?}"
            );
            assert!(found[k].width().to_f64() < 1e-6, "{radians}: {found:?}");
        }
    }
}

/// A turn of a half less a hair, exactly a half — from a start a little
/// turned, since from no turn at all every way round is alike — and one far
/// round a slanted axis: each reached from the start and enclosed where it
/// is.
fn poses_far_from_the_start_are_enclosed<S: Scalar + PartialEq>() {
    let pi = std::f64::consts::PI;
    let identity = Pose::identity();
    let off = Pose::from_euler(Vector3::zero(), [n(0.0), n(3.0), n(2.0)]).unwrap();
    pinned::<S>(identity, [0.0, 0.0, 1.0], pi - 1e-3, [1.0, -2.0, 0.5]);
    pinned::<S>(off, [1.0, 0.0, 0.0], pi, [0.0; 3]);
    pinned::<S>(identity, [1.0, 2.0, -0.5], 2.5, [-3.0, 0.25, 4.0]);
}

#[test]
fn poses_far_from_the_start_are_enclosed_in_every_scalar() {
    for_all_scalars!(poses_far_from_the_start_are_enclosed);
}
