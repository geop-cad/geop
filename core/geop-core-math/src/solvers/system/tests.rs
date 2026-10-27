//! The generic system on numbers and on poses: solved, enclosed, measured
//! for freedom, and its conflicts named.

use super::*;
use crate::{
    dual::Dual,
    primitives::Pose,
    scalars::{Number, Scalar},
    vector::Vector3,
};

fn n(v: f64) -> Scalar {
    Scalar::from_f64(v)
}

/// The derivatives a term of dual numbers carries by the slots `slots` of
/// its gradient: per block, one row per residual.
fn linearized<const N: usize>(
    residuals: &[Dual<N>],
    slots: &[(usize, std::ops::Range<usize>)],
) -> Linearized {
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
struct Lines(Vec<[Scalar; 3]>);

impl Equations for Lines {
    type Value = Scalar;

    fn count(&self) -> (usize, usize) {
        (self.0.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[Scalar], which: &[Term]) -> Vec<Option<Linearized>> {
        (which.iter())
            .map(|term| {
                let Term::Constraint(c) = term else {
                    return None;
                };
                let [a, b, c] = self.0[*c].map(Dual::<2>::cst);
                let (x, y) = (Dual::var(values[0], 0), Dual::var(values[1], 1));
                let r = a.mul(x).add(b.mul(y)).sub(c);
                Some(linearized(&[r], &[(0, 0..1), (1, 1..2)]))
            })
            .collect()
    }
}

/// `x + y = 5` and `x - y = 1`: `(3, 2)`, solved, held and enclosed.
#[test]
fn lines_are_solved_and_enclosed_in_every_scalar() {
    let lines = Lines(vec![[n(1.0), n(1.0), n(5.0)], [n(1.0), n(-1.0), n(1.0)]]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: Scalar::ONE,
    };
    let solved = system.solve(&[Scalar::ZERO, Scalar::ZERO]).unwrap();
    assert!(system.held(&solved.values).unwrap().converged);
    assert!(solved.enclosure.failures.is_empty());
    for (found, exact) in solved.enclosure.boxes.iter().zip([3.0, 2.0]) {
        let found = found.unwrap();
        assert!(n(exact).is_subset_of(found), "{found:?} misses {exact}");
    }
    assert_eq!(solved.enclosure.total, 0);
}

/// A third line through neither: no solution to enclose, and the
/// conflict is named — the line that cannot hold and those it depends on.
#[test]
fn conflicting_lines_are_named_in_every_scalar() {
    let lines = Lines(vec![
        [n(1.0), n(1.0), n(5.0)],
        [n(1.0), n(1.0), n(6.0)],
        [n(1.0), n(-1.0), n(1.0)],
    ]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: Scalar::ONE,
    };
    let at = [n(3.0), n(2.0)];
    let enclosure = system.enclose(&at).unwrap();
    let (_, error) = &enclosure.failures[0];
    assert!(matches!(error, EncloseError::Conflict { .. }), "{error:?}");
    assert_eq!(system.conflicting(&at).unwrap(), [0, 1]);
}

/// A number or a pose: the values of an assembly's state.
#[derive(Clone, Debug, PartialEq)]
enum Value {
    Number(Scalar),
    Pose(Pose),
}

impl Variable for Value {
    fn dimension(&self) -> usize {
        match self {
            Value::Number(v) => v.dimension(),
            Value::Pose(p) => p.dimension(),
        }
    }

    fn retract(&self, step: &[Scalar]) -> GeopResult<Self> {
        Ok(match self {
            Value::Number(v) => Value::Number(v.retract(step)?),
            Value::Pose(p) => Value::Pose(p.retract(step)?),
        })
    }

    fn retract_over(&self, step: &[Scalar]) -> GeopResult<Self> {
        Ok(match self {
            Value::Number(v) => Value::Number(v.retract_over(step)?),
            Value::Pose(p) => Value::Pose(p.retract_over(step)?),
        })
    }

    fn differential(&self, step: &[Scalar]) -> GeopResult<Option<Vec<Vec<Scalar>>>> {
        match self {
            Value::Number(v) => v.differential(step),
            Value::Pose(p) => p.differential(step),
        }
    }

    fn stay(&self, at: &Self, scale: Scalar) -> GeopResult<Rows> {
        match (self, at) {
            (Value::Number(v), Value::Number(at)) => v.stay(at, scale),
            (Value::Pose(p), Value::Pose(at)) => p.stay(at, scale),
            _ => Err(GeopError::new("values of different kinds")),
        }
    }

    fn same(&self, other: &Self) -> bool {
        self == other
    }

    fn carried(&self, from: &Self, to: &Self) -> Option<Self> {
        match (self, from, to) {
            (Value::Pose(p), Value::Pose(from), Value::Pose(to)) => {
                p.carried(from, to).map(Value::Pose)
            }
            _ => None,
        }
    }
}

type D = Dual<19>;

/// A body's pose, moved by a step of its own variables from `slot` on —
/// as a solver differentiates it.
fn stepped(pose: &Pose, slot: usize) -> Pose<D> {
    let var = |k: usize| D::var(Scalar::ZERO, slot + k);
    let step = Pose::exp(
        &Vector3::from_array([var(0), var(1), var(2)]),
        &Vector3::from_array([var(3), var(4), var(5)]),
    )
    .unwrap();
    pose.map(D::cst).compose(&step)
}

fn point(p: [f64; 3]) -> Vector3<D> {
    Vector3::from_array(p.map(|c| D::cst(n(c))))
}

/// Three points of a body: where it is, all of it.
const MARKS: [[f64; 3]; 3] = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];

/// Constraints on a state of numbers and poses, each a function of the
/// values by dual numbers whose gradient has the blocks' variables one
/// after another — `layout` saying where each block's start.
struct Mechanism {
    layout: Vec<usize>,
    #[allow(clippy::type_complexity)]
    constraints: Vec<Box<dyn Fn(&[Value], &dyn Fn(usize) -> Var) -> Vec<D>>>,
}

/// A block as a constraint reads it.
#[allow(clippy::large_enum_variant, reason = "a test's values, read once each")]
enum Var {
    Number(D),
    Pose(Pose<D>),
}

impl Var {
    fn number(self) -> D {
        match self {
            Var::Number(v) => v,
            Var::Pose(_) => panic!("not a number"),
        }
    }

    fn pose(self) -> Pose<D> {
        match self {
            Var::Pose(p) => p,
            Var::Number(_) => panic!("not a pose"),
        }
    }
}

impl Equations for Mechanism {
    type Value = Value;

    fn count(&self) -> (usize, usize) {
        (self.constraints.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[Value], which: &[Term]) -> Vec<Option<Linearized>> {
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
                // By the blocks it reads: those it has a derivative by — a
                // derivative by one it does not read is zero to the
                // smallest number the scalars tell apart from it, which a
                // product by zero rounds to.
                let read: Vec<(usize, std::ops::Range<usize>)> = (slots.iter())
                    .filter(|(_, range)| {
                        (residuals.iter()).any(|r| {
                            range
                                .clone()
                                .any(|k| r.d[k].abs().upper().to_f64() > 1e-200)
                        })
                    })
                    .cloned()
                    .collect();
                Some(linearized(&residuals, &read))
            })
            .collect()
    }
}

/// The residuals of `pose` putting the body's marks where `target` does.
fn at(pose: &Pose<D>, target: &Pose<D>) -> Vec<D> {
    MARKS
        .iter()
        .flat_map(|&m| {
            pose.apply(&point(m))
                .sub(&target.apply(&point(m)))
                .to_array()
        })
        .collect()
}

/// The size of the pose systems: some way above the bodies' own, which
/// are about one, so a turn's residual is a length the solver can
/// resolve.
fn size() -> Scalar {
    Scalar::from_i64(100)
}

/// A turn by `radians` about `z`, at `position`.
fn turned(radians: f64, position: [f64; 3]) -> Pose {
    Pose::rotation_about(
        &Vector3::zero(),
        &Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]),
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
#[test]
fn a_pinned_body_is_enclosed_in_every_scalar() {
    let frame = turned(std::f64::consts::FRAC_PI_2, [1.0, 2.0, -0.5]);
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
    let start = [Value::Pose(turned(0.3, [0.5, 1.0, 0.0]))];
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
                n(exact[k]).is_subset_of(moved[k]),
                "{mark:?} goes to {moved:?}, not {exact:?}"
            );
            assert!(moved[k].width().to_f64() < 1e-6, "{moved:?}");
        }
    }
    assert_eq!(solved.enclosure.freedom, [0]);
}

/// A revolute chain: an arm turning about `z` at the origin by a joint's
/// angle, a second link fixed to its end. Its axis is said by six
/// residuals for its two freedoms, its end twice: the rows chosen over a
/// box find the one freedom the joint's angle leaves, and with the angle
/// driven, none; the second link is enclosed where the angle puts it.
#[test]
fn a_revolute_chain_is_enclosed_in_every_scalar() {
    let z = |p: &Pose<D>| {
        p.apply(&point([0.0, 0.0, 1.0]))
            .sub(&p.apply(&point([0.0; 3])))
    };
    let axis = move |values: &[Value], read: &dyn Fn(usize) -> Var| {
        let _ = values;
        let arm = read(1).pose();
        let mut r: Vec<D> = arm.apply(&point([0.0; 3])).to_array().to_vec();
        r.extend(z(&arm).sub(&point([0.0, 0.0, 1.0])).to_array());
        r
    };
    let angle = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let (angle, arm) = (read(0).number(), read(1).pose());
        let x = arm
            .apply(&point([1.0, 0.0, 0.0]))
            .sub(&arm.apply(&point([0.0; 3])));
        vec![x[0].sub(angle.cos()), x[1].sub(angle.sin())]
    };
    let end = turned(0.0, [2.0, 0.0, 0.0]).map(D::cst);
    let link = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let (arm, link) = (read(1).pose(), read(2).pose());
        at(&link, &arm.compose(&end))
    };
    let drive =
        |_: &[Value], read: &dyn Fn(usize) -> Var| vec![read(0).number().sub(D::cst(n(0.5)))];
    let start = [
        Value::Number(n(0.3)),
        Value::Pose(turned(0.3, [0.0; 3])),
        Value::Pose(turned(0.2, [1.9, 0.6, 0.1])),
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
    let arm = turned(0.3, [0.0; 3]);
    let holding = [
        Value::Number(n(0.3)),
        Value::Pose(arm),
        Value::Pose(arm.compose(&turned(0.0, [2.0, 0.0, 0.0]))),
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
    let half = n(0.5);
    let exact = [
        Scalar::TWO.mul(half.cos()),
        Scalar::TWO.mul(half.sin()),
        Scalar::ZERO,
    ];
    for k in 0..3 {
        assert!(origin[k].could_be_equal(exact[k]), "{origin:?}");
        assert!(origin[k].width().to_f64() < 1e-6, "{origin:?}");
    }
}

/// An arm turning about `z` by a joint's angle, a link fastened to its end,
/// and a pin on the link, standing on its end along its `z` and free to
/// spin about it: the link hangs off the arm, the pin off the link — each
/// a pendant, solved and enclosed after what it hangs off, alone. Driven
/// to half a radian, the link and the pin go where the arm takes them, and
/// are enclosed there; undriven, the arm turns, the link with it, and the
/// pin with it and about its own axis: two freedoms, the pin's two.
#[test]
fn pendants_follow_what_they_hang_off_in_every_scalar() {
    let z = |p: &Pose<D>| {
        p.apply(&point([0.0, 0.0, 1.0]))
            .sub(&p.apply(&point([0.0; 3])))
    };
    let axis = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let arm = read(1).pose();
        let mut r: Vec<D> = arm.apply(&point([0.0; 3])).to_array().to_vec();
        r.extend(z(&arm).sub(&point([0.0, 0.0, 1.0])).to_array());
        r
    };
    let angle = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let (angle, arm) = (read(0).number(), read(1).pose());
        let x = arm
            .apply(&point([1.0, 0.0, 0.0]))
            .sub(&arm.apply(&point([0.0; 3])));
        vec![x[0].sub(angle.cos()), x[1].sub(angle.sin())]
    };
    let end = turned(0.0, [2.0, 0.0, 0.0]);
    let link = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let (arm, link) = (read(1).pose(), read(2).pose());
        at(&link, &arm.compose(&end.map(D::cst)))
    };
    // The pin's origin at the link's end, its `z` along the link's: five
    // rows of six variables.
    let tip = point([1.0, 0.0, 0.0]);
    let pin = move |_: &[Value], read: &dyn Fn(usize) -> Var| {
        let (link, pin) = (read(2).pose(), read(3).pose());
        let mut r: Vec<D> = (pin.apply(&point([0.0; 3])).sub(&link.apply(&tip)))
            .to_array()
            .to_vec();
        let off = z(&pin).sub(&z(&link));
        r.extend([off[0], off[1]]);
        r
    };
    let drive =
        |_: &[Value], read: &dyn Fn(usize) -> Var| vec![read(0).number().sub(D::cst(n(0.5)))];
    let mobility = vec![
        Mobility::Follows,
        Mobility::Held,
        Mobility::Held,
        Mobility::Held,
    ];
    let layout = vec![0, 1, 7, 13];
    let holding = |a: f64| {
        let arm = turned(a, [0.0; 3]);
        let link = arm.compose(&end);
        let pin = link.compose(&turned(0.7, [1.0, 0.0, 0.0]));
        vec![
            Value::Number(n(a)),
            Value::Pose(arm),
            Value::Pose(link),
            Value::Pose(pin),
        ]
    };

    // Undriven: the turn of the arm, and the spin of the pin.
    let free = Mechanism {
        layout: layout.clone(),
        constraints: vec![
            Box::new(axis),
            Box::new(angle),
            Box::new(link),
            Box::new(pin),
        ],
    };
    let system = System {
        equations: &free,
        mobility: mobility.clone(),
        scale: size(),
    };
    let start = holding(0.3);
    assert!(system.held(&start).unwrap().converged);
    let structure = system.structure(&start);
    let mut groups: Vec<(Vec<usize>, Vec<usize>, usize)> = (structure.groups.iter())
        .map(|g| (g.blocks.clone(), g.on.clone(), g.layer))
        .collect();
    groups.sort();
    // Undriven, the joint's angle only follows the arm too.
    assert_eq!(
        groups,
        [
            (vec![0], vec![1], 1),
            (vec![1], vec![], 0),
            (vec![2], vec![1], 1),
            (vec![3], vec![2], 2)
        ]
    );
    let freedom = system.freedom(&start).unwrap();
    assert_eq!(freedom.total, 2, "{freedom:?}");
    assert_eq!(freedom.blocks, [1, 1, 1, 2], "{freedom:?}");

    // Driven to half a radian: the pin where the arm takes it.
    let driven = Mechanism {
        layout,
        constraints: vec![
            Box::new(axis),
            Box::new(angle),
            Box::new(link),
            Box::new(pin),
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
    let enclosure = &solved.enclosure;
    assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
    assert_eq!(enclosure.total, 1, "the pin's spin: {enclosure:?}");
    let Some(Value::Pose(pin)) = &enclosure.boxes[3] else {
        panic!("the pin is not enclosed");
    };
    let origin = pin.apply(&Vector3::zero());
    let half = n(0.5);
    let exact = [n(3.0).mul(half.cos()), n(3.0).mul(half.sin()), Scalar::ZERO];
    for k in 0..3 {
        assert!(origin[k].could_be_equal(exact[k]), "{origin:?}");
        assert!(origin[k].width().to_f64() < 1e-6, "{origin:?}");
    }
}

/// A body pinned to two frames apart: nothing to enclose, and both pins
/// are named as what conflicts.
#[test]
fn two_pins_conflict_in_every_scalar() {
    let pin = |to: [f64; 3]| {
        let frame = turned(0.0, to);
        Box::new(move |_: &[Value], read: &dyn Fn(usize) -> Var| {
            at(&read(0).pose(), &frame.map(D::cst))
        }) as Box<dyn Fn(&[Value], &dyn Fn(usize) -> Var) -> Vec<D>>
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

/// The right Jacobian of the exponential map, enclosed over a box of
/// twists around zero and away from it, holds the derivative a finite
/// difference measures, and is the identity at zero.
#[test]
fn the_exponential_differential_is_enclosed_in_every_scalar() {
    let twist = [0.3, -0.2, 0.5, 1.0, 0.4, -0.7];
    let v3 = |a: &[f64]| Vector3::from_array([a[0], a[1], a[2]].map(n));
    let jacobian = Pose::exp_differential(&v3(&twist[..3]), &v3(&twist[3..])).unwrap();
    let pose = |t: &[f64; 6]| Pose::exp(&v3f(&t[..3]), &v3f(&t[3..])).unwrap();
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
    let tiny = n(1e-6);
    let around = tiny.neg().union(tiny);
    let boxed = Vector3::from_array([around; 3]);
    let jacobian = Pose::exp_differential(&boxed, &boxed).unwrap();
    for (i, row) in jacobian.iter().enumerate() {
        for (k, e) in row.iter().enumerate() {
            let identity = if i == k { Scalar::ONE } else { Scalar::ZERO };
            assert!(identity.is_subset_of(*e), "J[{i}][{k}] = {e:?}");
        }
    }
}

fn v3f(a: &[f64]) -> Vector3 {
    Vector3::from_array([a[0], a[1], a[2]].map(Scalar::from_f64))
}

/// Constraints on a few numbers, each a function of all of them in dual
/// numbers: what the closed-form tests below solve.
type Formula = Box<dyn Fn(&[Dual<4>]) -> Vec<Dual<4>>>;

struct Formulas(Vec<Formula>);

impl Equations for Formulas {
    type Value = Scalar;

    fn count(&self) -> (usize, usize) {
        (self.0.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[Scalar], which: &[Term]) -> Vec<Option<Linearized>> {
        let vars: Vec<Dual<4>> = (values.iter().enumerate())
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
fn numbers(formulas: &Formulas, count: usize) -> System<'_, Formulas> {
    System {
        equations: formulas,
        mobility: vec![Mobility::Held; count],
        scale: Scalar::ONE,
    }
}

/// `found` holds `exact` — enclosed itself, so overlapping it — and is
/// tight: an enclosure, not merely a box somewhere near.
fn holds(found: Option<Scalar>, exact: Scalar) {
    let found = found.expect("enclosed");
    assert!(found.could_be_equal(exact), "{found:?} misses {exact:?}");
    assert!(found.width().to_f64() < 1e-6, "{found:?} is not tight");
}

/// The unit circle `x² + y² = r²` as a constraint.
fn circle(r: f64) -> Formula {
    Box::new(move |v| vec![v[0].mul(v[0]).add(v[1].mul(v[1])).sub(Dual::cst(n(r * r)))])
}

/// The circle `x² + y² = 25` and the line `y = x + 1` cross at `(3, 4)` and
/// `(-4, -3)`: each enclosed from near it, exactly.
#[test]
fn a_circle_and_a_line_cross_where_known_in_every_scalar() {
    let formulas = Formulas(vec![
        circle(5.0),
        Box::new(|v| vec![v[1].sub(v[0]).sub(Dual::cst(Scalar::ONE))]),
    ]);
    let system = numbers(&formulas, 2);
    for (start, exact) in [([2.5, 3.6], [3.0, 4.0]), ([-3.5, -3.2], [-4.0, -3.0])] {
        let solved = system.solve(&start.map(n)).unwrap();
        let enclosure = &solved.enclosure;
        assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
        for (found, exact) in enclosure.boxes.iter().zip(exact) {
            assert!(
                n(exact).is_subset_of(found.unwrap()),
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

/// A two-link arm, links 2 and 1.5 long, its end put where the angles 0.7
/// and 1.1 put it: those angles are enclosed — not the elbow's other way.
#[test]
fn a_two_link_arm_reaches_with_known_angles_in_every_scalar() {
    let (a, b) = (n(0.7), n(1.1));
    let (l1, l2) = (n(2.0), n(1.5));
    let end = [
        l1.mul(a.cos()).add(l2.mul(a.add(b).cos())),
        l1.mul(a.sin()).add(l2.mul(a.add(b).sin())),
    ];
    let reach = move |v: &[Dual<4>], k: usize| {
        let (l1, l2) = (Dual::cst(l1), Dual::cst(l2));
        let both = v[0].add(v[1]);
        let at = match k {
            0 => l1.mul(v[0].cos()).add(l2.mul(both.cos())),
            _ => l1.mul(v[0].sin()).add(l2.mul(both.sin())),
        };
        vec![at.sub(Dual::cst(end[k]))]
    };
    let formulas = Formulas(vec![
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

/// One line `x + y = 5` over two numbers: one of them is free, and stays
/// exactly as the solve left it; the other is enclosed where the line puts
/// it.
#[test]
fn an_underdetermined_system_keeps_its_free_directions_in_every_scalar() {
    let lines = Lines(vec![[n(1.0), n(1.0), n(5.0)]]);
    let system = System {
        equations: &lines,
        mobility: vec![Mobility::Held; 2],
        scale: Scalar::ONE,
    };
    let solved = system.solve(&[n(1.0), n(1.0)]).unwrap();
    let enclosure = &solved.enclosure;
    assert!(enclosure.failures.is_empty(), "{:?}", enclosure.failures);
    assert_eq!(enclosure.total, 1);
    assert_eq!(enclosure.freedom, [1, 1]);
    let boxes: Vec<Scalar> = enclosure.boxes.iter().map(|b| b.unwrap()).collect();
    let free: Vec<usize> = (0..2)
        .filter(|&k| boxes[k].is_sharp() && boxes[k] == solved.values[k])
        .collect();
    // Both, where the line is met exactly.
    let Some(&free) = free.first() else {
        panic!("no value is free: {boxes:?} for {:?}", solved.values);
    };
    holds(Some(boxes[1 - free]), n(5.0).sub(boxes[free]));
}

/// The unit circle and its tangent `y = 1`, a point on both: the one
/// solution, `(0, 1)`, is singular, and no solve ends exactly there. Not
/// proven, from wherever the solve ends — neither a box claimed, nor a
/// conflict for constraints that hold.
#[test]
fn a_tangency_is_not_proven_in_every_scalar() {
    let formulas = Formulas(vec![
        circle(1.0),
        Box::new(|v| vec![v[1].sub(Dual::cst(Scalar::ONE))]),
    ]);
    let system = numbers(&formulas, 2);
    for x in [0.3, 1e-4, 1e-6, 1e-8] {
        let enclosure = system.enclose(&[n(x), Scalar::ONE]).unwrap();
        let [(_, error)] = &enclosure.failures[..] else {
            panic!("a tangency is proven at {x}: {enclosure:?}");
        };
        assert!(matches!(error, EncloseError::Unproven(_)), "{x}: {error:?}");
        assert!(enclosure.boxes.iter().all(Option::is_none));
    }
}

/// The unit circle and the line `y = 1.5`, which misses it, a point on
/// both: a conflict, named — the two constraints.
#[test]
fn a_line_missing_a_circle_conflicts_in_every_scalar() {
    let formulas = Formulas(vec![
        circle(1.0),
        Box::new(|v| vec![v[1].sub(Dual::cst(n(1.5)))]),
    ]);
    let system = numbers(&formulas, 2);
    let start = [n(0.3), n(0.9)];
    let solved = system.solve(&start).unwrap();
    assert!(!system.held(&solved.values).unwrap().converged);
    let [(_, error)] = &solved.enclosure.failures[..] else {
        panic!("{:?}", solved.enclosure);
    };
    assert!(matches!(error, EncloseError::Conflict { .. }), "{error:?}");
    assert_eq!(system.conflicting(&start).unwrap(), [0, 1]);
}

/// A body pinned to a frame turned by `radians` about `axis`, from `start`:
/// the marks of the enclosed pose are where the frame's are.
fn pinned(start: Pose, axis: [f64; 3], radians: f64, position: [f64; 3]) {
    let direction = Vector3::from_array(axis.map(n)).normalize().unwrap();
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
        let mark = Vector3::from_array(mark.map(n));
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
#[test]
fn poses_far_from_the_start_are_enclosed_in_every_scalar() {
    let pi = std::f64::consts::PI;
    let identity = Pose::identity();
    let off = Pose::from_euler(Vector3::zero(), [n(0.0), n(3.0), n(2.0)]).unwrap();
    pinned(identity, [0.0, 0.0, 1.0], pi - 1e-3, [1.0, -2.0, 0.5]);
    pinned(off, [1.0, 0.0, 0.0], pi, [0.0; 3]);
    pinned(identity, [1.0, 2.0, -0.5], 2.5, [-3.0, 0.25, 4.0]);
}

/// Copies of a circle `x² + y² = r²` and a line `y = a x + c`, each over
/// two numbers of its own — `2k` and `2k + 1` for copy `k`, its circle
/// constraint `2k` and its line `2k + 1` — and how many times they were
/// made.
struct Copies {
    copies: Vec<[Scalar; 3]>,
    evaluations: std::cell::Cell<usize>,
}

impl Copies {
    fn new(copies: Vec<[f64; 3]>) -> Self {
        Self {
            copies: copies.into_iter().map(|c| c.map(n)).collect(),
            evaluations: Default::default(),
        }
    }
}

impl Equations for Copies {
    type Value = Scalar;

    fn count(&self) -> (usize, usize) {
        (2 * self.copies.len(), 0)
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, values: &[Scalar], which: &[Term]) -> Vec<Option<Linearized>> {
        self.evaluations.set(self.evaluations.get() + 1);
        (which.iter())
            .map(|term| {
                let Term::Constraint(c) = term else {
                    return None;
                };
                let k = c / 2;
                let [r, a, offset] = self.copies[k].map(Dual::<2>::cst);
                let (x, y) = (Dual::var(values[2 * k], 0), Dual::var(values[2 * k + 1], 1));
                let residual = match c % 2 {
                    0 => x.mul(x).add(y.mul(y)).sub(r.mul(r)),
                    _ => y.sub(a.mul(x)).sub(offset),
                };
                Some(linearized(&[residual], &[(2 * k, 0..1), (2 * k + 1, 1..2)]))
            })
            .collect()
    }
}

/// How many evaluations each question about `copies`, from `start` (two
/// numbers per copy), takes: a solve and the enclosure of what it solved,
/// an enclosure, freedom and conflicts.
fn evaluations_asking(copies: &[[f64; 3]], start: &[[f64; 2]]) -> [usize; 5] {
    let equations = Copies::new(copies.to_vec());
    let system = System {
        equations: &equations,
        mobility: vec![Mobility::Held; 2 * copies.len()],
        scale: n(5.0),
    };
    let start: Vec<Scalar> = start.iter().flatten().map(|&v| n(v)).collect();
    let solved = system.solve(&start).unwrap();
    let mut counts = [solved.evaluations, solved.enclosure.evaluations, 0, 0, 0];
    let questions: [&dyn Fn(); 3] = [
        &|| drop(system.enclose(&start).unwrap()),
        &|| drop(system.freedom(&start).unwrap()),
        &|| drop(system.conflicting(&start).unwrap()),
    ];
    for (count, question) in counts[2..].iter_mut().zip(questions) {
        equations.evaluations.set(0);
        question();
        *count = equations.evaluations.get();
    }
    counts
}

/// Many independent groups are worked on in lockstep: one evaluation
/// answers every group waiting, so a system of forty copies of a circle
/// crossing a line — and of forty lines missing their circles — takes no
/// more evaluations than its slowest copy does alone. Counted, not timed:
/// asked one group at a time, a program's assembly re-ran per group, and a
/// drag of 2000 parts took seconds.
#[test]
fn independent_groups_are_evaluated_together_in_every_scalar() {
    for line in [[5.0, 1.0, 1.0], [1.0, 0.0, 1.5]] {
        let copies: Vec<[f64; 3]> = vec![line; 40];
        let start: Vec<[f64; 2]> = (0..40)
            .map(|k| [2.5 + 0.02 * k as f64, 3.6 - 0.01 * k as f64])
            .collect();
        let alone: Vec<[usize; 5]> = (copies.iter().zip(&start))
            .map(|(copy, start)| evaluations_asking(&[*copy], &[*start]))
            .collect();
        let slowest: [usize; 5] =
            std::array::from_fn(|q| alone.iter().map(|counts| counts[q]).max().unwrap_or(0));
        let together = evaluations_asking(&copies, &start);
        assert!(
            (0..5).all(|q| together[q] <= slowest[q]),
            "{line:?}: {together:?} evaluations for 40 copies, the slowest alone {slowest:?}"
        );
    }
}

/// The unit circle and the line `y = 1.5`, which misses it, from `(3.26,
/// 3.22)`: the solve ends. Once, in fixed point, the damping shrank to
/// zero after enough steps taken, no growth brought it back, and a step
/// turned down was tried again as it was, forever.
#[test]
fn a_line_missing_a_circle_from_afar_ends() {
    let copies = Copies::new(vec![[1.0, 0.0, 1.5]]);
    let system = System {
        equations: &copies,
        mobility: vec![Mobility::Held; 2],
        scale: n(5.0),
    };
    let start = [n(3.26), n(3.22)];
    let solved = system.solve(&start).unwrap();
    assert!(!system.held(&solved.values).unwrap().converged);
    assert_eq!(system.conflicting(&start).unwrap(), [0, 1]);
}
