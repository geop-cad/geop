//! The point at given offsets from several surfaces: where a shell moves a
//! vertex, onto the inner copies of its faces, where a draft moves one onto
//! its tilted faces, and where a blend's ball touches the faces it rounds.
//!
//! One condition per smooth surface ([`Offset`]): the point lies its
//! `offset` along the surface's unit normal, behind it for a negative one.
//! A surface may come in pieces — the quarters of a revolved face meeting
//! at its pole — and is then one condition, not several ([`tangent_groups`]).
//! Planes through a point add linear conditions. [`meet`] finds the point
//! nearest its start that meets all of them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::{
        iterate::{Repeat, Step, iterate},
        linear::shortest_solution,
    },
    vector::Vector3,
    with_context,
};

use crate::{nurb_surface::NurbSurface3D, search::PROJECT_ITERATIONS, shape::Plane};

/// Newton steps [`meet`] takes at most: between planes it lands in one, and
/// against curved surfaces it converges quadratically from a start a
/// thickness or a radius away.
const MEET_ITERATIONS: usize = 30;

/// A condition on the point [`meet`] finds: that it lie `offset` along the
/// unit normal of the smooth surface `pieces` are the pieces of, each with
/// where to start looking for the foot point on it. [`meet`] leaves there
/// the foot point of its last seed, sharp: a seed for the next search.
#[derive(Clone, Debug)]
pub struct Offset<'a> {
    pub pieces: Vec<(&'a NurbSurface3D, (Scalar, Scalar))>,
    pub offset: Scalar,
}

impl<'a> Offset<'a> {
    /// The condition to lie `offset` from `surface`, its foot point near
    /// `uv`.
    pub fn new(surface: &'a NurbSurface3D, uv: (Scalar, Scalar), offset: Scalar) -> Self {
        Offset {
            pieces: vec![(surface, uv)],
            offset,
        }
    }

    /// The plane each piece lies in, if it is flat ([`Self::foot`]).
    fn planes(&self) -> GeopResult<Vec<Option<Plane>>> {
        self.pieces
            .iter()
            .map(|(surface, _)| surface.as_plane())
            .collect()
    }

    /// The foot point of `p` on the union of the pieces — the nearest of
    /// theirs, which piece being a free choice where they tie — and the
    /// unit normal there. Each piece's `(u, v)` becomes its own foot
    /// point, sharp, where the next search starts: projection drops the
    /// surface's second derivatives and converges only linearly on a
    /// curved one, so it has to start near to end on the foot point.
    ///
    /// A piece that is flat ([`Self::planes`]) is its plane: the foot point
    /// is `p` dropped onto it and the normal the plane's, wherever `p` is.
    /// Through the patch, the foot point is clamped to it and the normal
    /// read off `S_u × S_v`, and at a pole neither says what the plane
    /// does. A disc's pole row is a point only to the width of its control
    /// points; a hair off it `S_u` is of the order of that hair, and the
    /// normal there as wide as the width over the hair: at `5e-14` from the
    /// pole of a cap, `±0.4`. Where a shell moves a vertex onto the axis of
    /// a revolved cap, every step passes there.
    ///
    /// Where the foot point's enclosure reaches an end of the domain, the
    /// seed is that end. Any point of the enclosure is a valid seed, but at
    /// a pole the enclosure reaches a hair off it, and a midpoint there
    /// lies so close to the pole that the surface's derivatives underflow
    /// and it has no normal; the pole itself has one.
    fn foot(&mut self, p: &Vector3, planes: &[Option<Plane>]) -> GeopResult<(Vector3, Vector3)> {
        let mut best: Option<(f64, Vector3, Vector3)> = None;
        for (k, (surface, uv)) in self.pieces.iter_mut().enumerate() {
            if let Some(plane) = &planes[k] {
                let at = plane.project(p);
                let distance = at.sub(p).norm_sq().upper().to_f64();
                if best.as_ref().is_none_or(|(d, _, _)| distance < *d) {
                    best = Some((distance, at, plane.normal));
                }
                continue;
            }
            let ctx = with_context!("the foot point of {p:?} on piece {k}, from {uv:?}");
            let (u, v) = surface
                .project_in_domain(*p, uv.0.sharpen(), uv.1.sharpen(), PROJECT_ITERATIONS)
                .with_context(ctx)?;
            let at = surface.evaluate(u, v).with_context(ctx)?;
            let distance = at.sub(p).norm_sq().upper().to_f64();
            if best.as_ref().is_none_or(|(d, _, _)| distance < *d) {
                let normal = surface.normal(u, v).with_context(with_context!(
                    "the normal at the foot point {:?} of {p:?} on piece {k}",
                    (u, v)
                ))?;
                best = Some((distance, at, normal));
            }
            let seed = |t: Scalar, (lo, hi): (Scalar, Scalar)| {
                if t.could_be_equal(lo) {
                    lo
                } else if t.could_be_equal(hi) {
                    hi
                } else {
                    t.sharpen().clamped(lo, hi)
                }
            };
            *uv = (seed(u, surface.domain_u()), seed(v, surface.domain_v()));
        }
        let (_, at, normal) = best.ok_or_else(|| GeopError::new("a surface of no pieces"))?;
        Ok((at, normal))
    }
}

/// `pieces`, each a surface and where a point is on it, grouped into the
/// smooth surfaces they are pieces of there: those whose normals there
/// could be parallel. The groups as indices into `pieces`, in order.
///
/// Decided once, at the point, not at the foot points a search moves
/// through: each piece clamps those to its own patch, their normals there
/// differ a little, and nearly equal conditions leave no step to take.
pub fn tangent_groups(
    pieces: &[(&NurbSurface3D, (Scalar, Scalar))],
) -> GeopResult<Vec<Vec<usize>>> {
    let mut groups: Vec<(Vector3, Vec<usize>)> = Vec::new();
    for (i, (surface, (u, v))) in pieces.iter().enumerate() {
        let normal = surface
            .normal(*u, *v)
            .with_context(with_context!("the normal of piece {i} at {:?}", (u, v)))?;
        match groups
            .iter_mut()
            .find(|(n, _)| n.prod_cross(&normal).norm_sq().could_be_equal(Scalar::ZERO))
        {
            Some((_, members)) => members.push(i),
            None => groups.push((normal, vec![i])),
        }
    }
    Ok(groups.into_iter().map(|(_, members)| members).collect())
}

/// The point nearest `start` that lies at every one of `offsets` and in
/// every one of `planes` (a unit normal and a point each): Gauss-Newton.
/// Each step measures how far off each condition the iterate is — along
/// the normal at its foot point on each surface, along each plane's normal
/// — and takes the shortest step that corrects all of them at once, to
/// first order. Between planes that is exact in one step; with fewer than
/// three conditions the point slides along where they meet, by no more
/// than it has to.
///
/// The iterates are seeds for the next step, so they are sharpened — but
/// they are also the answer: where fewer than three conditions meet, the
/// point is free to slide, and only the steps taken say how far it has. So
/// the answer is carried alongside, unsharpened: `start` less every step,
/// each with the width its own measurement gave it (keep two copies, see
/// `AGENTS.md`). A caller that only needs a construction aid sharpens it,
/// where it calls. A fixed point of the sharpened iteration ends it: every
/// further step would start from the very same seed and only add its width
/// again — not a tolerance, the seed is bit for bit unchanged.
///
/// The answer is checked against every condition at the end: a condition
/// lost on the way — a foot point clamped to the edge of its patch, a
/// projection that never moved — is an error here, naming the condition,
/// not a wrong point found two steps later.
pub fn meet(
    start: Vector3,
    offsets: &mut [Offset],
    planes: &[(Vector3, Vector3)],
) -> GeopResult<Vector3> {
    let mut answer = start;
    let flat: Vec<Vec<Option<Plane>>> = offsets
        .iter()
        .map(Offset::planes)
        .collect::<GeopResult<_>>()?;
    iterate(
        start.sharpen(),
        start,
        MEET_ITERATIONS,
        Repeat::FixedPoint,
        |next| (next.sharpen(), *next),
        |seed| -> GeopResult<Step<Vector3>> {
            let mut rows: Vec<(Vector3, Scalar)> = Vec::new();
            for (condition, flat) in offsets.iter_mut().zip(&flat) {
                let (at, normal) = condition.foot(seed, flat)?;
                rows.push((
                    normal,
                    normal.prod_dot(&seed.sub(&at)).sub(condition.offset),
                ));
            }
            for (normal, p) in planes {
                rows.push((*normal, normal.prod_dot(&seed.sub(p))));
            }
            // The seed meets every condition as far as they say where: a
            // further step would only add its width. On a fitted surface,
            // as wide as the fit, the sharpened iterates never repeat bit
            // for bit, and thirty steps' widths summed made a sheet's
            // corner thirty times as wide as the surface.
            if rows.iter().all(|(_, off)| off.could_be_equal(Scalar::ZERO)) {
                return Ok(Step::Done);
            }
            let step = shortest_solution(&rows).with_context(&|e: GeopError| {
                let feet: Vec<Vec<(Scalar, Scalar)>> = offsets
                    .iter()
                    .map(|o| o.pieces.iter().map(|&(_, uv)| uv).collect())
                    .collect();
                e.with_context(format!(
                    "the step of meet from {seed:?}, with the rows {rows:?}, the foot points at {feet:?}"
                ))
            })?;
            answer = answer.sub(&step);
            Ok(Step::Next(seed.sub(&step)))
        },
    )?;
    let count = offsets.len();
    for (k, (condition, flat)) in offsets.iter_mut().zip(&flat).enumerate() {
        let (at, normal) = condition.foot(&answer, flat)?;
        if !at
            .add(&normal.prod_scalar(condition.offset))
            .could_be_equal(&answer)
        {
            return Err(GeopError::new(format!(
                "the point {answer:?}, found from {start:?}, is not {:?} from surface {k} of {} (of {} pieces), whose nearest point is {at:?}, normal {normal:?}",
                condition.offset,
                count,
                condition.pieces.len(),
            )));
        }
    }
    for (k, (normal, p)) in planes.iter().enumerate() {
        let off = normal.prod_dot(&answer.sub(p));
        if !off.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "the point {answer:?}, found from {start:?}, is {off:?} off plane {k} of {}, through {p:?} with normal {normal:?}",
                planes.len()
            )));
        }
    }
    Ok(answer)
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use super::{Offset, meet};
    use crate::test_fixtures::sphere_octant;

    fn point(p: [f64; 3]) -> Vector3 {
        Vector3::from_array(p.map(Scalar::from_f64))
    }

    /// A point half a unit inside the unit sphere and in the plane
    /// `x = 0.3`: two conditions, so it slides from where it starts, and
    /// the answer encloses a point that meets both.
    #[test]
    fn slides_onto_an_offset_sphere_and_a_plane() {
        let sphere = sphere_octant();
        let half = Scalar::from_f64(0.5);
        let inward = sphere.normal(half, half).unwrap();
        // Half a unit towards the center, whichever way the normal faces.
        let offset = if inward
            .prod_dot(&point([1., 1., 1.]))
            .definitely_less(Scalar::ZERO)
        {
            half
        } else {
            half.neg()
        };
        let mut offsets = [Offset::new(&sphere, (half, half), offset)];
        let plane = (point([1., 0., 0.]), point([0.3, 0., 0.]));
        let p = meet(point([0.3, 0.3, 0.3]), &mut offsets, &[plane]).unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(0.3)), "{p:?}");
        assert!(p.norm().could_be_equal(half), "{p:?}");
        // Fixed point evaluates the rational sphere far less tightly.
        assert!(p.norm().width().to_f64() < 1e-5, "{p:?}");
    }

    /// A point over the sphere's pole: its foot point stays there, where
    /// the parametrization is singular and only the pole itself has a
    /// normal — and the foot points left behind seed the same answer again.
    /// (A pole at the start of a domain, where a seed a hair off it would
    /// underflow, is the shell's test `a_closed_revolved_cylinder`.)
    #[test]
    fn stays_over_a_pole() {
        let sphere = sphere_octant();
        let half = Scalar::from_f64(0.5);
        let mut offsets = [Offset::new(&sphere, (half, Scalar::ONE), half)];
        let p = meet(point([0., 0., 1.2]), &mut offsets, &[]).unwrap();
        let other = meet(point([0., 0., 1.2]), &mut offsets, &[]).unwrap();
        assert!(
            p.could_be_equal(&point([0., 0., 1.5])) || p.could_be_equal(&point([0., 0., 0.5])),
            "{p:?}"
        );
        assert!(other.could_be_equal(&p), "{other:?} {p:?}");
    }
}
