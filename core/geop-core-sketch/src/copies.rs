//! Copies of sketch geometry that stay copies: mirrored across a line, or
//! repeated along a line or round a center. Every copied point is tied to
//! the point it copies by a constraint — [`Constraint::Symmetric`] for a
//! mirror, [`Constraint::Moved`] for a pattern — and every copied arc's
//! sweep to the original's ([`Constraint::EqualSweep`]), every circle's
//! radius ([`Constraint::Equal`]). So the copies follow whatever is done to
//! the original: dragged, dimensioned, solved.
//!
//! **Patterns are structure, not records.** A pattern is its construction
//! curve `by` — a line for a linear pattern, an arc for a circular one —
//! and the [`Constraint::Moved`] constraints that name it: each copy is the
//! previous one moved by `by`. That is all there is to know about it: how
//! many copies it has ([`Sketch::pattern_count`]) is read off the chain of
//! constraints, and changing that ([`Sketch::set_pattern_count`]) adds or
//! removes copies at its end. Its spacing is a dimension of `by` — a
//! [`Constraint::Length`] of the line, an [`Constraint::Angle`] between
//! the radii to the arc's ends — edited like any other.
//!
//! Positions here are where the copies are placed before the sketch is
//! solved: free choices, computed in plain numbers (see [`crate::plain`]).

use std::collections::{BTreeMap, BTreeSet, btree_map::Entry};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use crate::{
    plain::{P2, add, dist, reflect, rotate, scale, sub, unit, xy},
    sketch::{Constraint, ConstraintId, Curve, CurveId, CurveKind, PointId, Sketch},
};

/// How a pattern moves each copy on from the one before.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step<S: Scalar> {
    /// Along the line `along` — towards its end, or its start if
    /// `backwards` — by `spacing`.
    Along {
        along: CurveId,
        backwards: bool,
        spacing: S,
    },
    /// Round the point `center`, counter-clockwise by `angle` radians
    /// (clockwise if negative).
    Round { center: PointId, angle: S },
}

/// A pattern made: its construction curve, and the dimension of its
/// spacing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Made {
    pub by: CurveId,
    pub spacing: ConstraintId,
}

/// The motion of a pattern's step, in plain numbers: `q ↦ e + R(q - s)`.
#[derive(Clone, Copy, Debug)]
struct Motion {
    s: P2,
    e: P2,
    turn: f64,
}

impl Motion {
    fn apply(&self, q: P2) -> P2 {
        add(self.e, rotate(sub(q, self.s), self.turn))
    }

    /// The motion `by` stands for, as drawn: see [`Constraint::Moved`].
    fn of<S: Scalar>(sketch: &Sketch<S>, by: CurveId) -> GeopResult<Motion> {
        Ok(match sketch.curve(by)?.kind {
            CurveKind::Line { start, end } => Motion {
                s: xy(sketch, start),
                e: xy(sketch, end),
                turn: 0.0,
            },
            CurveKind::Arc { start, end, sweep } => Motion {
                s: xy(sketch, start),
                e: xy(sketch, end),
                turn: sweep.to_f64(),
            },
            _ => {
                return Err(GeopError::new(format!(
                    "curve {by} is no line or arc: it moves nothing"
                )));
            }
        })
    }
}

impl<S: Scalar> Sketch<S> {
    fn new_point_at(&mut self, at: P2) -> PointId {
        self.add_point(S::from_f64(at[0]), S::from_f64(at[1]))
    }

    /// A copy of the curve `source`, its points mapped by `point`, tied to
    /// it where its points cannot say it all: an arc's sweep, a circle's
    /// radius. As construction geometry if the source is; never fixed.
    /// `reverse`: a mirror image, which runs the other way round.
    fn copy_curve(
        &mut self,
        source: CurveId,
        point: &dyn Fn(PointId) -> PointId,
        reverse: bool,
    ) -> CurveId {
        let Curve {
            kind, construction, ..
        } = self.curves[&source].clone();
        let kind = match kind {
            CurveKind::Line { start, end } => CurveKind::Line {
                start: point(start),
                end: point(end),
            },
            // Mirrored, an arc from `s` to `e` counter-clockwise is one
            // from `e'` to `s'`, counter-clockwise by as much.
            CurveKind::Arc { start, end, sweep } if reverse => CurveKind::Arc {
                start: point(end),
                end: point(start),
                sweep,
            },
            CurveKind::Arc { start, end, sweep } => CurveKind::Arc {
                start: point(start),
                end: point(end),
                sweep,
            },
            CurveKind::Circle { center, radius } => CurveKind::Circle {
                center: point(center),
                radius,
            },
            CurveKind::Spline {
                control_points,
                shape,
            } => CurveKind::Spline {
                control_points: control_points.iter().map(|&p| point(p)).collect(),
                shape,
            },
        };
        let copy = self.add_curve(kind);
        self.set_construction(copy, construction);
        match self.curves[&copy].kind {
            CurveKind::Arc { .. } => {
                self.constrain(Constraint::EqualSweep { a: source, b: copy });
            }
            CurveKind::Circle { .. } => {
                self.constrain(Constraint::Equal { a: source, b: copy });
            }
            _ => {}
        }
        copy
    }

    /// The curves `curves` checked to be ones a copy can be made of, without
    /// `except` — what the copy is made by — and without repeats.
    fn copyable(&self, curves: &[CurveId], except: Option<CurveId>) -> GeopResult<Vec<CurveId>> {
        let mut out = Vec::new();
        for &c in curves {
            self.curve(c)?;
            if Some(c) != except && !out.contains(&c) {
                out.push(c);
            }
        }
        if out.is_empty() {
            return Err(GeopError::new(
                "nothing to copy: pick curves besides the one copied by",
            ));
        }
        Ok(out)
    }

    /// Mirrors `curves` across the line `line`, each mirrored point
    /// [`Constraint::Symmetric`] to its original. A point on the line —
    /// by constraint, see [`Sketch::on_line`] — is its own mirror image,
    /// and a point mirrored before across the same line has the image it
    /// got then: so a half profile ending on the line closes into a whole
    /// one, and mirroring one curve after another joins the copies up. A
    /// curve that lies on the line is left out. Returns the new curves.
    pub fn mirror(&mut self, curves: &[CurveId], line: CurveId) -> GeopResult<Vec<CurveId>> {
        let CurveKind::Line { start, end } = self.curve(line)?.kind else {
            return Err(GeopError::new(format!(
                "curve {line} is no line: only a line can be mirrored across"
            )));
        };
        let curves = self.copyable(curves, Some(line))?;
        let (a, b) = (xy(self, start), xy(self, end));
        let class = self.point_classes();
        let on = self.on_line(line)?;
        // Images known already: the points' own, and those from mirroring
        // before.
        let mut image: BTreeMap<PointId, PointId> = BTreeMap::new();
        for &p in &on {
            image.insert(class[&p], p);
        }
        for c in self.constraints.values() {
            if let Constraint::Symmetric { a, b, line: l } = *c
                && l == line
            {
                image.entry(class[&a]).or_insert(b);
                image.entry(class[&b]).or_insert(a);
            }
        }
        let mut made = Vec::new();
        for c in curves {
            let points = self.curves[&c].points();
            if points.iter().all(|p| on.contains(p)) {
                continue;
            }
            for p in points {
                if let Entry::Vacant(slot) = image.entry(class[&p]) {
                    let q = self.new_point_at(reflect(xy(self, p), a, b));
                    self.constrain(Constraint::Symmetric { a: p, b: q, line });
                    slot.insert(q);
                }
            }
            let mapped = |p: PointId| image[&class[&p]];
            made.push(self.copy_curve(c, &mapped, true));
        }
        Ok(made)
    }

    /// Repeats `curves` `count` times in all, the original among them, each
    /// copy `step` on from the one before (see the module docs). Returns the
    /// pattern's construction curve and the dimension of its spacing.
    ///
    /// The construction curve runs from a point of the original — for a
    /// circular pattern the one furthest from the center — to that point's
    /// first copy. A point at a circular pattern's center is its own copy.
    pub fn pattern(
        &mut self,
        curves: &[CurveId],
        step: &Step<S>,
        count: usize,
    ) -> GeopResult<Made> {
        if count < 2 {
            return Err(GeopError::new(format!(
                "a pattern of {count} is no pattern: it takes 2 or more"
            )));
        }
        match *step {
            Step::Along { spacing, .. } if !spacing.definitely_greater(S::ZERO) => {
                return Err(GeopError::new(format!(
                    "a linear pattern's spacing must be more than 0, not {spacing:?}"
                )));
            }
            Step::Round { angle, .. } => once_round(angle, count)?,
            _ => {}
        }
        let except = match *step {
            Step::Along { along, .. } => Some(along),
            Step::Round { .. } => None,
        };
        let curves = self.copyable(curves, except)?;
        let class = self.point_classes();
        let mut sources: Vec<PointId> = Vec::new();
        for c in &curves {
            for p in self.curves[c].points() {
                if !sources.iter().any(|&q| class[&q] == class[&p]) {
                    sources.push(p);
                }
            }
        }
        let (by, spacing, center) = match *step {
            Step::Along {
                along,
                backwards,
                spacing,
            } => {
                let CurveKind::Line { start, end } = self.curve(along)?.kind else {
                    return Err(GeopError::new(format!(
                        "curve {along} is no line: a linear pattern runs along a line"
                    )));
                };
                let direction = unit(sub(xy(self, end), xy(self, start))).ok_or_else(|| {
                    GeopError::new(format!("line {along} has no length: it gives no direction"))
                })?;
                let direction = if backwards {
                    scale(direction, -1.0)
                } else {
                    direction
                };
                let anchor = sources[0];
                let first =
                    self.new_point_at(add(xy(self, anchor), scale(direction, spacing.to_f64())));
                let by = self.add_line(anchor, first);
                self.set_construction(by, true);
                self.constrain(Constraint::Parallel { a: along, b: by });
                let spacing = self.constrain(Constraint::Length {
                    curve: by,
                    value: spacing,
                });
                (by, spacing, None)
            }
            Step::Round { center, angle } => {
                self.point(center)?;
                let c = xy(self, center);
                let anchor = sources
                    .iter()
                    .copied()
                    .filter(|&p| class[&p] != class[&center])
                    .max_by(|&p, &q| dist(xy(self, p), c).total_cmp(&dist(xy(self, q), c)))
                    .filter(|&p| dist(xy(self, p), c) > 0.0)
                    .ok_or_else(|| {
                        GeopError::new(format!(
                            "every point copied is at the center {center}: turning moves nothing"
                        ))
                    })?;
                let turn = angle.to_f64();
                let at = add(c, rotate(sub(xy(self, anchor), c), turn));
                let first = self.new_point_at(at);
                let by = self.add_arc(anchor, first, angle);
                self.set_construction(by, true);
                self.constrain(Constraint::Center {
                    point: center,
                    curve: by,
                });
                let radii = [anchor, first].map(|p| {
                    let r = self.add_line(center, p);
                    self.set_construction(r, true);
                    r
                });
                let spacing = self.constrain(Constraint::Angle {
                    a: radii[0],
                    b: radii[1],
                    value: angle,
                });
                (by, spacing, Some(center))
            }
        };
        // The first copy of the anchor is the end of `by`: moved by it, as
        // all copies are — trivially so, which is what marks every point
        // of the original as copied, a lone one too.
        let (anchor, first) = self.curves[&by].endpoints().expect("a line or arc");
        self.constrain(Constraint::Moved {
            a: anchor,
            b: first,
            by,
        });
        let mut previous: BTreeMap<PointId, PointId> =
            sources.iter().map(|&p| (class[&p], p)).collect();
        if let Some(center) = center {
            previous.insert(class[&center], center);
        }
        let generation = Generation {
            points: previous,
            curves,
        };
        self.extend_pattern(by, generation, count - 1, Some(first))?;
        Ok(Made { by, spacing })
    }

    /// Adds `more` copies after the copy `last` of the pattern `by`: each
    /// point moved on from the last copy's, each curve copied. `anchor`:
    /// the first copy of the anchor, there already (the end of `by`).
    fn extend_pattern(
        &mut self,
        by: CurveId,
        mut last: Generation,
        more: usize,
        mut anchor: Option<PointId>,
    ) -> GeopResult<()> {
        let motion = Motion::of(self, by)?;
        let (by_start, _) = self.curves[&by].endpoints().expect("a line or arc");
        for _ in 0..more {
            let class = self.point_classes();
            let mut points = BTreeMap::new();
            for (&rep, &p) in &last.points {
                // A point the motion keeps where it is — a circular
                // pattern's center — is its own copy.
                let fixed = self.is_pattern_center(by, p);
                let q = if fixed {
                    p
                } else if let Some(first) = anchor.filter(|_| class[&p] == class[&by_start]) {
                    first
                } else {
                    let q = self.new_point_at(motion.apply(xy(self, p)));
                    self.constrain(Constraint::Moved { a: p, b: q, by });
                    q
                };
                points.insert(rep, q);
            }
            anchor = None;
            let mut curves = Vec::new();
            let class = self.point_classes();
            for &c in &last.curves {
                let mapped = |p: PointId| points[&class[&p]];
                curves.push(self.copy_curve(c, &mapped, false));
            }
            // From here on, a copy's points are known by their own class.
            let class = self.point_classes();
            last = Generation {
                points: points.values().map(|&q| (class[&q], q)).collect(),
                curves,
            };
        }
        Ok(())
    }

    /// Whether `p` is the center of the circular pattern `by`.
    fn is_pattern_center(&self, by: CurveId, p: PointId) -> bool {
        let class = self.point_classes();
        self.constraints.values().any(|c| {
            matches!(*c, Constraint::Center { point, curve } if curve == by && class[&point] == class[&p])
        })
    }

    /// Every point the pattern `by` moves, by how many steps from the
    /// original: the chain of its [`Constraint::Moved`] constraints. `None`
    /// if `by` is the construction curve of no pattern.
    fn pattern_steps(&self, by: CurveId) -> Option<BTreeMap<PointId, usize>> {
        let moved: Vec<(PointId, PointId)> = self
            .constraints
            .values()
            .filter_map(|c| match *c {
                Constraint::Moved { a, b, by: m } if m == by => Some((a, b)),
                _ => None,
            })
            .collect();
        if moved.is_empty() {
            return None;
        }
        let targets: BTreeSet<PointId> = moved.iter().map(|&(_, b)| b).collect();
        let mut steps: BTreeMap<PointId, usize> = moved
            .iter()
            .filter(|(a, _)| !targets.contains(a))
            .map(|&(a, _)| (a, 0))
            .collect();
        // Each copy one step on from the point it is moved from; at most
        // as many rounds as there are constraints.
        for _ in 0..moved.len() {
            let mut changed = false;
            for &(a, b) in &moved {
                if let Some(&k) = steps.get(&a)
                    && !steps.contains_key(&b)
                {
                    steps.insert(b, k + 1);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        Some(steps)
    }

    /// How many copies the pattern `by` has, the original among them;
    /// `None` if `by` is the construction curve of no pattern.
    pub fn pattern_count(&self, by: CurveId) -> Option<usize> {
        Some(self.pattern_steps(by)?.values().max()? + 1)
    }

    /// Gives the pattern `by` `count` copies, the original among them:
    /// removes the copies past the last one kept, or adds copies after its
    /// last one, as it is now.
    pub fn set_pattern_count(&mut self, by: CurveId, count: usize) -> GeopResult<()> {
        let steps = self
            .pattern_steps(by)
            .ok_or_else(|| GeopError::new(format!("curve {by} is no pattern's")))?;
        if count < 2 {
            return Err(GeopError::new(format!(
                "a pattern of {count} is no pattern: it takes 2 or more"
            )));
        }
        if let Some(Constraint::Angle { value, .. }) = self
            .pattern_spacing(by)
            .and_then(|k| self.constraints.get(&k))
        {
            once_round(*value, count)?;
        }
        let now = steps.values().max().copied().unwrap_or(0) + 1;
        if count < now {
            let gone: Vec<PointId> = steps
                .iter()
                .filter(|&(_, &k)| k >= count)
                .map(|(&p, _)| p)
                .collect();
            self.remove(&gone, &[], &[]);
            return Ok(());
        }
        if count == now {
            return Ok(());
        }
        // The last copy: its points, and the curves on them — on them
        // only, the center of a circular pattern aside, and not the
        // pattern's own construction.
        let class = self.point_classes();
        let mut last: BTreeMap<PointId, PointId> = steps
            .iter()
            .filter(|&(_, &k)| k == now - 1)
            .map(|(&p, _)| (class[&p], p))
            .collect();
        let own = self.pattern_own_curves(by);
        let center = self.constraints.values().find_map(|c| match *c {
            Constraint::Center { point, curve } if curve == by => Some(point),
            _ => None,
        });
        let curves: Vec<CurveId> = self
            .curves
            .iter()
            .filter(|(id, c)| {
                let points = c.points();
                !own.contains(id)
                    && points.iter().any(|p| last.contains_key(&class[p]))
                    && points.iter().all(|p| {
                        last.contains_key(&class[p])
                            || center.is_some_and(|m| class[&m] == class[p])
                    })
            })
            .map(|(&id, _)| id)
            .collect();
        if let Some(m) = center {
            last.insert(class[&m], m);
        }
        self.extend_pattern(
            by,
            Generation {
                points: last,
                curves,
            },
            count - now,
            None,
        )
    }

    /// The dimension of the spacing of the pattern `by`: a
    /// [`Constraint::Length`] of its line, or the [`Constraint::Angle`]
    /// between the radii to its arc's ends — if it has one still.
    pub fn pattern_spacing(&self, by: CurveId) -> Option<ConstraintId> {
        let (s, e) = self.curves.get(&by)?.endpoints()?;
        let ends = |l: CurveId| self.curves.get(&l).and_then(|k| k.endpoints());
        self.constraints.iter().find_map(|(&id, c)| match *c {
            Constraint::Length { curve, .. } if curve == by => Some(id),
            Constraint::Angle { a, b, .. }
                if ends(a).is_some_and(|(_, q)| q == s) && ends(b).is_some_and(|(_, q)| q == e) =>
            {
                Some(id)
            }
            _ => None,
        })
    }

    /// The construction a pattern made for itself: `by`, and a circular
    /// pattern's radii to its ends.
    fn pattern_own_curves(&self, by: CurveId) -> Vec<CurveId> {
        let mut own = vec![by];
        if let Some(Constraint::Angle { a, b, .. }) = self
            .pattern_spacing(by)
            .and_then(|k| self.constraints.get(&k))
        {
            own.extend([*a, *b]);
        }
        own
    }
}

/// Checks that `count` copies `angle` apart turn — they do not stay put —
/// and go round once at most, the last short of the first.
fn once_round<S: Scalar>(angle: S, count: usize) -> GeopResult<()> {
    let turned = angle.abs().mul(S::from_f64((count - 1) as f64));
    if angle.could_be_equal(S::ZERO) || !turned.definitely_less(S::TWO.mul(S::PI)) {
        return Err(GeopError::new(format!(
            "{count} copies {:.3}° apart do not go round once short of the first: a circular pattern turns, once at most",
            angle.to_f64().to_degrees()
        )));
    }
    Ok(())
}

/// One copy of a pattern: its points by class, and its curves.
struct Generation {
    points: BTreeMap<PointId, PointId>,
    curves: Vec<CurveId>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plain::Plain;
    use geop_core_math::scalars::ScalInF64;
    use std::f64::consts::PI;

    type T = ScalInF64;

    fn n(x: f64) -> T {
        T::from_f64(x)
    }

    fn close(a: P2, b: P2) -> bool {
        dist(a, b) < 1e-7
    }

    /// A unit square at `(x, y)`, its corner fixed and its sides
    /// dimensioned: fully constrained.
    fn square(s: &mut Sketch<T>, x: f64, y: f64) -> Vec<CurveId> {
        let p = [[x, y], [x + 1.0, y], [x + 1.0, y + 1.0], [x, y + 1.0]]
            .map(|q| s.add_point(n(q[0]), n(q[1])));
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(x),
            y: n(y),
        });
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[3] });
        s.constrain(Constraint::Length {
            curve: l[0],
            value: n(1.0),
        });
        s.constrain(Constraint::Length {
            curve: l[1],
            value: n(1.0),
        });
        l
    }

    fn y_axis(s: &mut Sketch<T>) -> CurveId {
        let o = s.add_fixed_point(n(0.0), n(0.0));
        let up = s.add_fixed_point(n(0.0), n(1.0));
        s.add_line(o, up)
    }

    /// A half slot right of the y axis, its ends on the axis, mirrored:
    /// one closed slot whose mirrored half follows the original's radius.
    #[test]
    fn a_mirrored_half_closes_on_the_line() {
        let mut s = Sketch::<T>::new();
        let axis = y_axis(&mut s);
        let p =
            [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let bottom = s.add_line(p[0], p[1]);
        let round = s.add_arc(p[1], p[2], n(PI));
        let top = s.add_line(p[2], p[3]);
        for q in [p[0], p[3]] {
            s.constrain(Constraint::PointOnCurve {
                point: q,
                curve: axis,
            });
        }
        s.constrain(Constraint::Tangent {
            a: bottom,
            b: round,
        });
        s.constrain(Constraint::Tangent { a: round, b: top });
        let made = s.mirror(&[bottom, round, top, axis], axis).unwrap();
        assert_eq!(made.len(), 3);
        assert_eq!(s.regions().unwrap().len(), 1, "one closed region");
        // Its ends were on the axis, and are shared: two new points.
        assert_eq!(s.points.len(), 4 + 2 + 2);
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        // The original's radius made 0.75: the image's follows.
        s.constrain(Constraint::Radius {
            curve: round,
            value: n(0.75),
        });
        assert!(s.solve().unwrap().converged);
        let image = made[1];
        let Some(Plain::Round { radius, center, .. }) = Plain::of(&s, image, false) else {
            panic!("an arc");
        };
        let Some(Plain::Round {
            center: original, ..
        }) = Plain::of(&s, round, false)
        else {
            panic!("an arc");
        };
        assert!((radius - 0.75).abs() < 1e-7, "{radius}");
        assert!(
            close(center, [-original[0], original[1]]),
            "{center:?} {original:?}"
        );
        s.enclose::<T>().unwrap();
    }

    /// Mirroring a semicircle's arc — where equal radii say nothing about
    /// the sweep — keeps the copy's sweep by constraint, and the solution
    /// is proven.
    #[test]
    fn mirrored_half_circles_keep_their_sweep() {
        let mut s = Sketch::<T>::new();
        let axis = y_axis(&mut s);
        let a = s.add_point(n(1.0), n(0.0));
        let b = s.add_point(n(3.0), n(0.0));
        let c = s.add_point(n(3.0), n(-1.0));
        let arc = s.add_arc(a, b, n(-PI));
        let down = s.add_line(b, c);
        for (q, at) in [(a, [1.0, 0.0]), (b, [3.0, 0.0]), (c, [3.0, -1.0])] {
            s.constrain(Constraint::Fix {
                point: q,
                x: n(at[0]),
                y: n(at[1]),
            });
        }
        // Its end square to the line down: a half circle, by tangency.
        s.constrain(Constraint::Tangent { a: arc, b: down });
        let made = s.mirror(&[arc], axis).unwrap();
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert!(
            matches!(s.curves[&made[0]].kind, CurveKind::Arc { sweep, .. } if (sweep.to_f64() + PI).abs() < 1e-9)
        );
        s.enclose::<T>().unwrap();
    }

    /// Three squares along the x axis, 2 apart: the copies follow the
    /// original and the spacing; more and fewer copies are added and
    /// removed at the end of the row.
    #[test]
    fn linear_patterns_follow_their_spacing_and_count() {
        let mut s = Sketch::<T>::new();
        let o = s.add_fixed_point(n(0.0), n(0.0));
        let x = s.add_fixed_point(n(1.0), n(0.0));
        let x_axis = s.add_line(o, x);
        let square = square(&mut s, 0.5, 0.5);
        let made = s
            .pattern(
                &square,
                &Step::Along {
                    along: x_axis,
                    backwards: false,
                    spacing: n(2.0),
                },
                3,
            )
            .unwrap();
        assert_eq!(s.pattern_count(made.by), Some(3));
        assert_eq!(s.pattern_count(x_axis), None);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert_eq!(s.regions().unwrap().len(), 3);
        let corners = |s: &Sketch<T>| -> Vec<P2> {
            let mut c: Vec<P2> = s.points.keys().map(|&p| xy(s, p)).collect();
            c.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
            c
        };
        assert!(corners(&s).iter().any(|&q| close(q, [5.5, 1.5])));
        // The spacing made 3: the row stretches.
        if let Some(Constraint::Length { value, .. }) = s.constraints.get_mut(&made.spacing) {
            *value = n(3.0);
        }
        assert!(s.solve().unwrap().converged);
        assert!(corners(&s).iter().any(|&q| close(q, [7.5, 1.5])));
        s.set_pattern_count(made.by, 5).unwrap();
        assert_eq!(s.pattern_count(made.by), Some(5));
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert_eq!(s.regions().unwrap().len(), 5);
        assert!(corners(&s).iter().any(|&q| close(q, [13.5, 1.5])));
        s.set_pattern_count(made.by, 2).unwrap();
        assert_eq!(s.pattern_count(made.by), Some(2));
        assert_eq!(s.regions().unwrap().len(), 2);
        assert!(s.solve().unwrap().converged);
        s.validate().unwrap();
        s.enclose::<T>().unwrap();
    }

    /// Six holes round a bolt circle: a circle and a slot copied round the
    /// origin, the center its own copy where a line ends on it.
    #[test]
    fn circular_patterns_turn_round_their_center() {
        let mut s = Sketch::<T>::new();
        let o = s.add_fixed_point(n(0.0), n(0.0));
        let c = s.add_point(n(2.0), n(0.0));
        let hole = s.add_circle(c, n(0.25));
        s.constrain(Constraint::Fix {
            point: c,
            x: n(2.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Radius {
            curve: hole,
            value: n(0.25),
        });
        let tip = s.add_point(n(1.0), n(0.5));
        let spoke = s.add_line(o, tip);
        s.constrain(Constraint::Fix {
            point: tip,
            x: n(1.0),
            y: n(0.5),
        });
        let made = s
            .pattern(
                &[hole, spoke],
                &Step::Round {
                    center: o,
                    angle: n(PI / 3.0),
                },
                6,
            )
            .unwrap();
        assert_eq!(s.pattern_count(made.by), Some(6));
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        let centers: Vec<P2> = s
            .curves
            .values()
            .filter_map(|k| match k.kind {
                CurveKind::Circle { center, .. } => Some(xy(&s, center)),
                _ => None,
            })
            .collect();
        assert_eq!(centers.len(), 6);
        for k in 0..6 {
            let a = PI / 3.0 * k as f64;
            assert!(
                centers
                    .iter()
                    .any(|&q| close(q, [2.0 * a.cos(), 2.0 * a.sin()])),
                "{k}: {centers:?}"
            );
        }
        // Every spoke starts at the origin itself.
        let spokes = s
            .curves
            .values()
            .filter(|k| {
                !k.construction && matches!(k.kind, CurveKind::Line { start, .. } if start == o)
            })
            .count();
        assert_eq!(spokes, 6);
        // Eight round the circle: 45° apart first, then two more.
        if let Some(Constraint::Angle { value, .. }) = s.constraints.get_mut(&made.spacing) {
            *value = n(PI / 4.0);
        }
        assert!(s.solve().unwrap().converged);
        s.set_pattern_count(made.by, 8).unwrap();
        assert_eq!(s.pattern_count(made.by), Some(8));
        assert!(s.solve().unwrap().converged);
        s.enclose::<T>().unwrap();
    }

    #[test]
    fn copies_refuse_what_they_cannot_copy_by() {
        let mut s = Sketch::<T>::new();
        let c = s.add_point(n(0.0), n(0.0));
        let circle = s.add_circle(c, n(1.0));
        let e = s.mirror(&[circle], circle).unwrap_err();
        assert!(e.root_message().contains("no line"), "{e}");
        let e = s
            .pattern(
                &[circle],
                &Step::Round {
                    center: c,
                    angle: n(1.0),
                },
                3,
            )
            .unwrap_err();
        assert!(e.root_message().contains("at the center"), "{e}");
        let e = s
            .pattern(
                &[circle],
                &Step::Round {
                    center: c,
                    angle: n(1.0),
                },
                1,
            )
            .unwrap_err();
        assert!(e.root_message().contains("2 or more"), "{e}");
        let d = s.add_point(n(3.0), n(0.0));
        let hole = s.add_circle(d, n(0.5));
        let e = s
            .pattern(
                &[hole],
                &Step::Round {
                    center: c,
                    angle: n(PI / 3.0),
                },
                7,
            )
            .unwrap_err();
        assert!(e.root_message().contains("once"), "{e}");
        let made = s
            .pattern(
                &[hole],
                &Step::Round {
                    center: c,
                    angle: n(PI / 3.0),
                },
                6,
            )
            .unwrap();
        let e = s.set_pattern_count(made.by, 7).unwrap_err();
        assert!(e.root_message().contains("once"), "{e}");
    }
}
