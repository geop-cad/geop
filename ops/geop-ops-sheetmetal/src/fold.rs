//! The solid sheet of a [`Layout`]: [`Sheet::folded`], every face in its
//! place — a flat where it lies, a bend wrapped round its axis — and
//! [`Sheet::unfolded`], every face where the layout has it.
//!
//! A flat's edges are its own curves placed in space, exactly. A bend's
//! edges along its axis are lines, its edges across it arcs, exact too; a
//! cut's curve that runs across a bend any other way is wrapped round it,
//! which no NURBS curve is exactly, and is enclosed: a spline through
//! points of it, widened to take in the rest (see
//! [`NurbCurve3D::enclosing_pad`]).

use std::collections::BTreeMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D, true_point_fractions},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::Sense;
use geop_ops_extrude_revolve::common::{arc3, end_point, line2, line3, start_point};

use crate::{
    layout::{Home, Layout, LayoutEdge, Seg},
    sheet::{BendFrame, Placement, Sheet},
    thicken::{SheetCoedge, SheetEdge, SheetFace, SheetSurface, SheetVertex, translate2},
};

/// How many intervals a wrapped curve is sampled in: bounds how wide its
/// enclosure comes out, not whether it encloses (see
/// [`NurbCurve3D::enclosing_pad`]).
const SAMPLES: usize = 32;

/// A vertex's two points, on the A side and the B side.
type Pair<S> = (Vector3<S>, Vector3<S>);

impl<S: Scalar> Sheet<S> {
    /// The A side of the sheet as it is bent, with its B side.
    pub fn folded(&self) -> GeopResult<SheetSurface<S>> {
        self.surface(&self.layout()?, true)
    }

    /// The A side of the sheet laid flat, with its B side: every face in
    /// the first flat's plane where the layout has it (see
    /// [`crate::layout`]), every bend a flat strip as wide as its
    /// developed length.
    pub fn unfolded(&self) -> GeopResult<SheetSurface<S>> {
        self.surface(&self.layout()?, false)
    }

    /// The faces of `layout` folded into place, or laid flat.
    fn surface(&self, layout: &Layout<S>, folded: bool) -> GeopResult<SheetSurface<S>> {
        let t = S::from_f64(self.rules.thickness);
        let shifts = self.flat_shifts()?;
        let used = layout.used_edges();
        let mut vertex_ids: Vec<usize> = used
            .iter()
            .flat_map(|&e| [layout.edges[e].start, layout.edges[e].end])
            .collect();
        vertex_ids.sort_unstable();
        vertex_ids.dedup();
        let vindex: BTreeMap<usize, usize> = vertex_ids
            .iter()
            .enumerate()
            .map(|(i, &v)| (v, i))
            .collect();
        let eindex: BTreeMap<usize, usize> =
            used.iter().enumerate().map(|(i, &e)| (e, i)).collect();
        let root = &self.flats[0].place;
        let wraps: Vec<Wrap<S>> = if folded {
            (0..self.bends.len())
                .map(|b| Wrap::new(self, b, layout))
                .collect::<GeopResult<_>>()?
        } else {
            Vec::new()
        };

        // A vertex is folded with a flat it lies on, if any; else with its
        // bend.
        let mut home: BTreeMap<usize, Home> = BTreeMap::new();
        for &e in &used {
            let edge = &layout.edges[e];
            for v in [edge.start, edge.end] {
                if !matches!(home.get(&v), Some(Home::Flat(_))) {
                    home.insert(v, edge.home);
                }
            }
        }
        let mut points: BTreeMap<usize, Pair<S>> = BTreeMap::new();
        let mut sheet = SheetSurface::default();
        for &v in &vertex_ids {
            let vertex = &layout.vertices[v];
            let pair = if !folded {
                (root.point(&vertex.at), root.offset(t).point(&vertex.at))
            } else if let Some((f, own)) = &vertex.own {
                let place = &self.flats[*f].place;
                (place.point(own), place.offset(t).point(own))
            } else {
                match home[&v] {
                    Home::Flat(f) => {
                        let own = vertex.at.sub(&shifts[f]);
                        let place = &self.flats[f].place;
                        (place.point(&own), place.offset(t).point(&own))
                    }
                    Home::Bend(b) => wraps[b].pair(&vertex.at)?,
                }
            };
            points.insert(v, pair);
            sheet.vertices.push(SheetVertex {
                a: pair.0,
                b: pair.1,
                name: vertex.name.clone().ok_or_else(|| {
                    GeopError::new(format!(
                        "the layout's vertex at {:?} has no name",
                        vertex.at
                    ))
                })?,
            });
        }

        // An edge in its flat's own coordinates.
        let own = |edge: &LayoutEdge<S>, f: usize| -> GeopResult<NurbCurve2D<S>> {
            match &edge.own {
                Some(curve) => Ok(curve.clone()),
                None => translate2(&edge.curve, &shifts[f].neg()),
            }
        };
        for &e in &used {
            let edge = &layout.edges[e];
            let ctx = with_context!("edge {}", edge.name.root());
            let (a, b) = if !folded {
                (root.curve(&edge.curve)?, root.offset(t).curve(&edge.curve)?)
            } else {
                match edge.home {
                    Home::Flat(f) => {
                        let curve = own(edge, f).with_context(ctx)?;
                        let place = &self.flats[f].place;
                        (place.curve(&curve)?, place.offset(t).curve(&curve)?)
                    }
                    Home::Bend(b) => wraps[b]
                        .edge(edge, &points[&edge.start], &points[&edge.end])
                        .with_context(ctx)?,
                }
            };
            sheet.edges.push(SheetEdge {
                a,
                b,
                start: vindex[&edge.start],
                end: vindex[&edge.end],
                name: edge.name.clone(),
            });
        }

        for face in &layout.faces {
            let ctx = with_context!("face {}", face.name.root());
            let surfaces = match (folded, face.home) {
                (true, Home::Bend(b)) => {
                    let wrap = &wraps[b];
                    let uv = wrap.vertex_uv(face, layout).with_context(ctx)?;
                    let coedges = |lp: &Vec<(usize, Sense)>| -> GeopResult<Vec<SheetCoedge<S>>> {
                        lp.iter()
                            .map(|&(e, sense)| {
                                Ok(SheetCoedge {
                                    edge: eindex[&e],
                                    sense,
                                    pcurve: wrap.pcurve(&layout.edges[e], sense, &uv)?,
                                })
                            })
                            .collect()
                    };
                    let outer = coedges(&face.outer).with_context(ctx)?;
                    let holes = face
                        .holes
                        .iter()
                        .map(coedges)
                        .collect::<GeopResult<_>>()
                        .with_context(ctx)?;
                    sheet.faces.push(SheetFace {
                        a: wrap.strips.0.clone(),
                        b: wrap.strips.1.clone(),
                        outer,
                        holes,
                        name: face.name.clone(),
                    });
                    continue;
                }
                (true, Home::Flat(f)) => Some(f),
                (false, _) => None,
            };
            // A plane, parametrized by the coordinates its curves are in.
            let place: &Placement<S> = match surfaces {
                Some(f) => &self.flats[f].place,
                None => root,
            };
            let curve = |e: usize| -> GeopResult<NurbCurve2D<S>> {
                match surfaces {
                    Some(f) => own(&layout.edges[e], f),
                    None => Ok(layout.edges[e].curve.clone()),
                }
            };
            let coedges = |lp: &Vec<(usize, Sense)>| -> GeopResult<Vec<SheetCoedge<S>>> {
                lp.iter()
                    .map(|&(e, sense)| {
                        let c = curve(e)?;
                        Ok(SheetCoedge {
                            edge: eindex[&e],
                            sense,
                            pcurve: match sense {
                                Sense::Forward => c,
                                Sense::Reversed => c.reverse(),
                            },
                        })
                    })
                    .collect()
            };
            let outer = coedges(&face.outer).with_context(ctx)?;
            let holes: Vec<Vec<SheetCoedge<S>>> = face
                .holes
                .iter()
                .map(coedges)
                .collect::<GeopResult<_>>()
                .with_context(ctx)?;
            let refs: Vec<&NurbCurve2D<S>> = std::iter::once(&outer)
                .chain(&holes)
                .flatten()
                .map(|c| &c.pcurve)
                .collect();
            sheet.faces.push(SheetFace {
                a: place.plane(&refs).with_context(ctx)?,
                b: place.offset(t).plane(&refs).with_context(ctx)?,
                outer,
                holes,
                name: face.name.clone(),
            });
        }
        Ok(sheet)
    }
}

/// A bend's place in the layout and in space: how a point of its strip,
/// laid flat, is wrapped round its axis.
///
/// A point of the strip lies `along` its parent edge from where that
/// starts and `across` it, out of the parent, on the neutral surface: so at
/// the angle `θ = across / (radius + k_factor * thickness)` round the axis.
/// On the bend's faces — rational quadratic arcs from the parent across to
/// the child, swept along the axis — that is `v = along / length` and the
/// `u` where the arc's parameter reaches `θ`: for an arc turning `α`,
/// `tan((θ - α/2) / 2) = (2u - 1) tan(α/4)`.
struct Wrap<S: Scalar> {
    frame: BendFrame<S>,
    /// Where its parent edge starts and runs, and out of the parent, in the
    /// layout.
    origin: Vector2<S>,
    along: Vector2<S>,
    out: Vector2<S>,
    length: S,
    neutral: S,
    /// How many rational quadratic spans its arcs are built of (see
    /// [`spans`]), and the angle each turns.
    spans: usize,
    step: S,
    /// `tan(step / 4)`.
    quarter_tan: S,
    /// Its faces, A side and B side.
    strips: (NurbSurface3D<S>, NurbSurface3D<S>),
    /// Its edges across, where its parent edge starts and ends, on both
    /// sides — as built, before any cut.
    sides: [(NurbCurve3D<S>, NurbCurve3D<S>); 2],
    /// The keys of its parent edge, its child edge and its sides.
    keys: [String; 4],
}

impl<S: Scalar> Wrap<S> {
    fn new(sheet: &Sheet<S>, b: usize, layout: &Layout<S>) -> GeopResult<Self> {
        let bend = &sheet.bends[b];
        let ctx = with_context!("bend {}", bend.name.root());
        let t = S::from_f64(sheet.rules.thickness);
        let frame = sheet.bend_frame(bend).with_context(ctx)?;
        let [pa, pb, ca, cb] = layout.corners[b];
        let corner = |v: usize| -> GeopResult<Pair<S>> {
            let (f, own) = layout.vertices[v].own.as_ref().ok_or_else(|| {
                GeopError::new(format!(
                    "bend {}'s corner is no vertex of the sheet as built",
                    bend.name.root()
                ))
            })?;
            let place = &sheet.flats[*f].place;
            Ok((place.point(own), place.offset(t).point(own)))
        };
        let (s0, s1) = {
            let turn = |p: Pair<S>, c: Pair<S>| -> GeopResult<(NurbCurve3D<S>, NurbCurve3D<S>)> {
                Ok((
                    turn(&frame, p.0, c.0, S::ZERO, bend.angle, frame.r_a)?,
                    turn(&frame, p.1, c.1, S::ZERO, bend.angle, frame.r_b)?,
                ))
            };
            (
                turn(corner(pa)?, corner(ca)?).with_context(ctx)?,
                turn(corner(pb)?, corner(cb)?).with_context(ctx)?,
            )
        };
        // `u` from the parent across to the child, `v` along the parent's
        // edge: `∂u × ∂v = m × tau = n`.
        let strip = |a: &NurbCurve3D<S>, b: &NurbCurve3D<S>| {
            NurbSurface::try_new(
                2,
                1,
                a.control_points
                    .iter()
                    .zip(&b.control_points)
                    .flat_map(|(p, q)| [*p, *q])
                    .collect(),
                a.knot_vector.clone(),
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )
        };
        let strips = (strip(&s0.0, &s1.0)?, strip(&s0.1, &s1.1)?);
        let origin = layout.vertices[pa].at;
        let run = layout.vertices[pb].at.sub(&origin);
        let along = run.normalize().with_context(ctx)?;
        let neutral = bend.radius.add(S::from_f64(sheet.rules.k_factor).mul(t));
        let spans = spans(bend.angle);
        let step = match spans {
            1 => bend.angle,
            n => bend.angle.div(S::from_i64(n as i64))?,
        };
        let quarter = step.div(S::from_f64(4.0))?;
        Ok(Self {
            origin,
            out: Vector2::from_array([along[1], along[0].neg()]),
            along,
            length: run.norm(),
            neutral,
            spans,
            step,
            quarter_tan: quarter.sin().div(quarter.cos())?,
            strips,
            sides: [s0, s1],
            keys: [
                bend.parent_edge.clone(),
                bend.child_edge.clone(),
                bend.name.scoped("s0").root(),
                bend.name.scoped("s1").root(),
            ],
            frame,
        })
    }

    /// The layout point `x` as how far along the parent edge it is, and
    /// at what angle round the axis.
    fn coords(&self, x: &Vector2<S>) -> GeopResult<(S, S)> {
        let d = x.sub(&self.origin);
        Ok((
            d.prod_dot(&self.along),
            d.prod_dot(&self.out).div(self.neutral)?,
        ))
    }

    /// The point `along` the axis, at angle `theta` round it, `radius`
    /// from it.
    fn point(&self, along: S, theta: S, radius: S) -> Vector3<S> {
        let f = &self.frame;
        let radial =
            f.m.prod_scalar(theta.sin())
                .sub(&f.n.prod_scalar(f.sigma.mul(theta.cos())));
        f.center
            .add(&f.tau.prod_scalar(along))
            .add(&radial.prod_scalar(radius))
    }

    /// The layout point `x` wrapped onto the A side and the B side.
    fn pair(&self, x: &Vector2<S>) -> GeopResult<Pair<S>> {
        let (along, theta) = self.coords(x)?;
        Ok((
            self.point(along, theta, self.frame.r_a),
            self.point(along, theta, self.frame.r_b),
        ))
    }

    /// The parameters on the bend's faces of a point `along` the axis at
    /// angle `theta`: in the span the angle falls in — at a joint between
    /// two, either gives the same point, so which is a free choice.
    fn uv(&self, along: S, theta: S) -> GeopResult<Vector2<S>> {
        let span =
            ((theta.to_f64() / self.step.to_f64()).floor().max(0.0) as usize).min(self.spans - 1);
        let local = match span {
            0 => theta,
            j => theta.sub(self.step.mul(S::from_i64(j as i64))),
        };
        let half = local.sub(self.step.div(S::TWO)?).div(S::TWO)?;
        let tan = half.sin().div(half.cos())?;
        let t = S::ONE.add(tan.div(self.quarter_tan)?).div(S::TWO)?;
        let u = match self.spans {
            1 => t,
            n => t.add(S::from_i64(span as i64)).div(S::from_i64(n as i64))?,
        };
        Ok(Vector2::from_array([u, along.div(self.length)?]))
    }

    /// The parameters on the bend's faces of every vertex of `face`: `u`
    /// exactly 0 on its parent edge and 1 on its child edge, `v` 0 and 1 on
    /// its sides, the rest where the point wraps to.
    fn vertex_uv(
        &self,
        face: &crate::layout::LayoutFace,
        layout: &Layout<S>,
    ) -> GeopResult<BTreeMap<usize, Vector2<S>>> {
        let mut fixed: BTreeMap<usize, [Option<S>; 2]> = BTreeMap::new();
        for &(e, _) in face.loops().flatten() {
            let edge = &layout.edges[e];
            let set = match edge.origin.as_deref() {
                Some(k) if k == self.keys[0] => Some((0, S::ZERO)),
                Some(k) if k == self.keys[1] => Some((0, S::ONE)),
                Some(k) if k == self.keys[2] => Some((1, S::ZERO)),
                Some(k) if k == self.keys[3] => Some((1, S::ONE)),
                _ => None,
            };
            for v in [edge.start, edge.end] {
                let slot = fixed.entry(v).or_default();
                if let Some((k, value)) = set {
                    slot[k] = Some(value);
                }
            }
        }
        fixed
            .into_iter()
            .map(|(v, [u, w])| {
                let (along, theta) = self.coords(&layout.vertices[v].at)?;
                let free = self.uv(along, theta)?;
                Ok((
                    v,
                    Vector2::from_array([u.unwrap_or(free[0]), w.unwrap_or(free[1])]),
                ))
            })
            .collect()
    }

    /// How a curve of the strip runs: along the axis, across it, or
    /// neither.
    fn kind(&self, edge: &LayoutEdge<S>) -> GeopResult<Run> {
        if let Some(origin) = &edge.origin {
            return Ok(if *origin == self.keys[2] || *origin == self.keys[3] {
                Run::Across
            } else {
                Run::Along
            });
        }
        Ok(match Seg::of(&edge.curve)? {
            Seg::Line { a, b } => {
                let d = b.sub(&a);
                if d.prod_cross(&self.along).could_be_equal(S::ZERO) {
                    Run::Along
                } else if d.prod_dot(&self.along).could_be_equal(S::ZERO) {
                    Run::Across
                } else {
                    Run::Wrapped
                }
            }
            _ => Run::Wrapped,
        })
    }

    /// The edge `edge` of the strip, from `start` to `end`, on both sides.
    fn edge(
        &self,
        edge: &LayoutEdge<S>,
        start: &Pair<S>,
        end: &Pair<S>,
    ) -> GeopResult<(NurbCurve3D<S>, NurbCurve3D<S>)> {
        // A side as built, uncut.
        for (k, side) in self.sides.iter().enumerate() {
            let key = &self.keys[2 + k];
            if edge.origin.as_ref() == Some(key) && edge.name.root() == *key {
                return Ok(side.clone());
            }
        }
        match self.kind(edge)? {
            Run::Along => Ok((line3(start.0, end.0)?, line3(start.1, end.1)?)),
            Run::Across => {
                let (_, from) = self.coords(&start_point(&edge.curve)?)?;
                let (_, to) = self.coords(&end_point(&edge.curve)?)?;
                Ok((
                    turn(&self.frame, start.0, end.0, from, to, self.frame.r_a)?,
                    turn(&self.frame, start.1, end.1, from, to, self.frame.r_b)?,
                ))
            }
            Run::Wrapped => self.wrapped(&edge.curve, start, end),
        }
    }

    /// The layout curve `curve` wrapped onto both sides, from `start` to
    /// `end`: splines through points of it at the same parameters — alike,
    /// so that the wall between them is ruled — each widened to enclose it.
    fn wrapped(
        &self,
        curve: &NurbCurve2D<S>,
        start: &Pair<S>,
        end: &Pair<S>,
    ) -> GeopResult<(NurbCurve3D<S>, NurbCurve3D<S>)> {
        let (params, samples, between) = sample(curve)?;
        let mut sides = Vec::new();
        for (k, radius) in [self.frame.r_a, self.frame.r_b].into_iter().enumerate() {
            let at = |x: &Vector2<S>| -> GeopResult<Vector3<S>> {
                let (along, theta) = self.coords(x)?;
                Ok(self.point(along, theta, radius))
            };
            let mut points = samples.iter().map(at).collect::<GeopResult<Vec<_>>>()?;
            let last = points.len() - 1;
            points[0] = if k == 0 { start.0 } else { start.1 };
            points[last] = if k == 0 { end.0 } else { end.1 };
            // Through the points' centres — which points it passes through
            // is a free choice — and widened to enclose them whole.
            let values: Vec<Vector4<S>> = points
                .iter()
                .map(|p| {
                    let c = p.sharpen();
                    Vector4::from_array([c[0], c[1], c[2], S::ONE])
                })
                .collect();
            let mut fit = NurbCurve::interpolate_homogeneous(&values, &params, 3)?;
            let mut checks: Vec<(S, Vector3<S>)> =
                params.iter().copied().zip(points.iter().copied()).collect();
            for (param, x) in between.iter().flatten() {
                checks.push((*param, at(x)?));
            }
            let pad = fit.enclosing_pad(&checks)?;
            fit.widen(&pad);
            sides.push(fit);
        }
        let b = sides.pop().expect("two sides");
        let a = sides.pop().expect("two sides");
        Ok((a, b))
    }

    /// The pcurve on the bend's faces of `edge`, run in `sense`, its ends
    /// at the parameters `uv` gives its vertices.
    fn pcurve(
        &self,
        edge: &LayoutEdge<S>,
        sense: Sense,
        uv: &BTreeMap<usize, Vector2<S>>,
    ) -> GeopResult<NurbCurve2D<S>> {
        let (from, to) = match sense {
            Sense::Forward => (edge.start, edge.end),
            Sense::Reversed => (edge.end, edge.start),
        };
        if !matches!(self.kind(edge)?, Run::Wrapped) {
            return line2(uv[&from], uv[&to]);
        }
        let curve = match sense {
            Sense::Forward => edge.curve.clone(),
            Sense::Reversed => edge.curve.reverse(),
        };
        let (_, samples, between) = sample(&curve)?;
        let at = |x: &Vector2<S>| -> GeopResult<Vector2<S>> {
            let (along, theta) = self.coords(x)?;
            self.uv(along, theta)
        };
        let mut points = samples.iter().map(at).collect::<GeopResult<Vec<_>>>()?;
        let last = points.len() - 1;
        points[0] = uv[&from];
        points[last] = uv[&to];
        let inside = between
            .iter()
            .map(|chunk| chunk.iter().map(|(_, x)| at(x)).collect())
            .collect::<GeopResult<Vec<Vec<_>>>>()?;
        NurbCurve2D::interpolate_enclosing(&points, &inside, 3)
    }
}

/// How many rational quadratic spans an arc turning `angle` is built of:
/// one up to 120°, its middle weight `cos(angle / 2)` then at least a half,
/// two beyond — up to the half turn of a hem, which one span cannot take.
fn spans<S: Scalar>(angle: S) -> usize {
    if angle.abs().to_f64() <= 2.0 * std::f64::consts::PI / 3.0 {
        1
    } else {
        2
    }
}

/// The arc round the axis of the bend `frame` from `p`, at angle `from`,
/// to `q`, at angle `to`, `radius` from the axis: of [`spans`] rational
/// quadratic spans, each through the point where its end tangents meet.
fn turn<S: Scalar>(
    frame: &BendFrame<S>,
    p: Vector3<S>,
    q: Vector3<S>,
    from: S,
    to: S,
    radius: S,
) -> GeopResult<NurbCurve3D<S>> {
    let starts_flat = from.is_sharp() && from.could_be_equal(S::ZERO);
    let delta = if starts_flat { to } else { to.sub(from) };
    let n = spans(delta);
    let step = match n {
        1 => delta,
        n => delta.div(S::from_i64(n as i64))?,
    };
    let half = step.div(S::TWO)?;
    let (tan, weight) = (half.sin().div(half.cos())?, half.cos());
    let direction = |theta: S, flat: bool| {
        // Along the arc, and out from the axis, at `theta`.
        if flat {
            (frame.m, frame.n.prod_scalar(frame.sigma.neg()))
        } else {
            let (c, s) = (theta.cos(), theta.sin());
            (
                frame
                    .m
                    .prod_scalar(c)
                    .add(&frame.n.prod_scalar(frame.sigma.mul(s))),
                frame
                    .m
                    .prod_scalar(s)
                    .sub(&frame.n.prod_scalar(frame.sigma.mul(c))),
            )
        }
    };
    if n == 1 {
        let (tangent, _) = direction(from, starts_flat);
        return arc3(p, p.add(&tangent.prod_scalar(radius.mul(tan))), q, weight);
    }
    let homogeneous =
        |x: &Vector3<S>, w: S| Vector4::from_array([x[0].mul(w), x[1].mul(w), x[2].mul(w), w]);
    let mut control_points = vec![homogeneous(&p, S::ONE)];
    let mut knots = vec![S::ZERO; 3];
    let mut at = p;
    for j in 0..n {
        let theta = if j == 0 {
            from
        } else {
            from.add(step.mul(S::from_i64(j as i64)))
        };
        let (tangent, radial) = direction(theta, j == 0 && starts_flat);
        let apex = at.add(&tangent.prod_scalar(radius.mul(tan)));
        let next = if j + 1 == n {
            q
        } else {
            let (_, out) = direction(from.add(step.mul(S::from_i64(j as i64 + 1))), false);
            at.add(&out.sub(&radial).prod_scalar(radius))
        };
        control_points.push(homogeneous(&apex, weight));
        control_points.push(homogeneous(&next, S::ONE));
        if j + 1 < n {
            let knot = S::from_f64((j + 1) as f64 / n as f64);
            knots.extend([knot, knot]);
        }
        at = next;
    }
    knots.extend([S::ONE; 3]);
    NurbCurve::try_new(2, control_points, knots)
}

/// How a curve of a bend's strip runs.
enum Run {
    /// Along the axis: a line wrapped.
    Along,
    /// Across it: an arc wrapped.
    Across,
    /// Any other way: no NURBS curve wrapped.
    Wrapped,
}

/// Points of `curve` to fit a wrapped copy through: its parameters spread
/// evenly from 0 to 1, the points there, and per interval between them
/// points inside it, at their own parameters (see
/// [`true_point_fractions`]).
#[allow(clippy::type_complexity)]
fn sample<S: Scalar>(
    curve: &NurbCurve2D<S>,
) -> GeopResult<(Vec<S>, Vec<Vector2<S>>, Vec<Vec<(S, Vector2<S>)>>)> {
    let (t0, t1) = curve.domain();
    // Where to sample is a free choice, so sharp.
    let at =
        |f: S| -> GeopResult<Vector2<S>> { curve.evaluate(t0.add(t1.sub(t0).mul(f)).sharpen()) };
    let mut params = Vec::new();
    let mut samples = Vec::new();
    let mut between = Vec::new();
    for i in 0..=SAMPLES {
        // Sharp: the parameters a fit passes through are a free choice.
        let f = S::from_f64(i as f64 / SAMPLES as f64);
        params.push(f);
        samples.push(at(f)?);
        if i < SAMPLES {
            let mut inside = Vec::new();
            for &(num, den) in true_point_fractions(i, SAMPLES) {
                let g = S::from_f64((i as i64 * den + num) as f64 / (SAMPLES as i64 * den) as f64);
                inside.push((g, at(g)?));
            }
            between.push(inside);
        }
    }
    Ok((params, samples, between))
}
