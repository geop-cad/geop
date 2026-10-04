//! The sheet-metal model a body is built from and unfolded with: its
//! [`SheetMetalRules`], and its [`Sheet`] — flat faces joined by bends —
//! recorded on the solid it builds (see [`geop_ops::Part::body_data`]).
//!
//! **Sheet coordinates.** Every flat is drawn in one plane of coordinates
//! `(x, y)`, the *sheet coordinates*, and placed in space by a
//! [`Placement`] of its own. A bend joins an edge `P` of its parent flat to
//! an edge `C` of its child flat, and the child's placement is the
//! parent's turned about the bend's axis: so `C` *is* `P` in sheet
//! coordinates, and the child lies on the far side of it. Laid flat, every
//! flat keeps its shape and only moves away from its parent by the bend's
//! developed length (see [`Sheet::unfolded`]), which is what makes the flat
//! pattern exact and the unfolding topological: which faces are flat and
//! which are bends is recorded here, never guessed from the geometry.
//!
//! **Sides.** The *A side* is the side drawn — the sketch's plane or line —
//! and the *B side* a thickness further along each flat's normal `n`. A
//! bend turns towards the B side (`toward_b`), which is then its inside,
//! or away from it.

use std::collections::BTreeMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::Sense;
use geop_ops::Namer;
use geop_ops_extrude_revolve::common::{
    arc3, embed_curve, end_point, line2, line3, start_point,
};
use serde::{Deserialize, Serialize};

use crate::thicken::{SheetCoedge, SheetEdge, SheetFace, SheetSurface, SheetVertex, translate2};

/// How a body's sheet metal is made and bent: what the base flange sets
/// and every flange and the flat pattern of the body follow.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SheetMetalRules {
    /// How thick the sheet is.
    pub thickness: f64,
    /// The inner radius of a bend, unless a flange gives its own.
    pub bend_radius: f64,
    /// Where the neutral surface lies across the thickness, from the inside
    /// of a bend (`0`) to its outside (`1`): what a bend's developed length
    /// is measured on, `angle * (radius + k_factor * thickness)`.
    pub k_factor: f64,
    /// What is cut beside a flange narrower than its edge.
    #[serde(default)]
    pub relief: Relief,
    /// A relief's width, and how far it reaches past the bend, in
    /// thicknesses.
    pub relief_ratio: f64,
    /// The gap a flange keeps from a bend that already meets its edge at a
    /// corner, so that the two stay apart: an open corner.
    pub corner_gap: f64,
}

impl Default for SheetMetalRules {
    fn default() -> Self {
        Self {
            thickness: 0.1,
            bend_radius: 0.1,
            k_factor: 0.44,
            relief: Relief::Rectangular,
            relief_ratio: 0.5,
            corner_gap: 0.1,
        }
    }
}

impl SheetMetalRules {
    /// Checks that the rules describe sheet metal that can be built.
    pub fn check(&self) -> GeopResult<()> {
        let positive = [
            ("thickness", self.thickness),
            ("bend radius", self.bend_radius),
            ("relief ratio", self.relief_ratio),
        ];
        for (what, value) in positive {
            if !(value.is_finite() && value > 0.0) {
                return Err(GeopError::new(format!(
                    "the sheet's {what} must be positive, not {value}"
                )));
            }
        }
        if !(0.0..=1.0).contains(&self.k_factor) {
            return Err(GeopError::new(format!(
                "the K-factor must lie between 0 and 1, not {}",
                self.k_factor
            )));
        }
        if !(self.corner_gap.is_finite() && self.corner_gap >= 0.0) {
            return Err(GeopError::new(format!(
                "the corner gap must not be negative, not {}",
                self.corner_gap
            )));
        }
        Ok(())
    }

    /// How wide a relief is, and how far past the bend it reaches; none
    /// for a tear.
    pub fn relief_size(&self) -> Option<f64> {
        match self.relief {
            Relief::Rectangular => Some(self.relief_ratio * self.thickness),
            Relief::Tear => None,
        }
    }
}

/// What is cut beside a flange narrower than its edge, where the bend would
/// otherwise tear the sheet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Relief {
    /// A rectangular notch on either side of the bend.
    #[default]
    Rectangular,
    /// Nothing: the sheet is cut straight along the bend's ends.
    Tear,
}

/// Where a flat lies: sheet point `(x, y)` at `origin + x e1 + y e2`, its
/// unit normal `n = e1 × e2` pointing from its A side to its B side.
/// `e1`, `e2` and `n` are orthonormal.
#[derive(Clone, Debug)]
pub struct Placement<S: Scalar> {
    pub origin: Vector3<S>,
    pub e1: Vector3<S>,
    pub e2: Vector3<S>,
    pub n: Vector3<S>,
}

impl<S: Scalar> Placement<S> {
    /// The sheet point `p` in space.
    pub fn point(&self, p: &Vector2<S>) -> Vector3<S> {
        self.origin
            .add(&self.e1.prod_scalar(p[0]))
            .add(&self.e2.prod_scalar(p[1]))
    }

    /// The sheet direction `d` in space.
    pub fn direction(&self, d: &Vector2<S>) -> Vector3<S> {
        self.e1.prod_scalar(d[0]).add(&self.e2.prod_scalar(d[1]))
    }

    /// The sheet curve `curve` in space: exact, an affine map of its
    /// control points.
    pub fn curve(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve3D<S>> {
        embed_curve(curve, &self.origin, &self.e1, &self.e2)
    }

    /// The same placement `t` further along the normal: the B side's.
    pub fn offset(&self, t: S) -> Self {
        Self {
            origin: self.origin.add(&self.n.prod_scalar(t)),
            ..self.clone()
        }
    }

    /// The same placement moved by the sheet vector `shift`.
    pub fn shifted(&self, shift: &Vector2<S>) -> Self {
        Self {
            origin: self.point(shift),
            ..self.clone()
        }
    }

    /// As a coordinate system: `u = e1`, `v = e2`, `w = n`.
    pub fn coordinate_system(&self) -> GeopResult<CoordinateSystem<S>> {
        CoordinateSystem::try_new(self.origin, self.e1, self.e2, self.n)
    }

    /// The plane through the placement, as a bilinear patch parametrized by
    /// sheet coordinates over the box around `curves`: so a curve drawn in
    /// sheet coordinates is its own pcurve on it.
    fn plane(&self, curves: &[&NurbCurve2D<S>]) -> GeopResult<NurbSurface3D<S>> {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for curve in curves {
            for cp in &curve.control_points {
                for k in 0..2 {
                    let x = cp[k].div(cp[2])?;
                    lo[k] = lo[k].min(x.lower().to_f64());
                    hi[k] = hi[k].max(x.upper().to_f64());
                }
            }
        }
        // The box's corners are a free choice — any box around the curves
        // will do — so sharp: and it encloses them, a NURBS curve lying in
        // the hull of its control points.
        let (lo, hi) = (lo.map(S::from_f64), hi.map(S::from_f64));
        let corner = |x: S, y: S| {
            let p = self.point(&Vector2::from_array([x, y]));
            Vector4::from_array([p[0], p[1], p[2], S::ONE])
        };
        NurbSurface::try_new(
            1,
            1,
            vec![
                corner(lo[0], lo[1]),
                corner(lo[0], hi[1]),
                corner(hi[0], lo[1]),
                corner(hi[0], hi[1]),
            ],
            vec![lo[0], lo[0], hi[0], hi[0]],
            vec![lo[1], lo[1], hi[1], hi[1]],
        )
    }
}

/// One edge of a flat's loop, in sheet coordinates, running the way the
/// loop does. The vertex where it starts is named after it.
#[derive(Clone, Debug)]
pub struct FlatEdge<S: Scalar> {
    pub curve: NurbCurve2D<S>,
    /// The edge's names: `N(a)`, `N(b)`, its wall `N`, the vertices it
    /// starts at `N(v,a)`, `N(v,b)` and the edge between them `N(v)`.
    pub name: Namer,
}

impl<S: Scalar> FlatEdge<S> {
    /// What a bend calls it by: its wall's name.
    pub fn key(&self) -> String {
        self.name.root()
    }

    /// Its ends, in sheet coordinates, if it is straight.
    pub fn line(&self) -> Option<GeopResult<[Vector2<S>; 2]>> {
        (self.curve.degree == 1 && self.curve.control_points.len() == 2).then(|| {
            Ok([start_point(&self.curve)?, end_point(&self.curve)?])
        })
    }
}

/// A flat face of the sheet: its loops in sheet coordinates — the outer
/// one counter-clockwise about its normal, holes clockwise — and where it
/// lies.
#[derive(Clone, Debug)]
pub struct Flat<S: Scalar> {
    pub place: Placement<S>,
    pub outer: Vec<FlatEdge<S>>,
    pub holes: Vec<Vec<FlatEdge<S>>>,
    /// Its faces are `N(a)` and `N(b)`.
    pub name: Namer,
}

/// A bend: a cylindrical strip from the straight edge `parent_edge` of
/// flat `parent` to the edge `child_edge` of flat `child` — the same
/// segment in sheet coordinates, run the other way.
#[derive(Clone, Debug)]
pub struct Bend<S: Scalar> {
    pub parent: usize,
    pub parent_edge: String,
    pub child: usize,
    pub child_edge: String,
    /// How far it turns, in radians: in `(0, π)`.
    pub angle: S,
    /// Its inner radius.
    pub radius: S,
    /// Whether it turns towards the parent's B side.
    pub toward_b: bool,
    /// Its faces are `N(a)` and `N(b)`; its edges across from parent to
    /// child, where `parent_edge` starts and where it ends, `N(s0)` and
    /// `N(s1)`, named like a [`FlatEdge`]'s.
    pub name: Namer,
}

/// A sheet-metal body as it is built: flat faces, the first the one the
/// rest hang off, and the bends joining them into a tree, each child built
/// after its parent.
#[derive(Clone, Debug)]
pub struct Sheet<S: Scalar> {
    pub rules: SheetMetalRules,
    pub flats: Vec<Flat<S>>,
    pub bends: Vec<Bend<S>>,
}

/// How a bend turns, worked out from its parent flat: the frame of its
/// first edge and its radii.
pub struct BendFrame<S: Scalar> {
    /// Along the parent's edge, in the direction its loop runs it.
    pub tau: Vector3<S>,
    /// Out of the parent, across the edge, in its plane.
    pub m: Vector3<S>,
    /// The parent's normal.
    pub n: Vector3<S>,
    /// `1` turning towards the B side, `-1` away from it.
    pub sigma: S,
    /// The radius of its A side and of its B side.
    pub r_a: S,
    pub r_b: S,
    pub cos: S,
    pub sin: S,
    /// `tan(angle / 2)`: how far the virtual sharp lies from where the bend
    /// starts, per unit of radius.
    pub half_tan: S,
    /// `cos(angle / 2)`: the weight of the arcs' middle control points.
    pub half_cos: S,
    /// A point on the axis.
    pub center: Vector3<S>,
}

impl<S: Scalar> BendFrame<S> {
    /// A bend from the straight parent edge `a -> b` of a flat placed at
    /// `place`, turning by `angle` with inner radius `radius` in a sheet
    /// `thickness` thick.
    pub fn new(
        place: &Placement<S>,
        [a, b]: [Vector2<S>; 2],
        angle: S,
        radius: S,
        thickness: S,
        toward_b: bool,
    ) -> GeopResult<Self> {
        let tau = place.direction(&b.sub(&a)).normalize()?;
        let n = place.n;
        let m = tau.prod_cross(&n);
        let (sigma, r_a, r_b) = if toward_b {
            (S::ONE, radius.add(thickness), radius)
        } else {
            (S::ONE.neg(), radius, radius.add(thickness))
        };
        let half = angle.div(S::TWO)?;
        let half_cos = half.cos();
        Ok(Self {
            center: place.point(&a).add(&n.prod_scalar(sigma.mul(r_a))),
            tau,
            m,
            n,
            sigma,
            r_a,
            r_b,
            cos: angle.cos(),
            sin: angle.sin(),
            half_tan: half.sin().div(half_cos)?,
            half_cos,
        })
    }

    /// The vector `x` turned by the bend: `m` towards the side it turns
    /// to, about its axis.
    pub fn rotate(&self, x: &Vector3<S>) -> Vector3<S> {
        let (along_m, along_tau, along_n) = (x.prod_dot(&self.m), x.prod_dot(&self.tau), x.prod_dot(&self.n));
        let sigma_sin = self.sigma.mul(self.sin);
        let m = self
            .m
            .prod_scalar(self.cos)
            .add(&self.n.prod_scalar(sigma_sin));
        let n = self
            .n
            .prod_scalar(self.cos)
            .sub(&self.m.prod_scalar(sigma_sin));
        m.prod_scalar(along_m)
            .add(&self.tau.prod_scalar(along_tau))
            .add(&n.prod_scalar(along_n))
    }

    /// Where the child flat lies: the parent's placement turned about the
    /// bend's axis. Its edge where the bend ends is the parent's where it
    /// starts, in sheet coordinates.
    pub fn child_placement(&self, parent: &Placement<S>) -> Placement<S> {
        Placement {
            origin: self
                .center
                .add(&self.rotate(&parent.origin.sub(&self.center))),
            e1: self.rotate(&parent.e1),
            e2: self.rotate(&parent.e2),
            n: self.rotate(&parent.n),
        }
    }
}

impl<S: Scalar> Sheet<S> {
    /// The flat and position in its outer loop of the edge `key`.
    pub fn outer_edge(&self, key: &str) -> Option<(usize, usize)> {
        self.flats.iter().enumerate().find_map(|(f, flat)| {
            flat.outer
                .iter()
                .position(|e| e.key() == key)
                .map(|k| (f, k))
        })
    }

    /// Whether a bend starts or ends at the edge `key`.
    pub fn is_bent(&self, key: &str) -> bool {
        self.bends
            .iter()
            .any(|b| b.parent_edge == key || b.child_edge == key)
    }

    /// The flat edge — `(flat, position in its outer loop)` — whose A side
    /// (`false`) or B side (`true`) edge of the body is named `name`.
    pub fn edge_named(&self, name: &str) -> Option<(usize, usize, bool)> {
        self.flats.iter().enumerate().find_map(|(f, flat)| {
            flat.outer.iter().enumerate().find_map(|(k, e)| {
                if e.name.name(&["a"]) == name {
                    Some((f, k, false))
                } else if e.name.name(&["b"]) == name {
                    Some((f, k, true))
                } else {
                    None
                }
            })
        })
    }

    /// Where bend `bend` starts: its parent edge's ends.
    fn parent_line(&self, bend: &Bend<S>) -> GeopResult<[Vector2<S>; 2]> {
        let (f, k) = self.outer_edge(&bend.parent_edge).ok_or_else(|| {
            GeopError::new(format!(
                "bend {} starts at {}, which is no edge of a flat's outline",
                bend.name.root(),
                bend.parent_edge
            ))
        })?;
        self.flats[f].outer[k].line().ok_or_else(|| {
            GeopError::new(format!(
                "bend {} starts at {}, which is not straight",
                bend.name.root(),
                bend.parent_edge
            ))
        })?
    }

    /// How bend `bend` turns.
    pub fn bend_frame(&self, bend: &Bend<S>) -> GeopResult<BendFrame<S>> {
        BendFrame::new(
            &self.flats[bend.parent].place,
            self.parent_line(bend)?,
            bend.angle,
            bend.radius,
            S::from_f64(self.rules.thickness),
            bend.toward_b,
        )
    }

    /// Bend `bend`'s developed length: its angle times the radius of the
    /// neutral surface, `radius + k_factor * thickness`.
    pub fn developed_length(&self, bend: &Bend<S>) -> S {
        let neutral = bend.radius.add(
            S::from_f64(self.rules.k_factor).mul(S::from_f64(self.rules.thickness)),
        );
        bend.angle.mul(neutral)
    }

    /// Where every flat lies in the flat pattern, in sheet coordinates:
    /// the first where it is, each child moved away from its parent, out
    /// across the bend's first edge, by the bend's developed length.
    pub fn flat_shifts(&self) -> GeopResult<Vec<Vector2<S>>> {
        let mut shifts: Vec<Option<Vector2<S>>> = vec![None; self.flats.len()];
        shifts[0] = Some(Vector2::zero());
        for bend in &self.bends {
            let [a, b] = self.parent_line(bend)?;
            let tau = b.sub(&a).normalize()?;
            let out = Vector2::from_array([tau[1], tau[0].neg()]);
            let parent = shifts[bend.parent].ok_or_else(|| {
                GeopError::new(format!(
                    "bend {} hangs off a flat that is not laid out before it",
                    bend.name.root()
                ))
            })?;
            shifts[bend.child] = Some(parent.add(&out.prod_scalar(self.developed_length(bend))));
        }
        shifts
            .into_iter()
            .enumerate()
            .map(|(f, s)| {
                s.ok_or_else(|| {
                    GeopError::new(format!(
                        "flat {} is joined to the rest by no bend",
                        self.flats[f].name.root()
                    ))
                })
            })
            .collect()
    }

    /// The A side of the sheet as it is bent, with its B side.
    pub fn folded(&self) -> GeopResult<SheetSurface<S>> {
        let places: Vec<Placement<S>> = self.flats.iter().map(|f| f.place.clone()).collect();
        self.surface(&places, None)
    }

    /// The A side of the sheet laid flat, with its B side: every flat in
    /// the first one's plane, moved by [`Sheet::flat_shifts`], and every
    /// bend a flat strip as wide as its developed length.
    pub fn unfolded(&self) -> GeopResult<SheetSurface<S>> {
        let shifts = self.flat_shifts()?;
        let places = vec![self.flats[0].place.clone(); self.flats.len()];
        self.surface(&places, Some(&shifts))
    }

    /// The sheet with flat `f` at `places[f]`, its curves moved by
    /// `shifts[f]` if laid flat — then every bend a flat strip.
    fn surface(
        &self,
        places: &[Placement<S>],
        shifts: Option<&[Vector2<S>]>,
    ) -> GeopResult<SheetSurface<S>> {
        let t = S::from_f64(self.rules.thickness);
        let mut sheet = SheetSurface::default();
        let mut edge_of: BTreeMap<String, usize> = BTreeMap::new();
        for (f, flat) in self.flats.iter().enumerate() {
            let ctx = with_context!("flat {}", flat.name.root());
            let (pa, pb) = (&places[f], places[f].offset(t));
            let mut loops = Vec::new();
            let mut curves = Vec::new();
            for lp in std::iter::once(&flat.outer).chain(&flat.holes) {
                let first = sheet.vertices.len();
                let n = lp.len();
                let mut coedges = Vec::new();
                for (k, e) in lp.iter().enumerate() {
                    let curve = match shifts {
                        Some(shifts) => translate2(&e.curve, &shifts[f]).with_context(ctx)?,
                        None => e.curve.clone(),
                    };
                    let p = start_point(&curve)?;
                    sheet.vertices.push(SheetVertex {
                        a: pa.point(&p),
                        b: pb.point(&p),
                        name: e.name.scoped("v"),
                    });
                    sheet.edges.push(SheetEdge {
                        a: pa.curve(&curve)?,
                        b: pb.curve(&curve)?,
                        start: first + k,
                        end: first + (k + 1) % n,
                        name: e.name.clone(),
                    });
                    if edge_of.insert(e.key(), sheet.edges.len() - 1).is_some() {
                        return Err(GeopError::new(format!("two edges are named {}", e.key())))
                            .with_context(ctx);
                    }
                    coedges.push(SheetCoedge {
                        edge: sheet.edges.len() - 1,
                        sense: Sense::Forward,
                        pcurve: curve,
                    });
                }
                curves.extend(coedges.iter().map(|c| c.pcurve.clone()));
                loops.push(coedges);
            }
            let curve_refs: Vec<&NurbCurve2D<S>> = curves.iter().collect();
            let outer = loops.remove(0);
            sheet.faces.push(SheetFace {
                a: pa.plane(&curve_refs).with_context(ctx)?,
                b: pb.plane(&curve_refs).with_context(ctx)?,
                outer,
                holes: loops,
                name: flat.name.clone(),
            });
        }

        for bend in &self.bends {
            let ctx = with_context!("bend {}", bend.name.root());
            let edge = |key: &str| {
                edge_of.get(key).copied().ok_or_else(|| {
                    GeopError::new(format!("bend {} meets no edge {key}", bend.name.root()))
                })
            };
            let (pe, ce) = (edge(&bend.parent_edge)?, edge(&bend.child_edge)?);
            let (pa, pb) = (sheet.edges[pe].start, sheet.edges[pe].end);
            let (cb, ca) = (sheet.edges[ce].start, sheet.edges[ce].end);
            let v = |i: usize| sheet.vertices[i].clone();
            let (face, s0, s1) = match shifts {
                None => {
                    let frame = self.bend_frame(bend).with_context(ctx)?;
                    let apex = |p: &Vector3<S>, r: S| p.add(&frame.m.prod_scalar(r.mul(frame.half_tan)));
                    let w = frame.half_cos;
                    let arc = |p: &SheetVertex<S>, c: &SheetVertex<S>| -> GeopResult<_> {
                        Ok((
                            arc3(p.a, apex(&p.a, frame.r_a), c.a, w)?,
                            arc3(p.b, apex(&p.b, frame.r_b), c.b, w)?,
                        ))
                    };
                    let (s0, s1) = (arc(&v(pa), &v(ca))?, arc(&v(pb), &v(cb))?);
                    // `u` from the parent across to the child, `v` along
                    // the parent's edge: `∂u × ∂v = m × tau = n`.
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
                    let p = |u: f64, v: f64| Vector2::from_array([S::from_f64(u), S::from_f64(v)]);
                    let pcurves = [
                        line2(p(0.0, 0.0), p(1.0, 0.0))?,
                        line2(p(1.0, 0.0), p(1.0, 1.0))?,
                        line2(p(1.0, 1.0), p(0.0, 1.0))?,
                        line2(p(0.0, 1.0), p(0.0, 0.0))?,
                    ];
                    let surfaces = (strip(&s0.0, &s1.0)?, strip(&s0.1, &s1.1)?);
                    ((surfaces, pcurves), s0, s1)
                }
                Some(shifts) => {
                    // Laid flat: a strip in the parent's sheet coordinates,
                    // the child's edge where the child was moved to.
                    let parent_line = self.parent_line(bend)?;
                    let into_parent = shifts[bend.child].sub(&shifts[bend.parent]);
                    let child = &self.flats[bend.child];
                    let c = child
                        .outer
                        .iter()
                        .find(|e| e.key() == bend.child_edge)
                        .ok_or_else(|| {
                            GeopError::new(format!(
                                "bend {} ends at {}, which is no edge of its child's outline",
                                bend.name.root(),
                                bend.child_edge
                            ))
                        })?;
                    let [c_b, c_a] = c.line().ok_or_else(|| {
                        GeopError::new(format!("bend {} ends at a curved edge", bend.name.root()))
                    })??;
                    let parent_shift = shifts[bend.parent];
                    let [p_a, p_b] = parent_line;
                    let (c_a, c_b) = (c_a.add(&into_parent), c_b.add(&into_parent));
                    let pcurves = [
                        line2(p_a, c_a)?,
                        line2(c_a, c_b)?,
                        line2(c_b, p_b)?,
                        line2(p_b, p_a)?,
                    ];
                    let place = places[bend.parent].shifted(&parent_shift);
                    let refs: Vec<&NurbCurve2D<S>> = pcurves.iter().collect();
                    let surfaces = (place.plane(&refs)?, place.offset(t).plane(&refs)?);
                    let straight = |p: &SheetVertex<S>, c: &SheetVertex<S>| -> GeopResult<_> {
                        Ok((line3(p.a, c.a)?, line3(p.b, c.b)?))
                    };
                    ((surfaces, pcurves), straight(&v(pa), &v(ca))?, straight(&v(pb), &v(cb))?)
                }
            };
            let ((a, b), [p0, p1, p2, p3]) = face;
            for (side, (curve_a, curve_b), start, end) in [("s0", s0, pa, ca), ("s1", s1, pb, cb)] {
                sheet.edges.push(SheetEdge {
                    a: curve_a,
                    b: curve_b,
                    start,
                    end,
                    name: bend.name.scoped(side),
                });
            }
            let n = sheet.edges.len();
            let coedge = |edge: usize, sense: Sense, pcurve: NurbCurve2D<S>| SheetCoedge {
                edge,
                sense,
                pcurve,
            };
            sheet.faces.push(SheetFace {
                a,
                b,
                outer: vec![
                    coedge(n - 2, Sense::Forward, p0),
                    coedge(ce, Sense::Reversed, p1),
                    coedge(n - 1, Sense::Reversed, p2),
                    coedge(pe, Sense::Reversed, p3),
                ],
                holes: Vec::new(),
                name: bend.name.clone(),
            });
        }
        Ok(sheet)
    }
}

/// A straight flat edge from `a` to `b`, named `name`.
pub fn straight<S: Scalar>(a: Vector2<S>, b: Vector2<S>, name: Namer) -> GeopResult<FlatEdge<S>> {
    Ok(FlatEdge {
        curve: line2(a, b)?,
        name,
    })
}
