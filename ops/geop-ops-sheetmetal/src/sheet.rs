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

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
};
use geop_core_topology::Body;
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::common::{end_point, line2, start_point};
use serde::{Deserialize, Serialize};

use crate::thicken::thicken;

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
        curve.embed(&self.origin, &self.e1, &self.e2)
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
    pub(crate) fn plane(&self, curves: &[&NurbCurve2D<S>]) -> GeopResult<NurbSurface3D<S>> {
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
        (self.curve.degree == 1 && self.curve.control_points.len() == 2)
            .then(|| Ok([start_point(&self.curve)?, end_point(&self.curve)?]))
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
    /// The cuts through it, in the order they were made, each taken out of
    /// the sheet as it is laid out flat (see [`crate::layout`]).
    pub cuts: Vec<Cut<S>>,
}

/// A cut through the sheet (see [`crate::SheetCut`]): closed loops, each
/// counter-clockwise around what it takes away, drawn in flat `flat`'s
/// sheet coordinates — and so, where they reach past the flat, across its
/// bends unrolled.
#[derive(Clone, Debug)]
pub struct Cut<S: Scalar> {
    pub flat: usize,
    pub loops: Vec<Vec<FlatEdge<S>>>,
    /// The cut's own name, `C` for the operation `sheet_cut(C)`.
    pub name: Namer,
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
        })
    }

    /// The vector `x` turned by the bend: `m` towards the side it turns
    /// to, about its axis.
    pub fn rotate(&self, x: &Vector3<S>) -> Vector3<S> {
        let (along_m, along_tau, along_n) = (
            x.prod_dot(&self.m),
            x.prod_dot(&self.tau),
            x.prod_dot(&self.n),
        );
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
    pub(crate) fn parent_line(&self, bend: &Bend<S>) -> GeopResult<[Vector2<S>; 2]> {
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
        let neutral = bend
            .radius
            .add(S::from_f64(self.rules.k_factor).mul(S::from_f64(self.rules.thickness)));
        bend.angle.mul(neutral)
    }

    /// Where each bend lies in the flat pattern, in the order the bends
    /// were made: the line down the middle of its strip, from where its
    /// parent edge starts to where it ends, in the first flat's sheet
    /// coordinates.
    pub fn bend_lines(&self) -> GeopResult<Vec<[Vector2<S>; 2]>> {
        let shifts = self.flat_shifts()?;
        self.bends
            .iter()
            .map(|bend| {
                let [a, b] = self.parent_line(bend)?;
                // Halfway across the strip, which reaches out of the parent.
                let tau = b.sub(&a).normalize()?;
                let out = Vector2::from_array([tau[1], tau[0].neg()]);
                let half = out.prod_scalar(self.developed_length(bend).div(S::TWO)?);
                let shift = shifts[bend.parent].add(&half);
                Ok([a.add(&shift), b.add(&shift)])
            })
            .collect()
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
}

impl<S: Scalar> Sheet<S> {
    /// Builds the solid of this sheet, as it is bent, in `part` in place of
    /// the solid `old` — named `name`, and recorded on it.
    pub(crate) fn replace(self, part: &mut Part<S>, old: &str, name: &str) -> GeopResult<()> {
        let folded = self.folded()?;
        let id = part.solid_id(old)?;
        part.assemble_sheet(&[Body::Solid(id)], &[])?;
        thicken(part, &folded, name, &|n| n)?;
        part.set_body_data(name, self)
    }

    /// The sheet-metal body of `part` with the edge `edge` on a flat's
    /// outline: the solid, its sheet, and the edge as
    /// [`Sheet::edge_named`] finds it. `what` is for the error: what is
    /// bent from such an edge.
    pub(crate) fn with_edge(
        part: &Part<S>,
        edge: &str,
        what: &str,
    ) -> GeopResult<(String, Self, (usize, usize, bool))> {
        part.solid_names()
            .into_iter()
            .find_map(|solid| {
                let sheet = part.body_data::<Sheet<S>>(&solid)?;
                let at = sheet.edge_named(edge)?;
                Some((solid, sheet.clone(), at))
            })
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{edge:?} is no edge of a sheet-metal body's flat face on its outline: {what} is bent from one of those"
                ))
            })
    }
}

/// A straight flat edge from `a` to `b`, named `name`.
pub fn straight<S: Scalar>(a: Vector2<S>, b: Vector2<S>, name: Namer) -> GeopResult<FlatEdge<S>> {
    Ok(FlatEdge {
        curve: line2(a, b)?,
        name,
    })
}
