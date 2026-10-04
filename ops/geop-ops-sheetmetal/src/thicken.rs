//! Thickening one side of a sheet into the solid sheet of metal it is:
//! [`SheetSurface`], and [`thicken`], which builds its solid in one go.
//!
//! A sheet-metal part is a surface — its *A side* — made solid by giving it
//! a thickness: every face of the A side has its copy on the *B side*, a
//! thickness further along the surface's normal, and every edge on the A
//! side's boundary sweeps a *wall* across the thickness to its copy. The
//! faces here are planes and cylinders around an axis, whose copies are
//! exact: a plane moved along its normal, a cylinder of a radius one
//! thickness larger or smaller about the same axis. So the B side is given
//! with the A side, and nothing is approximated.

use std::collections::BTreeMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Curve3, Sense,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_extrude_revolve::common::{line2, line3};

/// A vertex of the A side, and where it is on the B side.
#[derive(Clone, Debug)]
pub struct SheetVertex<S: Scalar> {
    pub a: Vector3<S>,
    pub b: Vector3<S>,
    /// Its vertices are `N(a)` and `N(b)`, the edge across the thickness
    /// between them, on the boundary, `N`.
    pub name: Namer,
}

/// An edge of the A side, from vertex `start` to vertex `end`, and its copy
/// on the B side: the same degree, knots and weights, so that the wall
/// between them is ruled exactly.
#[derive(Clone, Debug)]
pub struct SheetEdge<S: Scalar> {
    pub a: Curve3<S>,
    pub b: Curve3<S>,
    pub start: usize,
    pub end: usize,
    /// Its edges are `N(a)` and `N(b)`, its wall, on the boundary, `N`.
    pub name: Namer,
}

/// One coedge of a face's loop: an edge in a sense, and its pcurve — on
/// both of the face's surfaces, which share their parametrization.
#[derive(Clone, Debug)]
pub struct SheetCoedge<S: Scalar> {
    pub edge: usize,
    pub sense: Sense,
    pub pcurve: NurbCurve2D<S>,
}

/// A face of the A side, and its copy on the B side, parametrized alike:
/// each with `∂u × ∂v` along the sheet's normal, from the A side towards
/// the B side, and its loops running counter-clockwise in `(u, v)` — the
/// outer one, holes clockwise.
#[derive(Clone, Debug)]
pub struct SheetFace<S: Scalar> {
    pub a: NurbSurface3D<S>,
    pub b: NurbSurface3D<S>,
    pub outer: Vec<SheetCoedge<S>>,
    pub holes: Vec<Vec<SheetCoedge<S>>>,
    /// Its faces are `N(a)` and `N(b)`.
    pub name: Namer,
}

/// The A side of a sheet, with its B side: what [`thicken`] builds a solid
/// from.
#[derive(Clone, Debug, Default)]
pub struct SheetSurface<S: Scalar> {
    pub vertices: Vec<SheetVertex<S>>,
    pub edges: Vec<SheetEdge<S>>,
    pub faces: Vec<SheetFace<S>>,
}

impl<S: Scalar> SheetSurface<S> {
    /// Per edge, the faces using it: `(face, sense)`.
    fn uses(&self) -> Vec<Vec<(usize, Sense)>> {
        let mut uses = vec![Vec::new(); self.edges.len()];
        for (f, face) in self.faces.iter().enumerate() {
            for coedge in std::iter::once(&face.outer).chain(&face.holes).flatten() {
                uses[coedge.edge].push((f, coedge.sense));
            }
        }
        uses
    }

    /// Per edge, whether it lies on the boundary — used by one face only —
    /// checked to be used once or twice, then in opposite senses.
    fn boundary(&self) -> GeopResult<Vec<bool>> {
        self.uses()
            .iter()
            .enumerate()
            .map(|(e, uses)| match uses[..] {
                [_] => Ok(true),
                [(_, a), (_, b)] if a != b => Ok(false),
                _ => Err(GeopError::new(format!(
                    "the sheet's edge {} is used {:?} by its faces",
                    self.edges[e].name.root(),
                    uses
                ))),
            })
            .collect()
    }

    /// The names of the faces [`thicken`] builds, before `rename`: both
    /// sides of every face, and a wall on every edge of the boundary.
    pub fn face_names(&self) -> GeopResult<Vec<String>> {
        let boundary = self.boundary()?;
        let mut names: Vec<String> = self
            .faces
            .iter()
            .flat_map(|f| [f.name.name(&["a"]), f.name.name(&["b"])])
            .collect();
        names.extend(
            self.edges
                .iter()
                .zip(&boundary)
                .filter(|(_, b)| **b)
                .map(|(e, _)| e.name.root()),
        );
        Ok(names)
    }
}

/// The wall between an edge's A curve and its B curve: `u` along the
/// curve, `v` across the thickness. A ruled surface, exact for curves of
/// one degree, knots and weights.
fn wall<S: Scalar>(edge: &SheetEdge<S>) -> GeopResult<NurbSurface3D<S>> {
    let (a, b) = (&edge.a, &edge.b);
    if a.degree != b.degree
        || a.control_points.len() != b.control_points.len()
        || a.knot_vector.len() != b.knot_vector.len()
    {
        return Err(GeopError::new(format!(
            "the two sides of edge {} are not alike: {a:?} and {b:?}",
            edge.name.root()
        )));
    }
    let control_points = a
        .control_points
        .iter()
        .zip(&b.control_points)
        .flat_map(|(p, q)| [*p, *q])
        .collect();
    NurbSurface::try_new(
        a.degree,
        1,
        control_points,
        a.knot_vector.clone(),
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// Builds the solid sheet of metal `sheet` is the A side of — both sides of
/// each of its faces, a wall on each edge of its boundary — named `solid`,
/// and each of its entities as its [`SheetSurface`] entry says, through
/// `rename`.
///
/// Every vertex on the boundary must have one boundary edge arriving and
/// one leaving, so that the walls close up around it: the sheet is a
/// surface with a boundary, not two touching at a point.
pub fn thicken<S: Scalar>(
    part: &mut Part<S>,
    sheet: &SheetSurface<S>,
    solid: &str,
    rename: &dyn Fn(String) -> String,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!(
        "thicken({solid}: {} faces, {} edges, {} vertices)",
        sheet.faces.len(),
        sheet.edges.len(),
        sheet.vertices.len()
    );
    let boundary = sheet.boundary().with_context(ctx)?;
    let nv = sheet.vertices.len();
    let mut spec = BodySpec {
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: true,
    };
    let mut names = BodyNames {
        solid: Some(solid.to_string()),
        ..BodyNames::default()
    };
    // Vertex `v` is `v` on the A side and `nv + v` on the B side.
    for side in ["a", "b"] {
        for v in &sheet.vertices {
            spec.vertices.push(if side == "a" { v.a } else { v.b });
            names.vertices.push(rename(v.name.name(&[side])));
        }
    }
    // Edge `e` is `2e` on the A side and `2e + 1` on the B side.
    for e in &sheet.edges {
        spec.edges.push(EdgeSpec {
            curve: e.a.clone(),
            start: e.start,
            end: e.end,
        });
        names.edges.push(rename(e.name.name(&["a"])));
        spec.edges.push(EdgeSpec {
            curve: e.b.clone(),
            start: nv + e.start,
            end: nv + e.end,
        });
        names.edges.push(rename(e.name.name(&["b"])));
    }
    // An edge across the thickness at every vertex of the boundary, which
    // must have one boundary edge arriving and one leaving.
    let mut arriving = vec![0; nv];
    let mut leaving = vec![0; nv];
    for (e, edge) in sheet.edges.iter().enumerate() {
        if boundary[e] {
            leaving[edge.start] += 1;
            arriving[edge.end] += 1;
        }
    }
    let mut across: BTreeMap<usize, usize> = BTreeMap::new();
    for (v, vertex) in sheet.vertices.iter().enumerate() {
        match (arriving[v] + leaving[v], arriving[v] == leaving[v]) {
            (0, _) => {}
            (2, _) => {
                spec.edges.push(EdgeSpec {
                    curve: line3(vertex.a, vertex.b).with_context(ctx)?,
                    start: v,
                    end: nv + v,
                });
                names.edges.push(rename(vertex.name.root()));
                across.insert(v, spec.edges.len() - 1);
            }
            (n, _) => {
                return Err(GeopError::new(format!(
                    "the sheet's boundary meets itself at vertex {}: {n} boundary edges end there, where two must",
                    vertex.name.root()
                )))
                .with_context(ctx);
            }
        }
    }

    let coedges = |lp: &[SheetCoedge<S>], side: usize| -> Vec<CoedgeSpec<S>> {
        lp.iter()
            .map(|c| CoedgeSpec {
                on: CoedgeOn::Edge(2 * c.edge + side, c.sense),
                pcurve: c.pcurve.clone(),
            })
            .collect()
    };
    for face in &sheet.faces {
        // `∂u × ∂v` points from the A side to the B side: out of the
        // material on the B side, into it on the A side.
        let b = FaceSpec {
            surface: face.b.clone(),
            outer: coedges(&face.outer, 1),
            holes: face.holes.iter().map(|h| coedges(h, 1)).collect(),
        };
        let a = FaceSpec {
            surface: face.a.clone(),
            outer: coedges(&face.outer, 0),
            holes: face.holes.iter().map(|h| coedges(h, 0)).collect(),
        }
        .reversed();
        spec.faces.push(a);
        names.faces.push(rename(face.name.name(&["a"])));
        spec.faces.push(b);
        names.faces.push(rename(face.name.name(&["b"])));
    }

    // The walls. An edge of a face's loop runs with the face on its left,
    // seen from the B side, so its wall — `u` along the curve, `v` from the
    // A side to the B side — faces out of the material where the loop runs
    // the edge forwards, and is turned around where it runs it backwards.
    let uses = sheet.uses();
    let point = |u: S, v: S| Vector2::from_array([u, v]);
    for (e, edge) in sheet.edges.iter().enumerate() {
        if !boundary[e] {
            continue;
        }
        let (t0, t1) = edge.a.domain();
        let face = FaceSpec {
            surface: wall(edge).with_context(ctx)?,
            outer: vec![
                CoedgeSpec {
                    on: CoedgeOn::Edge(2 * e, Sense::Forward),
                    pcurve: line2(point(t0, S::ZERO), point(t1, S::ZERO))?,
                },
                CoedgeSpec {
                    on: CoedgeOn::Edge(across[&edge.end], Sense::Forward),
                    pcurve: line2(point(t1, S::ZERO), point(t1, S::ONE))?,
                },
                CoedgeSpec {
                    on: CoedgeOn::Edge(2 * e + 1, Sense::Reversed),
                    pcurve: line2(point(t1, S::ONE), point(t0, S::ONE))?,
                },
                CoedgeSpec {
                    on: CoedgeOn::Edge(across[&edge.start], Sense::Reversed),
                    pcurve: line2(point(t0, S::ONE), point(t0, S::ZERO))?,
                },
            ],
            holes: Vec::new(),
        };
        spec.faces.push(match uses[e][0].1 {
            Sense::Forward => face,
            Sense::Reversed => face.reversed(),
        });
        names.faces.push(rename(edge.name.root()));
    }
    spec.shells = vec![(0..spec.faces.len()).collect()];
    part.build_body(spec, names).with_context(ctx)
}

/// `curve` moved by `shift` in its plane: every homogeneous control point
/// by `shift` times its weight — exact for any NURBS. Moved by nothing, it
/// is the curve itself, not widened by adding zeros.
pub(crate) fn translate2<S: Scalar>(
    curve: &NurbCurve2D<S>,
    shift: &Vector2<S>,
) -> GeopResult<NurbCurve2D<S>> {
    let zero = |x: S| x.is_sharp() && x.could_be_equal(S::ZERO);
    if zero(shift[0]) && zero(shift[1]) {
        return Ok(curve.clone());
    }
    NurbCurve::try_new(
        curve.degree,
        curve
            .control_points
            .iter()
            .map(|cp| {
                Vector3::from_array([
                    cp[0].add(shift[0].mul(cp[2])),
                    cp[1].add(shift[1].mul(cp[2])),
                    cp[2],
                ])
            })
            .collect(),
        curve.knot_vector.clone(),
    )
}
