//! [`PartView`]: a part as the viewport draws it, and picking entities of it
//! with a [`Pointer`].
//!
//! A part is drawn from its rasterization — sampled edges, triangulated
//! faces, outlined sketches — and a pick tests against exactly that, never
//! re-deriving geometry of its own, so a pick cannot disagree with what is
//! on screen. The same goes for what the viewer draws besides the part: the
//! origin's gizmo and the datums are laid out here, by the constants the
//! viewer draws them with, and picked by the same rules.

use geop_core_math::{geop_error::GeopResult, primitives::DatumKind, scalars::Scalar};
use geop_core_sketch::{CurveId, profile::curve_polyline};
use geop_core_topology::{FaceId, Model, SolidId};
use geop_ops_rasterize::rasterize_model_tagged;
use serde::Serialize;

use super::{
    Frame, Pointer,
    hit::{HIT_PX, add, dot, norm, ray_plane, ray_point, ray_segment, ray_triangle, scale, sub},
    visual::to_f64,
};
use crate::{Part, WorldAxis, operation::EntityRef};

/// Samples per edge and per parametric direction of a face. A face's grid is
/// refined further where its own curvature asks for it (see
/// `geop_ops_rasterize::grid`), so this is the floor, not the ceiling.
const RESOLUTION: usize = 24;

/// How tall the origin's gizmo is on screen, in pixels. Its origin ball,
/// axes and base-plane squares are laid out in fractions of that, in
/// [`PartView::pick`] as in the viewer.
pub const GIZMO_PX: f64 = 90.0;

/// What kind of entity a pick looks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Target {
    Vertex,
    Edge,
    Face,
    /// A solid, hit on any of its faces.
    Solid,
    /// A sketch, hit inside its closed regions or on its curves.
    Sketch,
    Datum(DatumKind),
    /// The origin, on the gizmo.
    Origin,
    /// A world axis, on the gizmo.
    Axis,
    /// A base plane, on the gizmo.
    BasePlane,
}

/// An entity a pointer is over, where, and how far along its ray.
#[derive(Clone, Debug, PartialEq)]
pub struct PartHit {
    pub entity: EntityRef,
    pub point: [f64; 3],
    pub t: f64,
}

/// A vertex of the part, as drawn.
#[derive(Clone, Debug)]
pub struct ViewVertex {
    pub name: String,
    pub at: [f64; 3],
}

/// An edge of the part, as drawn: its curve sampled into a polyline.
#[derive(Clone, Debug)]
pub struct ViewEdge {
    pub name: String,
    pub polyline: Vec<[f64; 3]>,
}

/// A face of the part, as drawn: triangulated, with the surface's normal at
/// each corner.
#[derive(Clone, Debug)]
pub struct ViewFace {
    pub name: String,
    /// The solid the face bounds.
    pub solid: Option<String>,
    pub triangles: Vec<[[f64; 3]; 3]>,
    pub normals: Vec<[[f64; 3]; 3]>,
}

/// A curve of a sketch, as drawn, in the sketch's plane.
#[derive(Clone, Debug)]
pub struct ViewCurve {
    pub id: CurveId,
    pub construction: bool,
    pub polyline: Vec<[f64; 2]>,
}

/// A sketch of the part, as drawn: its plane, its closed regions (each an
/// outer loop and its holes) and its curves, in the plane's coordinates.
#[derive(Clone, Debug)]
pub struct ViewSketch {
    pub name: String,
    pub plane: Frame,
    pub regions: Vec<Vec<Vec<[f64; 2]>>>,
    pub curves: Vec<ViewCurve>,
}

/// A datum of the part.
#[derive(Clone, Debug, Serialize)]
pub struct ViewDatum {
    pub name: String,
    pub kind: DatumKind,
    pub frame: Frame,
}

/// Where the drawing is and how big: the center and diagonal (at least 1)
/// of the box around it. Datum planes and axes, which are endless, are
/// drawn this big around the point of them nearest the center.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Extent {
    pub center: [f64; 3],
    pub size: f64,
}

/// A part as the viewport draws it, every entity by name.
#[derive(Clone, Debug)]
pub struct PartView {
    pub vertices: Vec<ViewVertex>,
    pub edges: Vec<ViewEdge>,
    pub faces: Vec<ViewFace>,
    pub sketches: Vec<ViewSketch>,
    pub datums: Vec<ViewDatum>,
    /// The part's solids, oldest first.
    pub solids: Vec<String>,
    pub extent: Extent,
}

/// The solid that owns `face`: a face is part of exactly one shell, and a
/// shell of exactly one solid.
pub fn solid_of_face<S: Scalar>(model: &Model<S>, face: FaceId) -> Option<SolidId> {
    model
        .get_face(face)
        .ok()
        .and_then(|f| model.get_shell(f.shell).ok())
        .map(|s| s.solid)
}

/// Whether `p` lies inside the closed polylines `loops`, outer boundaries
/// and holes alike: an odd number of crossings of a ray along `+x`.
fn inside_loops(loops: &[Vec<[f64; 2]>], p: [f64; 2]) -> bool {
    let mut inside = false;
    for poly in loops {
        for (i, a) in poly.iter().enumerate() {
            let b = poly[(i + 1) % poly.len()];
            if (a[1] > p[1]) != (b[1] > p[1]) {
                let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                if x > p[0] {
                    inside = !inside;
                }
            }
        }
    }
    inside
}

/// Distance from `p` to the segment `a..b`, in a plane.
fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let l2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if l2 == 0.0 {
        0.0
    } else {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2).clamp(0.0, 1.0)
    };
    (p[0] - a[0] - t * ab[0]).hypot(p[1] - a[1] - t * ab[1])
}

/// A world axis as a unit vector.
fn unit(axis: WorldAxis) -> [f64; 3] {
    match axis {
        WorldAxis::X => [1.0, 0.0, 0.0],
        WorldAxis::Y => [0.0, 1.0, 0.0],
        WorldAxis::Z => [0.0, 0.0, 1.0],
    }
}

const AXES: [WorldAxis; 3] = [WorldAxis::X, WorldAxis::Y, WorldAxis::Z];

impl PartView {
    /// `part` as drawn.
    pub fn of<S: Scalar>(part: &Part<S>) -> GeopResult<Self> {
        let model = part.topology();
        let raster = rasterize_model_tagged(model, RESOLUTION)?;
        let name = |id: crate::RefId| part.name_of(id).unwrap_or_default().to_string();

        let mut vertices: Vec<_> = raster.vertices.iter().collect();
        vertices.sort_by_key(|(id, _)| id.0);
        let mut edges: Vec<_> = raster.edges.iter().collect();
        edges.sort_by_key(|(id, _)| id.0);
        let mut faces: Vec<_> = raster.faces.iter().collect();
        faces.sort_by_key(|(id, _)| id.0);
        let mut solids: Vec<SolidId> = model.solids.keys().copied().collect();
        solids.sort_by_key(|s| s.0);

        let sketches = part
            .sketches()
            .map(|(id, placed)| {
                let s = &placed.sketch;
                let positions = s.positions();
                // A sketch whose curves form no region is still a sketch,
                // hit on its curves.
                let regions = s
                    .regions()
                    .map(|regions| {
                        regions
                            .iter()
                            .map(|r| {
                                std::iter::once(&r.outer)
                                    .chain(&r.holes)
                                    .map(|l| l.polyline(s, &positions))
                                    .collect()
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                ViewSketch {
                    name: name(id.into()),
                    plane: Frame::of(&placed.plane),
                    regions,
                    curves: s
                        .curves
                        .iter()
                        .map(|(&id, c)| ViewCurve {
                            id,
                            construction: c.construction,
                            polyline: curve_polyline(s, &positions, id),
                        })
                        .collect(),
                }
            })
            .collect();

        let mut view = PartView {
            vertices: vertices
                .into_iter()
                .map(|(&id, p)| ViewVertex {
                    name: name(id.into()),
                    at: to_f64(p),
                })
                .collect(),
            edges: edges
                .into_iter()
                .map(|(&id, polyline)| ViewEdge {
                    name: name(id.into()),
                    polyline: polyline.iter().map(to_f64).collect(),
                })
                .collect(),
            faces: faces
                .into_iter()
                .map(|(&id, tris)| ViewFace {
                    name: name(id.into()),
                    solid: solid_of_face(model, id).map(|s| name(s.into())),
                    triangles: tris
                        .iter()
                        .map(|t| [to_f64(&t.a), to_f64(&t.b), to_f64(&t.c)])
                        .collect(),
                    normals: tris
                        .iter()
                        .map(|t| {
                            t.vertex_normals
                                .unwrap_or([t.normal; 3])
                                .map(|n| to_f64(&n))
                        })
                        .collect(),
                })
                .collect(),
            sketches,
            datums: part
                .datums()
                .map(|(id, datum)| ViewDatum {
                    name: name(id.into()),
                    kind: datum.kind,
                    frame: Frame::of(&datum.frame),
                })
                .collect(),
            solids: solids.into_iter().map(|s| name(s.into())).collect(),
            extent: Extent {
                center: [0.0; 3],
                size: 1.0,
            },
        };
        view.extent = view.measure();
        Ok(view)
    }

    /// The box around everything drawn.
    fn measure(&self) -> Extent {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut grow = |p: [f64; 3]| {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        };
        self.vertices.iter().for_each(|v| grow(v.at));
        self.edges
            .iter()
            .flat_map(|e| &e.polyline)
            .for_each(|&p| grow(p));
        self.faces
            .iter()
            .flat_map(|f| f.triangles.iter().flatten())
            .for_each(|&p| grow(p));
        for sketch in &self.sketches {
            for curve in &sketch.curves {
                curve
                    .polyline
                    .iter()
                    .for_each(|&p| grow(sketch.plane.to_world(p)));
            }
        }
        if lo[0] > hi[0] {
            return Extent {
                center: [0.0; 3],
                size: 1.0,
            };
        }
        Extent {
            center: [0, 1, 2].map(|k| (lo[k] + hi[k]) / 2.0),
            size: norm(sub(hi, lo)).max(1.0),
        }
    }

    /// Whatever of the `targets` kinds the pointer is over. The gizmo is
    /// drawn on top of everything, so it is picked first. Otherwise a vertex
    /// or an edge near the pointer — and not hidden behind a face — wins:
    /// the smallest entity under the pointer is the one meant. Else the
    /// nearest along the ray of the faces, sketches and datums hit.
    pub fn pick(&self, pointer: &Pointer, targets: &[Target]) -> Option<PartHit> {
        let wants = |t: Target| targets.contains(&t);
        if let Some(hit) = self.pick_gizmo(pointer, targets) {
            return Some(hit);
        }
        let face = (wants(Target::Face)
            || wants(Target::Solid)
            || wants(Target::Vertex)
            || wants(Target::Edge))
        .then(|| self.pick_face(pointer))
        .flatten();
        // In front of the face hit, give or take the tolerance: a vertex or
        // an edge on the face's own boundary lies right at it.
        let visible = |t: f64| {
            face.as_ref()
                .is_none_or(|(ft, _)| t <= ft + pointer.pixels(HIT_PX, *ft))
        };
        let near = |dist: f64, t: f64| dist <= pointer.pixels(HIT_PX, t);

        if wants(Target::Vertex) {
            let vertex = self
                .vertices
                .iter()
                .filter_map(|v| {
                    let (dist, t) = ray_point(pointer, v.at);
                    (near(dist, t) && visible(t)).then_some((t, v))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((t, v)) = vertex {
                return Some(PartHit {
                    entity: EntityRef::Vertex {
                        name: v.name.clone(),
                    },
                    point: v.at,
                    t,
                });
            }
        }
        if wants(Target::Edge) {
            let edge = self
                .edges
                .iter()
                .flat_map(|e| e.polyline.windows(2).map(move |w| (e, w)))
                .filter_map(|(e, w)| {
                    let (dist, t) = ray_segment(pointer, w[0], w[1]);
                    (near(dist, t) && visible(t)).then_some((t, e))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((t, e)) = edge {
                return Some(PartHit {
                    entity: EntityRef::Edge {
                        name: e.name.clone(),
                    },
                    point: pointer.at(t),
                    t,
                });
            }
        }

        let mut hits: Vec<PartHit> = Vec::new();
        if let Some((t, f)) = face {
            if wants(Target::Face) {
                hits.push(PartHit {
                    entity: EntityRef::Face {
                        name: f.name.clone(),
                    },
                    point: pointer.at(t),
                    t,
                });
            } else if wants(Target::Solid)
                && let Some(solid) = &f.solid
            {
                hits.push(PartHit {
                    entity: EntityRef::Solid {
                        name: solid.clone(),
                    },
                    point: pointer.at(t),
                    t,
                });
            }
        }
        if wants(Target::Sketch) {
            hits.extend(self.pick_sketch(pointer));
        }
        hits.extend(self.pick_datum(pointer, targets));
        hits.into_iter().min_by(|a, b| a.t.total_cmp(&b.t))
    }

    /// The nearest face the ray enters.
    fn pick_face(&self, pointer: &Pointer) -> Option<(f64, &ViewFace)> {
        self.faces
            .iter()
            .flat_map(|f| f.triangles.iter().map(move |tri| (f, tri)))
            .filter_map(|(f, [a, b, c])| ray_triangle(pointer, *a, *b, *c).map(|t| (t, f)))
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }

    /// The nearest sketch hit inside one of its closed regions — the area a
    /// viewer shades — or near one of its curves.
    fn pick_sketch(&self, pointer: &Pointer) -> Option<PartHit> {
        self.sketches
            .iter()
            .filter_map(|sketch| {
                let (p, t) = sketch.plane.at_pointer(pointer)?;
                let tolerance = pointer.pixels(HIT_PX, t);
                let in_region = sketch.regions.iter().any(|loops| inside_loops(loops, p));
                let on_curve = || {
                    sketch.curves.iter().any(|c| {
                        c.polyline
                            .windows(2)
                            .any(|w| segment_distance(p, w[0], w[1]) <= tolerance)
                    })
                };
                (in_region || on_curve()).then(|| PartHit {
                    entity: EntityRef::Sketch {
                        name: sketch.name.clone(),
                    },
                    point: pointer.at(t),
                    t,
                })
            })
            .min_by(|a, b| a.t.total_cmp(&b.t))
    }

    /// The nearest datum of a targeted kind the ray hits, as the viewer
    /// draws it: a point or a frame at its origin, an axis as a line and a
    /// plane as a square, each [`Extent::size`] long around the point of it
    /// nearest the extent's center.
    fn pick_datum(&self, pointer: &Pointer, targets: &[Target]) -> Option<PartHit> {
        let Extent { center, size } = self.extent;
        self.datums
            .iter()
            .filter(|d| targets.contains(&Target::Datum(d.kind)))
            .filter_map(|d| {
                let f = &d.frame;
                let t = match d.kind {
                    DatumKind::Point | DatumKind::Frame => {
                        let (dist, t) = ray_point(pointer, f.origin);
                        (dist <= pointer.pixels(HIT_PX, t)).then_some(t)?
                    }
                    DatumKind::Axis => {
                        let mid = add(
                            f.origin,
                            scale(f.normal, dot(sub(center, f.origin), f.normal)),
                        );
                        let reach = scale(f.normal, size);
                        let (dist, t) = ray_segment(pointer, sub(mid, reach), add(mid, reach));
                        (dist <= pointer.pixels(HIT_PX, t)).then_some(t)?
                    }
                    DatumKind::Plane => {
                        let (t, point) = ray_plane(pointer, f.origin, f.normal)?;
                        let d = sub(point, center);
                        (dot(d, f.u).abs() <= size / 2.0 && dot(d, f.v).abs() <= size / 2.0)
                            .then_some(t)?
                    }
                };
                Some(PartHit {
                    entity: EntityRef::Datum {
                        name: d.name.clone(),
                    },
                    point: pointer.at(t),
                    t,
                })
            })
            .min_by(|a, b| a.t.total_cmp(&b.t))
    }

    /// The part of the origin's gizmo of a targeted kind the ray hits: its
    /// ball, else an axis (from a tenth of its length to its tip), else a
    /// base plane's square (off the origin, between the other two axes) —
    /// the smallest part under the pointer, as for the part's entities.
    fn pick_gizmo(&self, pointer: &Pointer, targets: &[Target]) -> Option<PartHit> {
        let size = GIZMO_PX * pointer.pixel.at(norm(pointer.origin));
        if targets.contains(&Target::Origin) {
            let (dist, t) = ray_point(pointer, [0.0; 3]);
            if dist <= 0.12 * size {
                return Some(PartHit {
                    entity: EntityRef::Origin,
                    point: [0.0; 3],
                    t,
                });
            }
        }
        let (mut axes, mut planes) = (Vec::new(), Vec::new());
        for axis in AXES {
            let d = unit(axis);
            if targets.contains(&Target::Axis) {
                let (dist, t) = ray_segment(pointer, scale(d, 0.1 * size), scale(d, size));
                if dist <= (0.06 * size).max(pointer.pixels(4.0, t)) {
                    axes.push(PartHit {
                        entity: EntityRef::Axis { axis },
                        point: scale(d, size),
                        t,
                    });
                }
            }
            if targets.contains(&Target::BasePlane)
                && let Some((t, point)) = ray_plane(pointer, [0.0; 3], d)
            {
                let in_square = AXES
                    .iter()
                    .filter(|&&other| other != axis)
                    .all(|&other| (dot(point, unit(other)) / size - 0.45).abs() <= 0.15);
                if in_square {
                    planes.push(PartHit {
                        entity: EntityRef::Plane { normal: axis },
                        point,
                        t,
                    });
                }
            }
        }
        let nearest = |hits: Vec<PartHit>| hits.into_iter().min_by(|a, b| a.t.total_cmp(&b.t));
        nearest(axes).or_else(|| nearest(planes))
    }
}
