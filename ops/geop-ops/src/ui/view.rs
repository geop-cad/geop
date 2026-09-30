//! [`PartView`]: a part as the viewport draws it, and picking entities of it
//! with a [`Pointer`].
//!
//! A part is drawn from its rasterization — sampled edges, triangulated
//! faces, outlined sketches — and a pick tests against exactly that, never
//! re-deriving geometry of its own, so a pick cannot disagree with what is
//! on screen. The same goes for what the viewer draws besides the part: the
//! datums are laid out here, by the sizes the viewer draws them with, and
//! picked by the same rules.

use geop_core_math::{
    geop_error::GeopResult,
    polygon::loops_contain,
    primitives::{CoordinateSystem, DatumComponent, DatumKind, FrameAxis},
    scalars::{Scalar, as_f64},
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{CurveId, PointId, point::P2, profile::curve_polyline};
use geop_core_topology::{FaceId, Model, SolidId};
use geop_ops_rasterize::rasterize;
use serde::Serialize;

use super::{Pointer, hit::nearer};
use crate::{
    Part,
    operation::{Aspects, EntityRef, Role},
};

/// Samples per edge and per parametric direction of a face. A face's grid is
/// refined further where its own curvature asks for it (see
/// `geop_ops_rasterize::grid`), so this is the floor, not the ceiling.
const RESOLUTION: usize = 24;

/// How long a frame datum's axes are, in reaches: it is drawn at a constant
/// size on screen, with its origin ball, its axes and the squares of its
/// planes laid out in fractions of that, in [`PartView::pick`] as in the
/// viewer.
pub const FRAME: f64 = 10.0;

/// An entity a pointer is over, where, and how far along its ray.
#[derive(Clone, Debug, PartialEq)]
pub struct PartHit<S: Scalar> {
    pub entity: EntityRef,
    pub point: Vector3<S>,
    pub t: S,
}

/// A vertex of the part, as drawn.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewVertex<S: Scalar> {
    pub name: String,
    pub at: Vector3<S>,
}

/// The roles `entity` can fill in `part` (see [`Aspects`]): none for one
/// that does not resolve.
fn roles_of<S: Scalar>(entity: &EntityRef, part: &Part<S>) -> Vec<Role> {
    Aspects::of(entity, part)
        .map(|g| g.roles())
        .unwrap_or_default()
}

/// An edge of the part, as drawn: its curve sampled into a polyline.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewEdge<S: Scalar> {
    pub name: String,
    pub polyline: Vec<Vector3<S>>,
    /// What it can be picked as: an edge, and a line or a circle if it is
    /// one.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A face of the part, as drawn: triangulated, with the surface's normal at
/// each corner.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewFace<S: Scalar> {
    pub name: String,
    /// The solid the face bounds.
    pub solid: Option<String>,
    pub triangles: Vec<[Vector3<S>; 3]>,
    pub normals: Vec<[Vector3<S>; 3]>,
    /// What it can be picked as: a plane if it is flat, round if it turns
    /// around an axis.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A curve of a sketch, as drawn, in the sketch's plane.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewCurve<S: Scalar> {
    pub id: CurveId,
    pub construction: bool,
    pub polyline: Vec<Vector2<S>>,
    /// What it can be picked as: a line, if it is one.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A point of a sketch, as drawn, in the sketch's plane.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewSketchPoint<S: Scalar> {
    pub id: PointId,
    pub at: Vector2<S>,
}

/// A sketch of the part, as drawn: its plane, its closed regions (each an
/// outer loop and its holes), its curves and its points, in the plane's
/// `u`/`v` coordinates.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewSketch<S: Scalar> {
    pub name: String,
    pub plane: CoordinateSystem<S>,
    #[serde(skip)]
    pub regions: Vec<Vec<Vec<Vector2<S>>>>,
    pub curves: Vec<ViewCurve<S>>,
    pub points: Vec<ViewSketchPoint<S>>,
}

/// A datum of the part.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewDatum<S: Scalar> {
    pub name: String,
    pub kind: DatumKind,
    pub frame: CoordinateSystem<S>,
}

/// Where the drawing is and how big: the center and diagonal (at least 1)
/// of the box around it. Datum planes and axes, which are endless, are
/// drawn this big around the point of them nearest the center.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Extent<S: Scalar> {
    pub center: Vector3<S>,
    #[serde(with = "as_f64")]
    pub size: S,
}

/// A part as the viewport draws it, every entity by name — and, serialized,
/// what a viewer draws.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct PartView<S: Scalar> {
    pub vertices: Vec<ViewVertex<S>>,
    pub edges: Vec<ViewEdge<S>>,
    pub faces: Vec<ViewFace<S>>,
    pub sketches: Vec<ViewSketch<S>>,
    pub datums: Vec<ViewDatum<S>>,
    /// The part's solids, oldest first.
    pub solids: Vec<String>,
    pub extent: Extent<S>,
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

impl<S: Scalar> PartView<S> {
    /// `part` as drawn.
    pub fn of(part: &Part<S>) -> GeopResult<Self> {
        let model = part.topology();
        let raster = rasterize(model, RESOLUTION)?;
        let name = |id: crate::RefId| part.name_of(id).unwrap_or_default().to_string();
        let uv = |polyline: Vec<P2>| {
            polyline
                .into_iter()
                .map(|p| Vector2::from_array(p.map(S::from_f64)))
                .collect::<Vec<_>>()
        };

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
                let sketch = name(id.into());
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
                                    .map(|l| uv(l.polyline(s, &positions)))
                                    .collect()
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                ViewSketch {
                    name: sketch.clone(),
                    plane: placed.plane.clone(),
                    regions,
                    curves: s
                        .curves
                        .iter()
                        .map(|(&id, c)| ViewCurve {
                            id,
                            construction: c.construction,
                            polyline: uv(curve_polyline(s, &positions, id)),
                            roles: roles_of(
                                &EntityRef::SketchCurve {
                                    sketch: sketch.clone(),
                                    curve: id,
                                },
                                part,
                            ),
                        })
                        .collect(),
                    points: positions
                        .iter()
                        .map(|(&id, &p)| ViewSketchPoint {
                            id,
                            at: Vector2::from_array(p.map(S::from_f64)),
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
                    at: *p,
                })
                .collect(),
            edges: edges
                .into_iter()
                .map(|(&id, polyline)| {
                    let edge = name(id.into());
                    ViewEdge {
                        roles: roles_of(&EntityRef::Edge { name: edge.clone() }, part),
                        name: edge,
                        polyline: polyline.clone(),
                    }
                })
                .collect(),
            faces: faces
                .into_iter()
                .map(|(&id, tris)| {
                    let face = name(id.into());
                    ViewFace {
                        roles: roles_of(&EntityRef::Face { name: face.clone() }, part),
                        name: face,
                        solid: solid_of_face(model, id).map(|s| name(s.into())),
                        triangles: tris.iter().map(|t| [t.a, t.b, t.c]).collect(),
                        normals: tris
                            .iter()
                            .map(|t| t.vertex_normals.unwrap_or([t.normal; 3]))
                            .collect(),
                    }
                })
                .collect(),
            sketches,
            datums: part
                .datums()
                .map(|(id, datum)| ViewDatum {
                    name: name(id.into()),
                    kind: datum.kind,
                    frame: datum.frame.clone(),
                })
                .collect(),
            solids: solids.into_iter().map(|s| name(s.into())).collect(),
            extent: Extent {
                center: Vector3::zero(),
                size: S::ONE,
            },
        };
        view.extent = view.measure();
        Ok(view)
    }

    /// The box around everything drawn: the union of every point of it.
    fn measure(&self) -> Extent<S> {
        let sketch_points = self.sketches.iter().flat_map(|sketch| {
            sketch
                .curves
                .iter()
                .flat_map(|c| &c.polyline)
                .map(|p| sketch.plane.uv_to_xyz(p))
        });
        let hull = self
            .vertices
            .iter()
            .map(|v| v.at)
            .chain(self.edges.iter().flat_map(|e| e.polyline.iter().copied()))
            .chain(
                self.faces
                    .iter()
                    .flat_map(|f| f.triangles.iter().flatten().copied()),
            )
            .chain(sketch_points)
            .reduce(|a, b| a.union(&b));
        let Some(hull) = hull else {
            return Extent {
                center: Vector3::zero(),
                size: S::ONE,
            };
        };
        let size = Vector3::from_array([0, 1, 2].map(|k| hull[k].width())).norm();
        Extent {
            center: Vector3::from_array([0, 1, 2].map(|k| hull[k].midpoint())),
            size: if size.definitely_less(S::ONE) {
                S::ONE
            } else {
                size
            },
        }
    }

    /// Whatever the pointer is over that can fill one of `roles` — and, with
    /// a `scope`, is part of it: a line of one sketch. Frame datums are drawn
    /// on top of everything, so they are picked first. Otherwise a vertex or
    /// a sketch point, else an edge or a sketch curve, near the pointer — and not hidden behind a
    /// face — wins: the smallest entity under the pointer is the one meant.
    /// Else the nearest along the ray of the faces, solids, sketches and
    /// other datums hit.
    pub fn pick(
        &self,
        pointer: &Pointer<S>,
        roles: &[Role],
        scope: Option<&EntityRef>,
    ) -> Option<PartHit<S>> {
        let accept = |entity: &EntityRef, its: &[Role]| {
            its.iter().any(|r| roles.contains(r)) && scope.is_none_or(|s| entity.lies_in(s))
        };
        if let Some(hit) = self.pick_frame(pointer, &accept) {
            return Some(hit);
        }
        let ray = &pointer.ray;
        let face = self.pick_face(pointer);
        // In front of the face hit, give or take the reach: a vertex or an
        // edge on the face's own boundary lies right at it.
        let visible = |t: S| {
            face.as_ref()
                .is_none_or(|(ft, _)| !t.definitely_greater(ft.add(pointer.reach_at(1.0, *ft))))
        };
        let near = |(dist, t): (S, S)| (pointer.within(dist, t, 1.0) && visible(t)).then_some(t);
        let nearest = |hits: Vec<PartHit<S>>| hits.into_iter().min_by(|a, b| nearer(a.t, b.t));

        let vertices = self.vertices.iter().map(|v| {
            let entity = EntityRef::Vertex {
                name: v.name.clone(),
            };
            (entity, v.at)
        });
        let sketch_points = self.sketches.iter().flat_map(|sketch| {
            sketch.points.iter().map(|p| {
                let entity = EntityRef::SketchPoint {
                    sketch: sketch.name.clone(),
                    point: p.id,
                };
                (entity, sketch.plane.uv_to_xyz(&p.at))
            })
        });
        let points = vertices
            .chain(sketch_points)
            .filter(|(entity, _)| accept(entity, &[Role::Point]))
            .filter_map(|(entity, at)| {
                near(ray.distance_to_point(&at)).map(|t| PartHit {
                    entity,
                    point: at,
                    t,
                })
            })
            .collect();
        if let Some(hit) = nearest(points) {
            return Some(hit);
        }

        let polyline_hit =
            |entity: EntityRef, polyline: &mut dyn Iterator<Item = (Vector3<S>, Vector3<S>)>| {
                polyline
                    .filter_map(|(a, b)| near(ray.distance_to_segment(&a, &b)))
                    .min_by(|&a, &b| nearer(a, b))
                    .map(|t| PartHit {
                        entity: entity.clone(),
                        point: ray.at(t),
                        t,
                    })
            };
        let mut curves = Vec::new();
        for e in &self.edges {
            let entity = EntityRef::Edge {
                name: e.name.clone(),
            };
            if accept(&entity, &e.roles) {
                let mut segments = e.polyline.windows(2).map(|w| (w[0], w[1]));
                curves.extend(polyline_hit(entity, &mut segments));
            }
        }
        for sketch in &self.sketches {
            for c in &sketch.curves {
                let entity = EntityRef::SketchCurve {
                    sketch: sketch.name.clone(),
                    curve: c.id,
                };
                if accept(&entity, &c.roles) {
                    let world = |p: &Vector2<S>| sketch.plane.uv_to_xyz(p);
                    let mut segments = c.polyline.windows(2).map(|w| (world(&w[0]), world(&w[1])));
                    curves.extend(polyline_hit(entity, &mut segments));
                }
            }
        }
        if let Some(hit) = nearest(curves) {
            return Some(hit);
        }

        let mut hits: Vec<PartHit<S>> = Vec::new();
        if let Some((t, f)) = face {
            let face = EntityRef::Face {
                name: f.name.clone(),
            };
            let solid = f.solid.clone().map(|name| EntityRef::Solid { name });
            let entity = if accept(&face, &f.roles) {
                Some(face)
            } else {
                solid.filter(|solid| accept(solid, &[Role::Solid]))
            };
            hits.extend(entity.map(|entity| PartHit {
                entity,
                point: ray.at(t),
                t,
            }));
        }
        hits.extend(self.pick_sketch(pointer, &accept));
        hits.extend(self.pick_datum(pointer, &accept));
        nearest(hits)
    }

    /// Whether `entity` — a sketch or a datum — or a part of it can fill one
    /// of `roles`: whether a pick for them could take something of it.
    pub fn can_fill(&self, entity: &EntityRef, roles: &[Role]) -> bool {
        let fills = |its: &[Role]| its.iter().any(|r| roles.contains(r));
        match entity {
            EntityRef::Sketch { name } => self.sketches.iter().any(|s| {
                s.name == *name
                    && (fills(&[Role::Sketch])
                        || (!s.points.is_empty() && fills(&[Role::Point]))
                        || s.curves.iter().any(|c| fills(&c.roles)))
            }),
            EntityRef::Datum { name, .. } => self.datums.iter().any(|d| {
                d.name == *name
                    && match d.kind {
                        DatumKind::Frame => fills(&[Role::Point, Role::Line, Role::Plane]),
                        kind => fills(&[datum_role(kind)]),
                    }
            }),
            _ => false,
        }
    }

    /// The nearest face the ray enters.
    fn pick_face(&self, pointer: &Pointer<S>) -> Option<(S, &ViewFace<S>)> {
        self.faces
            .iter()
            .flat_map(|f| f.triangles.iter().map(move |tri| (f, tri)))
            .filter_map(|(f, [a, b, c])| pointer.ray.intersect_triangle(a, b, c).map(|t| (t, f)))
            .min_by(|a, b| nearer(a.0, b.0))
    }

    /// The nearest sketch hit inside one of its closed regions — the area a
    /// viewer shades — or near one of its curves.
    fn pick_sketch(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
    ) -> Option<PartHit<S>> {
        self.sketches
            .iter()
            .filter(|sketch| {
                let entity = EntityRef::Sketch {
                    name: sketch.name.clone(),
                };
                accept(&entity, &[Role::Sketch])
            })
            .filter_map(|sketch| {
                let (t, p) = pointer.ray.intersect_uv_plane(&sketch.plane)?;
                let reach = pointer.reach_at(1.0, t);
                let in_region = sketch.regions.iter().any(|loops| loops_contain(loops, &p));
                let on_curve = || {
                    sketch.curves.iter().any(|c| {
                        c.polyline.windows(2).any(|w| {
                            !p.distance_to_segment(&w[0], &w[1])
                                .definitely_greater(reach)
                        })
                    })
                };
                (in_region || on_curve()).then(|| PartHit {
                    entity: EntityRef::Sketch {
                        name: sketch.name.clone(),
                    },
                    point: pointer.ray.at(t),
                    t,
                })
            })
            .min_by(|a, b| nearer(a.t, b.t))
    }

    /// The nearest datum other than a frame `accept`ed the ray hits, as the
    /// viewer draws it: a point at its origin, an axis as a line and a plane
    /// as a square, each [`Extent::size`] long around the point of it
    /// nearest the extent's center.
    fn pick_datum(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
    ) -> Option<PartHit<S>> {
        let Extent { center, size } = self.extent;
        let ray = &pointer.ray;
        let half = size.div(S::TWO).ok()?;
        self.datums
            .iter()
            .filter(|d| {
                d.kind != DatumKind::Frame
                    && accept(&EntityRef::datum(d.name.clone()), &[datum_role(d.kind)])
            })
            .filter_map(|d| {
                let f = &d.frame;
                let (origin, w) = (f.origin(), f.w());
                let t = match d.kind {
                    DatumKind::Point => {
                        let (dist, t) = ray.distance_to_point(origin);
                        pointer.within(dist, t, 1.0).then_some(t)?
                    }
                    DatumKind::Axis => {
                        let mid = origin.add(&w.prod_scalar(center.sub(origin).prod_dot(w)));
                        let reach = w.prod_scalar(size);
                        let (dist, t) = ray.distance_to_segment(&mid.sub(&reach), &mid.add(&reach));
                        pointer.within(dist, t, 1.0).then_some(t)?
                    }
                    DatumKind::Plane => {
                        let (t, point) = ray.intersect_plane(origin, w)?;
                        let d = point.sub(&center);
                        let inside =
                            |axis: &Vector3<S>| !d.prod_dot(axis).abs().definitely_greater(half);
                        (inside(f.u()) && inside(f.v())).then_some(t)?
                    }
                    DatumKind::Frame => unreachable!("frames are filtered out above"),
                };
                Some(PartHit {
                    entity: EntityRef::datum(d.name.clone()),
                    point: ray.at(t),
                    t,
                })
            })
            .min_by(|a, b| nearer(a.t, b.t))
    }

    /// The part of a frame datum `accept`ed the ray hits, the frame
    /// [`FRAME`] reaches tall: its origin ball — the frame as a whole, a
    /// point — else an axis (from a tenth of its length to its tip), else a
    /// plane's square (off the origin, between the other two axes). The
    /// smallest part under the pointer, as for the part's entities, and of
    /// the nearest frame.
    fn pick_frame(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
    ) -> Option<PartHit<S>> {
        let ray = &pointer.ray;
        let frac = |size: S, x: f64| size.mul(S::from_f64(x));
        let (mut balls, mut axes, mut planes) = (Vec::new(), Vec::new(), Vec::new());
        for d in self.datums.iter().filter(|d| d.kind == DatumKind::Frame) {
            let f = &d.frame;
            let origin = f.origin();
            let size = pointer.reach_at(FRAME, ray.closest_to_point(origin));
            if accept(&EntityRef::datum(d.name.clone()), &[Role::Point]) {
                let (dist, t) = ray.distance_to_point(origin);
                if !dist.definitely_greater(frac(size, 0.12)) {
                    balls.push(PartHit {
                        entity: EntityRef::datum(d.name.clone()),
                        point: *origin,
                        t,
                    });
                }
            }
            for axis in FrameAxis::ALL {
                let dir = axis.of(f);
                let component = |c| EntityRef::datum_component(d.name.clone(), c);
                if accept(&component(DatumComponent::Axis(axis)), &[Role::Line]) {
                    let (a, b) = (
                        origin.add(&dir.prod_scalar(frac(size, 0.1))),
                        origin.add(&dir.prod_scalar(size)),
                    );
                    let (dist, t) = ray.distance_to_segment(&a, &b);
                    if pointer.within(dist, t, 0.6) {
                        axes.push(PartHit {
                            entity: EntityRef::datum_component(
                                d.name.clone(),
                                DatumComponent::Axis(axis),
                            ),
                            point: b,
                            t,
                        });
                    }
                }
                if accept(&component(DatumComponent::Plane(axis)), &[Role::Plane])
                    && let Some((t, point)) = ray.intersect_plane(origin, &dir)
                {
                    let local = point.sub(origin);
                    let in_square =
                        FrameAxis::ALL
                            .iter()
                            .filter(|&&other| other != axis)
                            .all(|other| {
                                let x = local
                                    .prod_dot(&other.of(f))
                                    .div(size)
                                    .unwrap_or(S::INFINITY);
                                !x.sub(S::from_f64(0.45))
                                    .abs()
                                    .definitely_greater(S::from_f64(0.15))
                            });
                    if in_square {
                        planes.push(PartHit {
                            entity: EntityRef::datum_component(
                                d.name.clone(),
                                DatumComponent::Plane(axis),
                            ),
                            point,
                            t,
                        });
                    }
                }
            }
        }
        let nearest = |hits: Vec<PartHit<S>>| hits.into_iter().min_by(|a, b| nearer(a.t, b.t));
        nearest(balls)
            .or_else(|| nearest(axes))
            .or_else(|| nearest(planes))
    }
}

/// The role a datum of `kind` fills as a whole, other than a frame's.
fn datum_role(kind: DatumKind) -> Role {
    match kind {
        DatumKind::Point | DatumKind::Frame => Role::Point,
        DatumKind::Axis => Role::Line,
        DatumKind::Plane => Role::Plane,
    }
}
