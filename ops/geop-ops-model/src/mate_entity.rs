//! [`MateEntity`]: what a mate holds by — an element of the part itself, or
//! of a part placed in it, however deep — and the reference frame it gives
//! where it is put ([`MateEntity::frame`]): where it sits, and which way
//! its `z` axis runs.

use crate::{
    NamedModel, frame_along,
    placed::locate,
    selection::{Selected, Shape},
};
use geop_core_editor::editing::{Style, Visual};
use geop_core_editor::selectable::Selectable;
use geop_core_editor::view::{FramePick, drawn};
use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, min_subdivision_size},
    shape::Axis,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{element::PickedElement, reader::Reader};
use geop_core_topology::{CoedgeGeometry, EdgeId, FaceId};
use serde::{Deserialize, Serialize};

/// What a mate holds by: the element `on`, of the part itself or of a part
/// placed in it, however deep (see [`geop_core_program::placed`]) — and,
/// for a joint's frame, the element of the same part the frame sits at: a
/// corner or a side of a face, an end of an edge (see
/// [`MateEntity::frame`]). Saved as `{"on": {"face": {"target":
/// "/bolt:model", "name": "extrude(head,end)"}}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MateEntity {
    pub on: Selected,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at: Option<Selected>,
}

impl MateEntity {
    /// A frame on `on`, sitting where nothing else says.
    pub fn on(on: Selected) -> Self {
        Self { on, at: None }
    }

    /// A frame on `on`, of the part placed at `part`, as that part names it.
    pub fn of(part: &str, on: Selected) -> Self {
        Self::on(on.behind(part))
    }

    /// It with its frame put at `at`, of the same part as what it is on.
    pub fn at(self, at: Selected) -> Self {
        let at = at.local().behind(self.part());
        Self {
            at: Some(at),
            ..self
        }
    }

    /// The path of the placed part it is of: empty for the part itself.
    pub fn part(&self) -> &str {
        self.on.instance()
    }

    /// How it reads in a list: `bolt: face extrude(head,end)`.
    pub fn label(&self) -> String {
        let on = self.on.local().picked().label();
        match self.part() {
            "" => on,
            part => format!("{part}: {on}"),
        }
    }

    /// What it is in its own part — the part `reader` reads, or the one
    /// placed in it that it is of. Reads no state: where that part is
    /// placed, [`locate`](crate::placed::locate) says.
    pub fn shape(&self, reader: &Reader) -> GeopResult<Shape> {
        Ok(self.on.shape_in_instance(reader)?.element)
    }

    /// The surface of the wall the face it is on is of, in its own part —
    /// the part `reader` reads, or the one placed in it that it is of — its
    /// normal out of its solid: the face's, and those of the faces it runs
    /// on into smoothly, joined (see [`wall`]). A cam's side swept along a
    /// profile is a face per piece of the profile, and a follower rolls
    /// round all of them. None for any element but a face. Reads no state,
    /// as [`MateEntity::shape`] does not.
    pub fn surface(&self, reader: &Reader) -> GeopResult<Option<NurbSurface3D>> {
        let Selected::Face(face) = self.on.local() else {
            return Ok(None);
        };
        let ctx = with_context!("the surface of {}", self.label());
        let inside = match self.part() {
            "" => reader.clone(),
            part => reader.placed_part(part).with_context(ctx)?,
        };
        let found = inside.retrieve_element(&face).with_context(ctx)?;
        wall(found.model, found.id).map(Some).with_context(ctx)
    }

    /// The curve it runs along, in its own part — the part `reader` reads,
    /// or the one placed in it that it is of: of an edge, the edge's and
    /// those of the edges it runs on into smoothly at its ends, joined (see
    /// [`run`]): a slot's rim is an edge per piece of its outline, and a pin
    /// runs round all of them. A sketch's curve as it is; none for what is
    /// no curve. Reads no state.
    pub fn curve(&self, reader: &Reader) -> GeopResult<Option<NurbCurve3D>> {
        let ctx = with_context!("the curve of {}", self.label());
        let Selected::Edge(edge) = self.on.local() else {
            return Ok(self.shape(reader).with_context(ctx)?.curve);
        };
        let inside = match self.part() {
            "" => reader.clone(),
            part => reader.placed_part(part).with_context(ctx)?,
        };
        let found = inside.retrieve_element(&edge).with_context(ctx)?;
        run(found.model, found.id).map(Some).with_context(ctx)
    }

    /// The frame it gives in its own part — the part `reader` reads, or
    /// the one placed in it that it is of. Reads no state: where that part
    /// is placed, [`locate`](crate::placed::locate) says.
    ///
    /// Its `z` axis is the element's own, and what is turned about it, or
    /// slid along:
    ///
    /// - A planar face: its normal, out of its solid. `at` is where it sits
    ///   in the plane, else the middle of the face.
    /// - A round face — a cylinder, a cone: its axis. `at` is where along
    ///   it, as near as the axis is to it, else half way along the face.
    /// - A circular edge: the axis of its circle, at its center.
    /// - A straight edge: its direction. `at` is where along it — a vertex
    ///   at one end, say — else the middle.
    /// - A point: the world's axes, there.
    /// - A datum — a frame, an axis, a plane: its own.
    ///
    /// Where the frame sits is a free choice, so it is sharp.
    pub fn frame(&self, reader: &Reader) -> GeopResult<CoordinateSystem> {
        let ctx = with_context!("the frame on {}", self.label());
        let inside = match self.part() {
            "" => reader.clone(),
            part => reader.placed_part(part).with_context(ctx)?,
        };
        let at = self.at.as_ref().map(Selected::local);
        frame_on(&inside, &self.on.local(), at.as_ref()).with_context(ctx)
    }

    /// The frame it gives, where the part it is of is placed in the part
    /// `reader` reads — in the world, as the part is drawn.
    pub fn frame_in_world(&self, reader: &Reader) -> GeopResult<CoordinateSystem> {
        let frame = self.frame(reader)?;
        match self.part() {
            "" => Ok(frame),
            part => (locate(reader, part)?.world.motion())
                .apply_frame(&frame)
                .with_context(with_context!("the frame on {}", self.label())),
        }
    }
}

/// The surface of `face` of `named`, and of every face it runs on into
/// across an edge — and they on — where one surface carries on from the
/// other's side, tangent to it: joined into one along the way they run
/// (see [`NurbSurface3D::joined`]). The faces are taken in the order of
/// their ids, which a model gives in creation order.
fn wall(named: &NamedModel, face: FaceId) -> GeopResult<NurbSurface3D> {
    let model = named.topology();
    let mut wall = model.get_face(face)?.surface.clone();
    let mut used = vec![face];
    let mut along = None;
    loop {
        let mut next: Vec<FaceId> = (used.iter())
            .flat_map(|&f| model.face_edges(f).collect::<Vec<_>>())
            .filter_map(|edge| model.faces_of_edge(edge))
            .flatten()
            .filter(|f| !used.contains(f))
            .collect();
        next.sort();
        next.dedup();
        let joined = next.iter().find_map(|&f| {
            let surface = &model.get_face(f).ok()?.surface;
            let ways = along.map_or(vec![true, false], |along| vec![along]);
            ways.into_iter().find_map(|way| {
                let after = (runs_on(&wall, surface, way))
                    .then(|| wall.joined(surface, way).ok())
                    .flatten();
                let before = || {
                    (runs_on(surface, &wall, way))
                        .then(|| surface.joined(&wall, way).ok())
                        .flatten()
                };
                Some((f, way, after.or_else(before)?))
            })
        });
        let Some((f, way, joined)) = joined else {
            return Ok(wall);
        };
        (wall, along) = (joined, Some(way));
        used.push(f);
    }
}

/// The curve of `edge` of `named`, and of every edge it runs on into at
/// either end — and they on — tangent to it: joined into one (see
/// [`NurbCurve3D::joined`]), each turned to run on the way the run goes.
/// Edges are taken in the order of their ids.
fn run(named: &NamedModel, edge: EdgeId) -> GeopResult<NurbCurve3D> {
    let model = named.topology();
    let first = model.get_edge(edge)?;
    let mut run = first.curve.clone();
    let mut ends = [first.start_vertex, first.end_vertex];
    let mut used = vec![edge];
    loop {
        let mut grown = false;
        for at_end in [true, false] {
            let vertex = ends[usize::from(at_end)];
            let mut next = model.edges_at_vertex(vertex);
            next.retain(|e| !used.contains(e));
            next.sort();
            for e in next {
                let found = model.get_edge(e)?;
                // Turned to leave the vertex, or to arrive there.
                let leaves = found.start_vertex == vertex;
                let curve = match leaves == at_end {
                    true => found.curve.clone(),
                    false => found.curve.reverse(),
                };
                let far = if leaves {
                    found.end_vertex
                } else {
                    found.start_vertex
                };
                let joined = match at_end {
                    true => tangent(&run, &curve).then(|| run.joined(&curve).ok()),
                    false => tangent(&curve, &run).then(|| curve.joined(&run).ok()),
                };
                if let Some(Some(joined)) = joined {
                    run = joined;
                    ends[usize::from(at_end)] = far;
                    used.push(e);
                    grown = true;
                    break;
                }
            }
        }
        // Closed round: back where it started.
        if !grown || ends[0] == ends[1] {
            return Ok(run);
        }
    }
}

/// Whether `next` carries on from where `curve` ends, tangent to it: the
/// step from its first control point to its second along the step from
/// `curve`'s last but one to its last, as far as their enclosures can tell.
fn tangent(curve: &NurbCurve3D, next: &NurbCurve3D) -> bool {
    let point = |p: &geop_core_math::vector::Vector<4>| {
        Vector3::from_array([0, 1, 2].map(|k| p[k].div(p[3]).unwrap_or(Scalar::ZERO)))
    };
    let (a, b) = (&curve.control_points, &next.control_points);
    if a.len() < 2 || b.len() < 2 {
        return false;
    }
    let (end, before) = (point(&a[a.len() - 1]), point(&a[a.len() - 2]));
    let (start, after) = (point(&b[0]), point(&b[1]));
    let (into, out) = (end.sub(&before), after.sub(&start));
    end.could_be_equal(&start)
        && into.prod_cross(&out).norm_sq().could_be_equal(Scalar::ZERO)
        && into.prod_dot(&out).definitely_greater(Scalar::ZERO)
}

/// Whether `next` carries on from where `surface` ends along `u` — along
/// `v`, if not `along_u` — tangent to it: at every control point of the row
/// they share, the step to the next one on is along the step from the one
/// before, as far as the points' enclosures can tell.
fn runs_on(surface: &NurbSurface3D, next: &NurbSurface3D, along_u: bool) -> bool {
    let point = |s: &NurbSurface3D, i: usize, j: usize| {
        let p = match along_u {
            true => s.control_points[i * s.num_v + j],
            false => s.control_points[j * s.num_v + i],
        };
        let w = p[3];
        Vector3::from_array([0, 1, 2].map(|k| p[k].div(w).unwrap_or(Scalar::ZERO)))
    };
    let count = |s: &NurbSurface3D| match along_u {
        true => (s.num_u, s.num_v),
        false => (s.num_v, s.num_u),
    };
    let ((n, m), (n2, m2)) = (count(surface), count(next));
    if m != m2 || n < 2 || n2 < 2 {
        return false;
    }
    (0..m).all(|j| {
        let (end, before) = (point(surface, n - 1, j), point(surface, n - 2, j));
        let (start, after) = (point(next, 0, j), point(next, 1, j));
        let (into, out) = (end.sub(&before), after.sub(&start));
        end.could_be_equal(&start)
            && into.prod_cross(&out).norm_sq().could_be_equal(Scalar::ZERO)
            && into.prod_dot(&out).definitely_greater(Scalar::ZERO)
    })
}

/// The frame `on` gives in the part `reader` reads, put at `at` (see
/// [`MateEntity::frame`]).
fn frame_on(reader: &Reader, on: &Selected, at: Option<&Selected>) -> GeopResult<CoordinateSystem> {
    let shape = on.shape(reader)?;
    if let Some(frame) = shape.frame {
        return Ok(frame);
    }
    if let Some(arc) = &shape.arc {
        return frame_along(arc.circle.center.sharpen(), &arc.circle.normal);
    }
    let anchor = at.map(|at| middle_of(reader, at)).transpose()?;
    // Where on `on` the frame sits, unless `at` says: only worked out where
    // needed, since not every element has a middle.
    let near = || match anchor {
        Some(anchor) => Ok(anchor),
        None => middle_of(reader, on),
    };
    let along = |axis: &Axis| -> GeopResult<CoordinateSystem> {
        frame_along(axis.project(&near()?).sharpen(), &axis.direction)
    };
    if let Some(plane) = &shape.plane {
        let near = match (anchor, on) {
            (Some(anchor), _) => anchor,
            (None, Selected::Face(_)) => middle_of(reader, on)?,
            (None, _) => *plane.origin(),
        };
        let off = near.sub(plane.origin()).prod_dot(plane.w());
        let origin = near.sub(&plane.w().prod_scalar(off)).sharpen();
        return CoordinateSystem::try_new(origin, *plane.u(), *plane.v(), *plane.w());
    }
    if let Some(axis) = &shape.round {
        return along(axis);
    }
    if let Some(axis) = &shape.line {
        return along(axis);
    }
    if let Some(point) = shape.point {
        return Ok(CoordinateSystem::world_at(point));
    }
    Err(GeopError::new(
        "it gives no frame: pick a face, an edge, a point or a datum",
    ))
}

/// Where an element is in the middle of itself, in the part `reader`
/// reads: a vertex's point, the center of a circular edge, the middle of
/// any other edge — of its parameter, not its length — and of a face the
/// middle of the box round its edges, in its own surface. Where a frame
/// sits when it sits nowhere else: a free choice, so sharp.
fn middle_of(reader: &Reader, element: &Selected) -> GeopResult<Vector3> {
    let ctx = with_context!("the middle of {}", element.picked().label());
    if let Selected::Face(face) = element {
        let found = reader.retrieve_element(face).with_context(ctx)?;
        return face_middle(found.model, found.id).with_context(ctx);
    }
    let shape = element.shape(reader).with_context(ctx)?;
    if let Some(arc) = &shape.arc {
        return Ok(arc.circle.center.sharpen());
    }
    if let Some(curve) = &shape.curve {
        let t = curve.mid_parameter().with_context(ctx)?;
        return Ok(curve.evaluate(t).with_context(ctx)?.sharpen());
    }
    (shape.point.map(|p| p.sharpen()))
        .ok_or_else(|| GeopError::new("it has no middle"))
        .with_context(ctx)
}

/// The middle of the box round the edges of `face` — each edge's own
/// extremes, not its control points' — or the point of a bare vertex face.
/// In the part's own frame.
fn face_middle(named: &NamedModel, face: FaceId) -> GeopResult<Vector3> {
    let model = named.topology();
    let mut bounds: Option<[Scalar; 3]> = None;
    for coedge in model.iterate_face_coedges(face) {
        let spans = match model.get_coedge(coedge)?.geometry {
            CoedgeGeometry::Edge(edge) => model
                .get_edge(edge)?
                .curve
                .bounds(MAX_NODES, min_subdivision_size())?,
            CoedgeGeometry::Vertex(vertex) => model.get_vertex(vertex)?.point.to_array(),
        };
        bounds = Some(match bounds {
            Some(b) => std::array::from_fn(|k| b[k].union(spans[k])),
            None => spans,
        });
    }
    let bounds =
        bounds.ok_or_else(|| GeopError::new(format!("face {face} has no edge to measure")))?;
    Ok(Vector3::from_array(bounds.map(|axis| axis.midpoint())))
}

/// A mate's element is picked from what the editor draws of the part and
/// the parts placed in it; a frame, with where it sits (see
/// [`drawn::FRAME`]). Its element is held as the pick names it, of the
/// placed part behind its path.
impl Selectable for MateEntity {
    fn picked_from() -> Vec<&'static str> {
        let mut kinds = Selected::picked_from();
        kinds.push(drawn::FRAME);
        kinds
    }

    fn from_picked(picked: &PickedElement) -> Option<Self> {
        if let Some(frame) = FramePick::of(picked) {
            let at = match &frame.at {
                Some(at) => Some(Selected::from_picked(at)?),
                None => None,
            };
            return Some(MateEntity {
                on: Selected::from_picked(&frame.on)?,
                at,
            });
        }
        Selected::from_picked(picked).map(MateEntity::on)
    }

    fn picked(&self) -> PickedElement {
        let on = self.on.picked();
        match &self.at {
            None => on,
            Some(at) => FramePick {
                on,
                at: Some(at.picked()),
            }
            .picked(),
        }
    }

    fn select(reader: &Reader, hit: &PickedElement) -> Option<Self> {
        let _ = reader;
        // What a click hits is held as it is named: whether it is what a
        // mate needs is said by the mate, against the part it is of.
        Self::from_picked(hit)
    }

    fn drawn(&self, reader: &Reader) -> Vec<PickedElement> {
        let _ = reader;
        let mut drawn = vec![drawn_of(&self.on)];
        drawn.extend(self.at.iter().map(drawn_of));
        drawn
    }

    /// The frame it gives, as a triad where it is drawn: nothing for an
    /// element that gives none, which the mate says of it when it is held.
    fn visuals(&self, reader: &Reader) -> Vec<Visual> {
        let Ok(frame) = self.frame_in_world(reader) else {
            return Vec::new();
        };
        let shape = geop_core_editor::editing::Shape::Triad {
            at: *frame.origin(),
            axes: [*frame.u(), *frame.v(), *frame.w()],
        };
        vec![Visual::new("frame", shape, Style::Selected)]
    }
}

/// What the editor draws of `element`, by the kind it is drawn as.
fn drawn_of(element: &Selected) -> PickedElement {
    let picked = element.picked();
    let kind = match element {
        Selected::Vertex(_) => drawn::VERTEX,
        Selected::Edge(_) => drawn::EDGE,
        Selected::Face(_) => drawn::FACE,
        Selected::Datum(_) => drawn::DATUM,
        Selected::SketchPoint(_) => drawn::SKETCH_POINT,
        Selected::SketchCurve(_) => drawn::SKETCH_CURVE,
    };
    PickedElement {
        kind: kind.to_string(),
        ..picked
    }
}
