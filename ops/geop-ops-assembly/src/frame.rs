//! [`MateEntity`]: what a mate holds by — an element of the part itself, or
//! of a part placed in it, however deep — and the reference frame it gives
//! where it is put ([`MateEntity::frame`]): where it sits, and which way
//! its `z` axis runs.

use geop_core_editor::selectable::Selectable;
use geop_core_editor::view::{FramePick, drawn};
use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{element::PickedElement, reader::Reader, values::Design};
use geop_core_topology::{CoedgeGeometry, FaceId};
use geop_ops_model::{NamedModel, frame_along};
use geop_ops_reference_geometry::{Selected, Shape};
use serde::{Deserialize, Serialize};

/// How many points of each of a face's edges its middle is measured from.
const SAMPLES: i64 = 8;

/// What a mate holds by: the element `on`, of the part itself or of a part
/// placed in it, however deep (see [`geop_core_program::instance`]) — and,
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
    pub fn frame(&self, reader: &Reader) -> GeopResult<CoordinateSystem<Design>> {
        let ctx = with_context!("the frame on {}", self.label());
        let inside = match self.part() {
            "" => reader.clone(),
            part => reader.placed_part(part).with_context(ctx)?,
        };
        let at = self.at.as_ref().map(Selected::local);
        frame_on(&inside, &self.on.local(), at.as_ref()).with_context(ctx)
    }
}

/// The frame `on` gives in the part `reader` reads, put at `at` (see
/// [`MateEntity::frame`]).
fn frame_on(
    reader: &Reader,
    on: &Selected,
    at: Option<&Selected>,
) -> GeopResult<CoordinateSystem<Design>> {
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
    let along = |axis: &Axis<Design>| -> GeopResult<CoordinateSystem<Design>> {
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
fn middle_of(reader: &Reader, element: &Selected) -> GeopResult<Vector3<Design>> {
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

/// The middle of the box round the edges of `face`, sampled: the point of a
/// bare vertex face. In the part's own frame.
fn face_middle<S: Scalar>(named: &NamedModel<S>, face: FaceId) -> GeopResult<Vector3<S>> {
    let model = named.topology();
    let mut points = Vec::new();
    for coedge in model.iterate_face_coedges(face) {
        match model.get_coedge(coedge)?.geometry {
            CoedgeGeometry::Edge(edge) => {
                let curve = &model.get_edge(edge)?.curve;
                let (t0, t1) = curve.domain();
                for k in 0..=SAMPLES {
                    let t = S::interpolate(t0, t1, S::from_ratio(k, SAMPLES)?);
                    points.push(curve.evaluate(t)?);
                }
            }
            CoedgeGeometry::Vertex(vertex) => points.push(model.get_vertex(vertex)?.point),
        }
    }
    let [first, rest @ ..] = points.as_slice() else {
        return Err(GeopError::new(format!(
            "face {face} has no edge to measure"
        )));
    };
    let (lo, hi) = rest.iter().fold((*first, *first), |(lo, hi), p| {
        (
            Vector3::from_array([0, 1, 2].map(|k| lo[k].min(p[k]))),
            Vector3::from_array([0, 1, 2].map(|k| hi[k].max(p[k]))),
        )
    });
    Ok(lo.add(&hi).prod_scalar(S::ONE.div(S::TWO)?).sharpen())
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
