//! [`MateEntity`]: what a mate holds by — an element of the part itself, or
//! of a part placed in it, however deep — and the reference frame it gives
//! where it is put ([`MateEntity::frame`]): where it sits, and which way
//! its `z` axis runs.

use geop_core_editor::selectable::Selectable;
use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    element::PickedElement,
    part::{Input, Reader, State},
};
use geop_core_topology::{CoedgeGeometry, FaceId};
use geop_ops_add_datum::{Selected, Shape};
use geop_ops_model::{Design, NamedModel, frame_along};
use geop_ops_parameters::PartReader;
use geop_ops_view::{FramePick, INSTANCE_SEPARATOR, drawn};
use serde::{Deserialize, Serialize};

use crate::placed::{component_at, instance_path};

/// How many points of each of a face's edges its middle is measured from.
const SAMPLES: i64 = 8;

/// An element of the part, or of a part placed in it however deep: the
/// element `on` of the part placed at `part` — `bolt`, or `asm/bolt` for
/// one placed in a placed part; the part itself, for none. Picked as the
/// element itself, never a frame on it. Saved as `{"part": "bolt", "on":
/// {"vertex": "extrude(head,p3,end)"}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlacedElement {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub part: String,
    pub on: Selected,
}

impl PlacedElement {
    /// `on`, of the part itself.
    pub fn own(on: Selected) -> Self {
        Self::of("", on)
    }

    /// `on`, of the part placed at `part`.
    pub fn of(part: impl Into<String>, on: Selected) -> Self {
        Self {
            part: part.into(),
            on,
        }
    }

    /// How it reads in a list: `bolt: face extrude(head,end)`.
    pub fn label(&self) -> String {
        let on = self.on.picked().label();
        match self.part.is_empty() {
            true => on,
            false => format!("{}: {on}", self.part),
        }
    }

    /// What it is in its own part, placed in the part `reader` reads — see
    /// [`Shape`]. Reads no state: where that part is placed, [`locate`]
    /// says.
    pub fn shape(&self, reader: &PartReader) -> GeopResult<Shape> {
        self.on.shape(&component_at(reader, &self.part)?)
    }
}

/// An element is picked from what the editor draws of the part and the
/// parts placed in it, and held as the pick names it, of the placed part
/// behind its path.
impl Selectable for PlacedElement {
    fn picked_from() -> Vec<&'static str> {
        Selected::picked_from()
    }

    fn from_picked(picked: &PickedElement) -> Option<Self> {
        let (part, inner) = inside(picked);
        Some(PlacedElement::of(part, Selected::from_picked(&inner)?))
    }

    fn picked(&self) -> PickedElement {
        outside(&self.part, self.on.picked())
    }

    fn select<I: Input, S: State>(reader: &Reader<I, S>, hit: &PickedElement) -> Option<Self> {
        let _ = reader;
        // What a click hits is held as it is named: whether it is what is
        // needed is said by what holds it, against the part it is of.
        Self::from_picked(hit)
    }

    fn drawn<I: Input, S: State>(&self, reader: &Reader<I, S>) -> Vec<PickedElement> {
        let _ = reader;
        vec![outside(&self.part, drawn_of(&self.on))]
    }
}

/// A frame on an element: its element's, sitting where nothing else says.
impl From<PlacedElement> for MateEntity {
    fn from(element: PlacedElement) -> Self {
        MateEntity::of(element.part, element.on)
    }
}

/// What a mate holds by: the element `on` of the part placed at `part` —
/// `bolt`, or `asm/bolt` for one placed in a placed part; the part itself,
/// for none — and, for a joint's frame, the element of it the frame sits
/// at: a corner or a side of a face, an end of an edge (see
/// [`MateEntity::frame`]). Saved as `{"part": "bolt", "on": {"face":
/// "extrude(head,end)"}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MateEntity {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub part: String,
    pub on: Selected,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at: Option<Selected>,
}

impl MateEntity {
    /// `on`, of the part itself.
    pub fn own(on: Selected) -> Self {
        Self {
            part: String::new(),
            on,
            at: None,
        }
    }

    /// `on`, of the part placed at `part`.
    pub fn of(part: impl Into<String>, on: Selected) -> Self {
        Self {
            part: part.into(),
            on,
            at: None,
        }
    }

    /// It with its frame put at `at`.
    pub fn at(self, at: Selected) -> Self {
        Self {
            at: Some(at),
            ..self
        }
    }

    /// It of the part placed as `instance` in the part it is of now: what a
    /// mate of a placed part is to the part placing it.
    pub fn behind(&self, instance: &str) -> Self {
        Self {
            part: match self.part.is_empty() {
                true => instance.to_string(),
                false => instance_path(instance, &self.part),
            },
            ..self.clone()
        }
    }

    /// The element it is on, of its part.
    pub fn element(&self) -> PlacedElement {
        PlacedElement::of(self.part.clone(), self.on.clone())
    }

    /// How it reads in a list: `bolt: face extrude(head,end)`.
    pub fn label(&self) -> String {
        self.element().label()
    }

    /// What it is in its own part, placed in the part `reader` reads — see
    /// [`PlacedElement::shape`].
    pub fn shape(&self, reader: &PartReader) -> GeopResult<Shape> {
        self.element().shape(reader)
    }

    /// The frame it gives in its own part, placed in the part `reader`
    /// reads. Reads no state: where that part is placed, [`locate`](crate::placed::locate) says.
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
    pub fn frame(&self, reader: &PartReader) -> GeopResult<CoordinateSystem<Design>> {
        let ctx = with_context!("the frame on {}", self.label());
        let inside = component_at(reader, &self.part).with_context(ctx)?;
        frame_on(&inside, &self.on, self.at.as_ref()).with_context(ctx)
    }
}

/// The frame `on` gives in the part `reader` reads, put at `at` (see
/// [`MateEntity::frame`]).
fn frame_on(
    reader: &PartReader,
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
fn middle_of(reader: &PartReader, element: &Selected) -> GeopResult<Vector3<Design>> {
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

/// The part path and the target of the part placed there that `target`, a
/// target of the part read as a pick names it, is: `("bolt", "model")` for
/// `bolt/model`, `("", "model")` for the part's own.
fn split_target(target: &str) -> (&str, &str) {
    target.rsplit_once(INSTANCE_SEPARATOR).unwrap_or(("", target))
}

/// `picked` with its target as the part placed at `part` names it.
fn inside(picked: &PickedElement) -> (String, PickedElement) {
    let (part, target) = split_target(&picked.target);
    let inner = PickedElement {
        target: target.to_string(),
        ..picked.clone()
    };
    (part.to_string(), inner)
}

/// `picked`, of the part placed at `part`, as the part read names it.
fn outside(part: &str, picked: PickedElement) -> PickedElement {
    match part.is_empty() {
        true => picked,
        false => PickedElement {
            target: format!("{part}{INSTANCE_SEPARATOR}{}", picked.target),
            ..picked
        },
    }
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
            let (part, on) = inside(&frame.on);
            let at = match &frame.at {
                Some(at) => Some(Selected::from_picked(&inside(at).1)?),
                None => None,
            };
            return Some(MateEntity {
                part,
                on: Selected::from_picked(&on)?,
                at,
            });
        }
        PlacedElement::from_picked(picked).map(MateEntity::from)
    }

    fn picked(&self) -> PickedElement {
        let on = self.element().picked();
        match &self.at {
            None => on,
            Some(at) => FramePick {
                on,
                at: Some(outside(&self.part, at.picked())),
            }
            .picked(),
        }
    }

    fn select<I: Input, S: State>(reader: &Reader<I, S>, hit: &PickedElement) -> Option<Self> {
        let _ = reader;
        // What a click hits is held as it is named: whether it is what a
        // mate needs is said by the mate, against the part it is of.
        Self::from_picked(hit)
    }

    fn drawn<I: Input, S: State>(&self, reader: &Reader<I, S>) -> Vec<PickedElement> {
        let mut drawn = self.element().drawn(reader);
        drawn.extend((self.at.iter()).map(|at| outside(&self.part, drawn_of(at))));
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
