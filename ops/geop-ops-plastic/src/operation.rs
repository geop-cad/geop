//! The plastic features as operations of a program: [`Rib`], [`Lip`],
//! [`Groove`] and [`Draft`].

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Choice, Dialog, Event, Form, Number, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_core_sketch::Shape;
use geop_core_topology::{EdgeId, FaceId};
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{
        Build, BuildReader, EntityRef, Input, Lookup, ModelStep, ResolveEntity, Role, Scope, Seen,
        build_model,
    },
};
use geop_ops_extrude_revolve::operation::shape_loops;
use serde::{Deserialize, Serialize};

use crate::{
    draft::draft,
    lip::{LipSize, groove, lip},
    rib::{Growth, rib},
};

/// How the plastic features are offered: rarely used, and not yet robust
/// enough to offer by default.
const fn info(kind: &'static str, label: &'static str, doc: &'static str) -> OperationInfo {
    OperationInfo::new(
        kind,
        label,
        doc,
        OperationGroup::Features,
        OperationTier::Menu,
    )
    .experimental()
}

/// The one entity `name` stands for, as a reference field holds it: none
/// for an empty name.
fn one(kind: fn(String) -> EntityRef, name: &str) -> Vec<EntityRef> {
    EntityRef::of_names(kind, std::slice::from_ref(&name.to_string()))
}

/// The last of the entities of `kind` picked, by name: empty if none.
fn picked_one(kind: fn(String) -> EntityRef, picked: &[EntityRef]) -> String {
    EntityRef::names_of(kind, picked).pop().unwrap_or_default()
}

/// Fails unless a size typed in is a number.
fn finite(what: &str, value: f64) -> GeopResult<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(GeopError::new(format!("the {what} is not a number")))
    }
}

/// Tilts the planar faces `faces` of a solid by `angle` degrees about the
/// plane `neutral`, so the part comes out of a mould pulled along the
/// neutral plane's normal — into the solid, for a face of it picked as the
/// neutral plane — or the other way if `reversed`: past the neutral plane,
/// that way, the solid gets narrower. The solid is consumed and the result
/// named `draft(D)` for the operation `D`; every face, edge and vertex keeps
/// its name (see [`crate::draft::draft`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Draft;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DraftArgs {
    /// The faces to tilt, all planar, all of one solid.
    pub faces: Vec<String>,
    /// The plane they tilt about: a planar face, a datum plane.
    pub neutral: Option<EntityRef>,
    /// How far they tilt, in degrees.
    pub angle: f64,
    /// Pull the other way.
    #[serde(default)]
    pub reversed: bool,
}

impl ModelStep for Draft {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &DraftArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("draft({step}, {args:?})");
        let namer = Namer::new("draft", step)?;
        let faces = (args.faces.iter())
            .map(|name| model.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        let Some(neutral) = &args.neutral else {
            return Err(GeopError::new("pick the neutral plane")).with_context(ctx);
        };
        let frame = (neutral.resolve_plane(&mut Scope { model, reader })).with_context(ctx)?;
        // A face's frame points out of its solid; the pull goes into it.
        let into_solid = matches!(neutral, EntityRef::Face { .. });
        let pull = if into_solid != args.reversed {
            frame.w().neg()
        } else {
            *frame.w()
        };
        let plane = Plane::try_new(*frame.origin(), pull).with_context(ctx)?;
        finite("angle", args.angle).with_context(ctx)?;
        let angle = Design::from_f64(args.angle.to_radians());
        draft(model, &namer, &faces, &plane, angle).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Draft {
    const INFO: OperationInfo = info(
        "draft",
        "Draft",
        "Tilt planar faces about a neutral plane, so the part comes out of its mould.",
    );
    type Args = DraftArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// No faces yet, no neutral plane, and three degrees.
    fn new_args(_: &Context<Input, ParamValue>) -> DraftArgs {
        DraftArgs {
            faces: Vec::new(),
            neutral: None,
            angle: 3.0,
            reversed: false,
        }
    }

    fn run(step: &str, part: &mut Build, args: &DraftArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The faces and the neutral plane, picked, the angle and the pull
    /// direction.
    fn presentation(_: &Context<Input, ParamValue>, args: &DraftArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let faces = EntityRef::of_names(EntityRef::face, &args.faces);
        dialog.reference("faces", "faces", faces, &[Role::Plane], None, true);
        let neutral = args.neutral.iter().cloned().collect();
        dialog.reference(
            "neutral",
            "neutral plane",
            neutral,
            &[Role::Plane],
            None,
            false,
        );
        let angle = Number::new("angle", args.angle, Unit::Angle).range(-45.0, 45.0);
        (dialog.number("angle", angle)).checkbox(
            "reversed",
            "reverse pull direction",
            args.reversed,
        );
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &DraftArgs,
        _: &(),
        event: Event<Design>,
    ) -> (DraftArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("faces", Value::Entities(picked)) => {
                args.faces = EntityRef::names_of(EntityRef::face, &picked);
            }
            ("neutral", Value::Entities(picked)) => args.neutral = picked.into_iter().next(),
            ("angle", Value::Number(angle)) => args.angle = angle,
            ("reversed", Value::Bool(b)) => args.reversed = b,
            _ => {}
        }
        (args, ())
    }
}

/// The rim face and its edges, as the fields of a lip or a groove hold
/// them.
fn rim_fields<S: Scalar>(dialog: &mut Dialog<S>, face: &str, edges: &[String]) {
    let face = one(EntityRef::face, face);
    dialog.reference("face", "rim face", face, &[Role::Plane], None, false);
    let edges = EntityRef::of_names(EntityRef::edge, edges);
    (dialog.reference("edges", "edges", edges, &[Role::Edge], None, true)).optional("edges");
}

/// The rim face `face` and its edges `edges` after the field `key` is set
/// to `value`: the edges picked go with the face they were picked on.
/// Whether `key` is a rim field.
fn set_rim(face: &mut String, edges: &mut Vec<String>, key: &str, value: &Value) -> bool {
    match (key, value) {
        ("face", Value::Entities(picked)) => {
            let name = picked_one(EntityRef::face, picked);
            if name != *face {
                edges.clear();
            }
            *face = name;
            true
        }
        ("edges", Value::Entities(picked)) => {
            *edges = EntityRef::names_of(EntityRef::edge, picked);
            true
        }
        _ => false,
    }
}

/// The rim face and edges named, resolved.
fn resolve_rim<S: Scalar>(
    model: &NamedModel<S>,
    face: &str,
    edges: &[String],
) -> GeopResult<(FaceId, Vec<EdgeId>)> {
    if face.is_empty() {
        return Err(GeopError::new("pick the rim face"));
    }
    Ok((
        model.face_id(face)?,
        (edges.iter())
            .map(|name| model.edge_id(name))
            .collect::<GeopResult<_>>()?,
    ))
}

/// Raises a lip `width` wide and `height` high along the edges `edges` of
/// the planar rim face `face` — all along its hole if none are picked: the
/// inside of a shelled enclosure's rim — flush with the wall they are on
/// and joined to the solid (see [`crate::lip::lip`]), for the operation
/// `L`: the result is named `lip(L)`, the lip's side on the wall along an
/// edge `E` `lip(L,E)`, its side on the rim `lip(L,E,far)`, its top
/// `lip(L,end)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Lip;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LipArgs {
    /// The rim: a planar face.
    pub face: String,
    /// Edges of the rim to run along, one chain; none for all of its hole.
    #[serde(default)]
    pub edges: Vec<String>,
    pub width: f64,
    pub height: f64,
}

impl ModelStep for Lip {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &LipArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("lip({step}, {args:?})");
        let namer = Namer::new("lip", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges).with_context(ctx)?;
        finite("width", args.width).with_context(ctx)?;
        finite("height", args.height).with_context(ctx)?;
        let size = LipSize {
            width: Design::from_f64(args.width),
            height: Design::from_f64(args.height),
        };
        lip(model, &namer, step, face, &edges, size).with_context(ctx)
    }
}

impl Operation for Lip {
    const INFO: OperationInfo = info(
        "lip",
        "Lip",
        "Raise a lip along the rim of one half of an enclosure.",
    );
    type Args = LipArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// No rim yet, a lip 0.05 wide and 0.1 high.
    fn new_args(_: &Context<Input, ParamValue>) -> LipArgs {
        LipArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
        }
    }

    fn run(step: &str, part: &mut Build, args: &LipArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The rim and its edges, picked, the width and the height.
    fn presentation(_: &Context<Input, ParamValue>, args: &LipArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        rim_fields(dialog, &args.face, &args.edges);
        let length = |label: &str, value: f64| Number::new(label, value, Unit::Length);
        (dialog.number("width", length("width", args.width).range(0.0, 1.0)))
            .number("height", length("height", args.height).range(0.0, 1.0));
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &LipArgs,
        _: &(),
        event: Event<Design>,
    ) -> (LipArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        if !set_rim(&mut args.face, &mut args.edges, &key, &value) {
            match (key.as_str(), value) {
                ("width", Value::Number(w)) => args.width = w,
                ("height", Value::Number(h)) => args.height = h,
                _ => {}
            }
        }
        (args, ())
    }
}

/// Cuts the groove that takes a [`Lip`] `width` wide and `height` high,
/// `clearance` wider and deeper, along the edges `edges` of the planar rim
/// face `face` — all along its hole if none are picked — from the solid
/// (see [`crate::lip::groove`]), for the operation `G`: the result is named
/// `groove(G)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Groove;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GrooveArgs {
    /// The rim: a planar face.
    pub face: String,
    /// Edges of the rim to run along, one chain; none for all of its hole.
    #[serde(default)]
    pub edges: Vec<String>,
    /// The lip's width.
    pub width: f64,
    /// The lip's height.
    pub height: f64,
    /// How much wider and deeper the groove is than the lip.
    pub clearance: f64,
}

impl ModelStep for Groove {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &GrooveArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("groove({step}, {args:?})");
        let namer = Namer::new("groove", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges).with_context(ctx)?;
        finite("width", args.width).with_context(ctx)?;
        finite("height", args.height).with_context(ctx)?;
        finite("clearance", args.clearance).with_context(ctx)?;
        let size = LipSize {
            width: Design::from_f64(args.width),
            height: Design::from_f64(args.height),
        };
        let clearance = Design::from_f64(args.clearance);
        groove(model, &namer, step, face, &edges, size, clearance).with_context(ctx)
    }
}

impl Operation for Groove {
    const INFO: OperationInfo = info(
        "groove",
        "Groove",
        "Cut the groove that takes a lip into the rim of the other half.",
    );
    type Args = GrooveArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// No rim yet, for a lip 0.05 wide and 0.1 high, 0.01 clearance.
    fn new_args(_: &Context<Input, ParamValue>) -> GrooveArgs {
        GrooveArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
            clearance: 0.01,
        }
    }

    fn run(step: &str, part: &mut Build, args: &GrooveArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The rim and its edges, picked, the lip's width and height, and the
    /// clearance.
    fn presentation(_: &Context<Input, ParamValue>, args: &GrooveArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        rim_fields(dialog, &args.face, &args.edges);
        let length = |label: &str, value: f64| Number::new(label, value, Unit::Length);
        (dialog.number("width", length("lip width", args.width).range(0.0, 1.0)))
            .number("height", length("lip height", args.height).range(0.0, 1.0))
            .number(
                "clearance",
                length("clearance", args.clearance).range(0.0, 0.1),
            );
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &GrooveArgs,
        _: &(),
        event: Event<Design>,
    ) -> (GrooveArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        if !set_rim(&mut args.face, &mut args.edges, &key, &value) {
            match (key.as_str(), value) {
                ("width", Value::Number(w)) => args.width = w,
                ("height", Value::Number(h)) => args.height = h,
                ("clearance", Value::Number(c)) => args.clearance = c,
                _ => {}
            }
        }
        (args, ())
    }
}

/// Which side of its profile a rib is thick on: both alike, its first side
/// — to the profile's left, growing normal to the sketch; along the
/// sketch plane's normal, growing parallel to it — or its second.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RibSide {
    #[default]
    Symmetric,
    First,
    Second,
}

impl RibSide {
    const ALL: [(RibSide, &'static str, &'static str); 3] = [
        (RibSide::Symmetric, "symmetric", "Symmetric"),
        (RibSide::First, "first", "First side"),
        (RibSide::Second, "second", "Second side"),
    ];
}

/// Which way a rib grows from its profile: along the sketch plane's normal,
/// or in the sketch plane, square to the chord between the profile's ends.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RibDirection {
    #[default]
    Parallel,
    Normal,
}

impl RibDirection {
    const ALL: [(RibDirection, &'static str, &'static str); 2] = [
        (RibDirection::Parallel, "parallel", "Parallel to sketch"),
        (RibDirection::Normal, "normal", "Normal to sketch"),
    ];
}

/// The value `options` give `chosen`, and the choices they offer.
fn choices<T: PartialEq>(options: &[(T, &str, &str)], chosen: &T) -> (String, Vec<Choice>) {
    let value = (options.iter().find(|o| o.0 == *chosen)).map_or("", |o| o.1);
    let choices = (options.iter()).map(|&(_, v, l)| Choice::new(v, l));
    (value.to_string(), choices.collect())
}

/// What of `options` has the value `value`.
fn chosen<T: Copy>(options: &[(T, &str, &str)], value: &str) -> Option<T> {
    options.iter().find(|o| o.1 == value).map(|o| o.0)
}

/// Grows a rib `thickness` thick from the open profile of a sketch — lines
/// and arcs — until it meets the solid `solid`, its ends run on into the
/// walls they point at, and joins it (see [`crate::rib::rib`]), for the
/// operation `R`: the result is named `rib(R)`. Growing normal to the
/// sketch, the rib's caps are `rib(R,start)` on the sketch's plane; its
/// sides along a profile piece `K,X` are `rib(R,K,X)` and `rib(R,K,X,far)`.
/// Parallel, its sides on either side of the sketch's plane are
/// `rib(R,start)` and `rib(R,end)`, its edge along the profile `rib(R,K,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Rib;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RibArgs {
    /// The sketch of the profile: one open chain.
    pub sketch: String,
    /// The solid it grows up to and joins.
    pub solid: String,
    pub thickness: f64,
    #[serde(default)]
    pub side: RibSide,
    #[serde(default)]
    pub direction: RibDirection,
    /// Grow the other way.
    #[serde(default)]
    pub flipped: bool,
}

impl ModelStep for Rib {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &RibArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("rib({step}, {args:?})");
        let namer = Namer::new("rib", step)?;
        let placed = (Scope { model, reader }.sketch(&args.sketch))
            .with_context(ctx)?
            .clone();
        let target = model.solid_id(&args.solid).with_context(ctx)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<Design>().with_context(ctx)?;
        let chain = match sketch.shape().with_context(ctx)? {
            chain @ Shape::Chain(_) => chain,
            Shape::Region(_) => {
                return Err(GeopError::new(format!(
                    "sketch {:?} encloses an area: a rib grows from an open profile",
                    args.sketch
                )))
                .with_context(ctx);
            }
        };
        let profile = (shape_loops(&args.sketch, sketch, &geometry, chain))
            .with_context(ctx)?
            .remove(0)
            .profile;
        finite("thickness", args.thickness).with_context(ctx)?;
        let t = args.thickness;
        let (near, far) = match args.side {
            RibSide::Symmetric => (-t / 2.0, t / 2.0),
            RibSide::First => (0.0, t),
            RibSide::Second => (-t, 0.0),
        };
        let thickness = (Design::from_f64(near), Design::from_f64(far));
        let growth = match args.direction {
            RibDirection::Normal => Growth::Normal {
                flipped: args.flipped,
            },
            RibDirection::Parallel => Growth::Parallel {
                flipped: args.flipped,
            },
        };
        rib(
            model,
            &namer,
            step,
            &placed.plane,
            &profile,
            target,
            thickness,
            growth,
        )
        .with_context(ctx)
    }
}

impl Operation for Rib {
    const INFO: OperationInfo = info(
        "rib",
        "Rib",
        "Grow a thin wall from an open sketch profile up to a solid's faces, and join it.",
    );
    type Args = RibArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest sketch and solid, 0.05 thick, symmetric, parallel to the
    /// sketch.
    fn new_args(context: &Context<Input, ParamValue>) -> RibArgs {
        let seen = Seen::new(context.before);
        RibArgs {
            sketch: seen.sketch_names().pop().unwrap_or_default(),
            solid: seen.model().solid_names().pop().unwrap_or_default(),
            thickness: 0.05,
            side: RibSide::Symmetric,
            direction: RibDirection::Parallel,
            flipped: false,
        }
    }

    fn run(step: &str, part: &mut Build, args: &RibArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The sketch and the solid, picked, the thickness and its side, which
    /// way it grows.
    fn presentation(_: &Context<Input, ParamValue>, args: &RibArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let sketch = one(EntityRef::sketch, &args.sketch);
        dialog.reference("sketch", "sketch", sketch, &[Role::Sketch], None, false);
        let solid = one(EntityRef::solid, &args.solid);
        dialog.reference("solid", "solid", solid, &[Role::Solid], None, false);
        let thickness = Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0);
        dialog.number("thickness", thickness);
        let (side, sides) = choices(&RibSide::ALL, &args.side);
        dialog.select("side", "thickness side", side, sides, false);
        let (direction, directions) = choices(&RibDirection::ALL, &args.direction);
        (dialog.select("direction", "direction", direction, directions, false)).checkbox(
            "flipped",
            "flip direction",
            args.flipped,
        );
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &RibArgs,
        _: &(),
        event: Event<Design>,
    ) -> (RibArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("sketch", Value::Entities(picked)) => {
                args.sketch = picked_one(EntityRef::sketch, &picked);
            }
            ("solid", Value::Entities(picked)) => {
                args.solid = picked_one(EntityRef::solid, &picked);
            }
            ("thickness", Value::Number(t)) => args.thickness = t,
            ("side", Value::Choice(value)) => {
                if let Some(side) = chosen(&RibSide::ALL, &value) {
                    args.side = side;
                }
            }
            ("direction", Value::Choice(value)) => {
                if let Some(direction) = chosen(&RibDirection::ALL, &value) {
                    args.direction = direction;
                }
            }
            ("flipped", Value::Bool(b)) => args.flipped = b,
            _ => {}
        }
        (args, ())
    }
}
