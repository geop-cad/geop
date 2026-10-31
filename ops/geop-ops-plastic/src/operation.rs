//! The plastic features as operations of a program: [`Rib`], [`Lip`],
//! [`Groove`] and [`Draft`].

use geop_core_editor::{
    edit_form::{EditForm, FormState, choices, chosen},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_core_sketch::Shape;
use geop_core_topology::{EdgeId, FaceId};
use geop_ops_model::sketch::shape_profiles;
use geop_ops_model::sketch::{PlacedSketch, SketchPlane, SketchRef};
use geop_ops_model::{
    Edge, Face, ModelOperation, NamedModel, Namer, build_model, model, read_model,
};
use geop_ops_model::{SolidRef, solid};
use serde::{Deserialize, Serialize};

use crate::{
    draft::draft,
    lip::{LipSize, groove, lip},
    rib::{Growth, rib},
};

/// How the plastic features are offered: rarely used, and not yet robust
/// enough to offer by default.
const fn info(label: &'static str, doc: &'static str) -> OperationInfo {
    OperationInfo::new(label, doc, OperationGroup::Features, OperationTier::Menu).experimental()
}

/// A face of the model, by name.
type FaceRef = ElementReference<NamedModel, Face>;

/// An edge of the model, by name.
type EdgeRef = ElementReference<NamedModel, Edge>;

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
    pub faces: Vec<FaceRef>,
    /// The plane they tilt about: a planar face, a datum plane, a sketch's
    /// plane.
    pub neutral: Option<SketchPlane>,
    /// How far they tilt, in degrees.
    pub angle: f64,
    /// Pull the other way.
    #[serde(default)]
    pub reversed: bool,
}

part_operation!(
    Draft(DraftArgs),
    Lip(LipArgs),
    Groove(GrooveArgs),
    Rib(RibArgs)
);

impl PartOperation for Draft {
    const KIND: &'static str = "draft";
    const INFO: OperationInfo = info(
        "Draft",
        "Tilt planar faces about a neutral plane, so the part comes out of its mould.",
    )
    .icon(crate::icons::DRAFT);
    type Args = DraftArgs;
    type Session = ();

    /// No faces yet, no neutral plane, and three degrees.
    fn new_args(_: &Context) -> DraftArgs {
        DraftArgs {
            faces: Vec::new(),
            neutral: None,
            angle: 3.0,
            reversed: false,
        }
    }

    fn run(ctx: &mut StepContext, args: &DraftArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The faces and the neutral plane, picked, the angle and the pull
    /// direction.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &DraftArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, DraftArgs> {
        let before = &context.before;
        let mut f = EditForm::<DraftArgs>::default();
        f.pick(
            "faces",
            "faces",
            before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        f.pick_one(
            "neutral",
            "neutral plane",
            before,
            args.neutral.as_ref(),
            |edit, neutral| edit.args.neutral = Some(neutral),
        );
        let angle = Number::new("angle", args.angle, Unit::Angle).range(-45.0, 45.0);
        f.number("angle", angle, |edit, angle| edit.args.angle = angle);
        f.checkbox(
            "reversed",
            "reverse pull direction",
            args.reversed,
            |edit, b| edit.args.reversed = b,
        );
        f
    }
}

impl ModelOperation for Draft {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &DraftArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("draft", step)?;
        let faces = (args.faces.iter())
            .map(|face| model.face_id(face.name()))
            .collect::<GeopResult<Vec<_>>>()?;
        let Some(neutral) = &args.neutral else {
            return Err(GeopError::new("pick the neutral plane"));
        };
        let frame = neutral.resolve(reader)?;
        // A face's frame points out of its solid; the pull goes into it.
        let into_solid = matches!(neutral, SketchPlane::Face(_));
        let pull = if into_solid != args.reversed {
            frame.w().neg()
        } else {
            *frame.w()
        };
        let plane = Plane::try_new(*frame.origin(), pull)?;
        finite("angle", args.angle)?;
        let angle = Scalar::from_f64(args.angle.to_radians());
        draft(model, &namer, &faces, &plane, angle)?;
        Ok(())
    }
}

/// The fields of the rim face `face` and its edges `edges`, of a lip or a
/// groove whose arguments `rim` says where they are held, in the part
/// `before` the step: the edges picked go with the face they were picked
/// on.
fn rim_fields<'a, A: 'a>(
    f: &mut EditForm<'a, A>,
    before: &Reader<'a>,
    face: &FaceRef,
    edges: &[EdgeRef],
    rim: fn(&mut A) -> (&mut FaceRef, &mut Vec<EdgeRef>),
) {
    f.pick_one(
        "face",
        "rim face",
        before,
        Some(face),
        move |edit, picked| {
            let (face, edges) = rim(edit.args);
            if picked != *face {
                edges.clear();
            }
            *face = picked;
        },
    );
    f.pick(
        "edges",
        "edges",
        before,
        edges,
        true,
        move |edit, picked| *rim(edit.args).1 = picked,
    );
    f.form.dialog.optional("edges");
}

/// The rim face and edges, resolved.
fn resolve_rim(
    model: &NamedModel,
    face: &FaceRef,
    edges: &[EdgeRef],
) -> GeopResult<(FaceId, Vec<EdgeId>)> {
    if face.name().is_empty() {
        return Err(GeopError::new("pick the rim face"));
    }
    Ok((
        model.face_id(face.name())?,
        (edges.iter())
            .map(|edge| model.edge_id(edge.name()))
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
    pub face: FaceRef,
    /// Edges of the rim to run along, one chain; none for all of its hole.
    #[serde(default)]
    pub edges: Vec<EdgeRef>,
    pub width: f64,
    pub height: f64,
}

impl PartOperation for Lip {
    const KIND: &'static str = "lip";
    const INFO: OperationInfo = info(
        "Lip",
        "Raise a lip along the rim of one half of an enclosure.",
    )
    .icon(crate::icons::LIP);
    type Args = LipArgs;
    type Session = ();

    /// No rim yet, a lip 0.05 wide and 0.1 high.
    fn new_args(_: &Context) -> LipArgs {
        LipArgs {
            face: ElementReference::new(model(), ""),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
        }
    }

    fn run(ctx: &mut StepContext, args: &LipArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The rim and its edges, picked, the width and the height.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &LipArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, LipArgs> {
        let mut f = EditForm::<LipArgs>::default();
        rim_fields(&mut f, &context.before, &args.face, &args.edges, |a| {
            (&mut a.face, &mut a.edges)
        });
        let length = |label: &str, value: f64| Number::new(label, value, Unit::Length);
        f.number(
            "width",
            length("width", args.width).range(0.0, 1.0),
            |edit, w| edit.args.width = w,
        );
        f.number(
            "height",
            length("height", args.height).range(0.0, 1.0),
            |edit, h| edit.args.height = h,
        );
        f
    }
}

impl ModelOperation for Lip {
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &LipArgs) -> GeopResult<()> {
        let namer = Namer::new("lip", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges)?;
        finite("width", args.width)?;
        finite("height", args.height)?;
        let size = LipSize {
            width: Scalar::from_f64(args.width),
            height: Scalar::from_f64(args.height),
        };
        lip(model, &namer, step, face, &edges, size)
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
    pub face: FaceRef,
    /// Edges of the rim to run along, one chain; none for all of its hole.
    #[serde(default)]
    pub edges: Vec<EdgeRef>,
    /// The lip's width.
    pub width: f64,
    /// The lip's height.
    pub height: f64,
    /// How much wider and deeper the groove is than the lip.
    pub clearance: f64,
}

impl PartOperation for Groove {
    const KIND: &'static str = "groove";
    const INFO: OperationInfo = info(
        "Groove",
        "Cut the groove that takes a lip into the rim of the other half.",
    )
    .icon(crate::icons::GROOVE);
    type Args = GrooveArgs;
    type Session = ();

    /// No rim yet, for a lip 0.05 wide and 0.1 high, 0.01 clearance.
    fn new_args(_: &Context) -> GrooveArgs {
        GrooveArgs {
            face: ElementReference::new(model(), ""),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
            clearance: 0.01,
        }
    }

    fn run(ctx: &mut StepContext, args: &GrooveArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The rim and its edges, picked, the lip's width and height, and the
    /// clearance.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &GrooveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, GrooveArgs> {
        let mut f = EditForm::<GrooveArgs>::default();
        rim_fields(&mut f, &context.before, &args.face, &args.edges, |a| {
            (&mut a.face, &mut a.edges)
        });
        let length = |label: &str, value: f64| Number::new(label, value, Unit::Length);
        f.number(
            "width",
            length("lip width", args.width).range(0.0, 1.0),
            |edit, w| edit.args.width = w,
        );
        f.number(
            "height",
            length("lip height", args.height).range(0.0, 1.0),
            |edit, h| edit.args.height = h,
        );
        let clearance = length("clearance", args.clearance).range(0.0, 0.1);
        f.number("clearance", clearance, |edit, c| edit.args.clearance = c);
        f
    }
}

impl ModelOperation for Groove {
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &GrooveArgs) -> GeopResult<()> {
        let namer = Namer::new("groove", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges)?;
        finite("width", args.width)?;
        finite("height", args.height)?;
        finite("clearance", args.clearance)?;
        let size = LipSize {
            width: Scalar::from_f64(args.width),
            height: Scalar::from_f64(args.height),
        };
        let clearance = Scalar::from_f64(args.clearance);
        groove(model, &namer, step, face, &edges, size, clearance)
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
    pub sketch: SketchRef,
    /// The solid it grows up to and joins.
    pub solid: SolidRef,
    pub thickness: f64,
    #[serde(default)]
    pub side: RibSide,
    #[serde(default)]
    pub direction: RibDirection,
    /// Grow the other way.
    #[serde(default)]
    pub flipped: bool,
}

impl PartOperation for Rib {
    const KIND: &'static str = "rib";
    const INFO: OperationInfo = info(
        "Rib",
        "Grow a thin wall from an open sketch profile up to a solid's faces, and join it.",
    )
    .icon(crate::icons::RIB);
    type Args = RibArgs;
    type Session = ();

    /// The newest sketch and solid, 0.05 thick, symmetric, parallel to the
    /// sketch.
    fn new_args(context: &Context) -> RibArgs {
        let before = &context.before;
        let newest = before.targets_of::<PlacedSketch>().pop();
        let solid_name = read_model(before)
            .ok()
            .and_then(|model| model.newest_solid());
        RibArgs {
            sketch: SketchRef::whole(newest.unwrap_or_else(|| geop_ops_model::sketch::sketch(""))),
            solid: solid(solid_name.unwrap_or_default()),
            thickness: 0.05,
            side: RibSide::Symmetric,
            direction: RibDirection::Parallel,
            flipped: false,
        }
    }

    fn run(ctx: &mut StepContext, args: &RibArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sketch and the solid, picked, the thickness and its side, which
    /// way it grows.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &RibArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, RibArgs> {
        let before = &context.before;
        let mut f = EditForm::<RibArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            before,
            Some(&args.sketch),
            |edit, sketch| edit.args.sketch = sketch,
        );
        f.pick_one(
            "solid",
            "solid",
            before,
            Some(&args.solid),
            |edit, solid| edit.args.solid = solid,
        );
        let thickness = Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0);
        f.number("thickness", thickness, |edit, t| edit.args.thickness = t);
        let (side, sides) = choices(&RibSide::ALL, &args.side);
        f.select(
            "side",
            "thickness side",
            side,
            sides,
            false,
            |edit, value| {
                if let Some(side) = chosen(&RibSide::ALL, value) {
                    edit.args.side = side;
                }
            },
        );
        let (direction, directions) = choices(&RibDirection::ALL, &args.direction);
        f.select(
            "direction",
            "direction",
            direction,
            directions,
            false,
            |edit, value| {
                if let Some(direction) = chosen(&RibDirection::ALL, value) {
                    edit.args.direction = direction;
                }
            },
        );
        f.checkbox("flipped", "flip direction", args.flipped, |edit, b| {
            edit.args.flipped = b
        });
        f
    }
}

impl ModelOperation for Rib {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &RibArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("rib", step)?;
        let placed = reader.retrieve_element(&args.sketch).concerning("sketch")?;
        let target = model.solid_id(args.solid.name())?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose()?;
        let chain = match sketch.shape()? {
            chain @ Shape::Chain(_) => chain,
            Shape::Region(_) => {
                return Err(GeopError::new(format!(
                    "sketch {:?} encloses an area: a rib grows from an open profile",
                    placed.name
                )));
            }
        };
        let profile = shape_profiles(&placed.name, sketch, &geometry, chain)?.remove(0);
        finite("thickness", args.thickness)?;
        let t = args.thickness;
        let (near, far) = match args.side {
            RibSide::Symmetric => (-t / 2.0, t / 2.0),
            RibSide::First => (0.0, t),
            RibSide::Second => (-t, 0.0),
        };
        let thickness = (Scalar::from_f64(near), Scalar::from_f64(far));
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
    }
}
