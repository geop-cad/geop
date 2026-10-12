//! The plastic features as operations of a program: [`Rib`], [`Lip`],
//! [`Groove`] and [`Draft`].

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_sketch::Shape;
use geop_core_topology::{EdgeId, FaceId};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{
        BuildReader, EntityRef, Lookup, PartOperation, ResolveEntity, Role, Scope, Seen,
        StepContext,
    },
    part_operation,
    ui::{EditForm, FormState, Number, Unit, choices, chosen},
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

part_operation!(Draft, Lip, Groove, Rib);

impl PartOperation for Draft {
    const INFO: OperationInfo = info(
        "draft",
        "Draft",
        "Tilt planar faces about a neutral plane, so the part comes out of its mould.",
    )
    .icon(crate::icons::DRAFT);
    type Args = DraftArgs;
    type Session = ();

    /// No faces yet, no neutral plane, and three degrees.
    fn new_args(_: &StepContext) -> DraftArgs {
        DraftArgs {
            faces: Vec::new(),
            neutral: None,
            angle: 3.0,
            reversed: false,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &DraftArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("draft", step)?;
        let faces = (args.faces.iter())
            .map(|name| model.face_id(name))
            .collect::<GeopResult<Vec<_>>>()?;
        let Some(neutral) = &args.neutral else {
            return Err(GeopError::new("pick the neutral plane"));
        };
        let frame = neutral.resolve_plane(&mut Scope { model, reader })?;
        // A face's frame points out of its solid; the pull goes into it.
        let into_solid = matches!(neutral, EntityRef::Face { .. });
        let pull = if into_solid != args.reversed {
            frame.w().neg()
        } else {
            *frame.w()
        };
        let plane = Plane::try_new(*frame.origin(), pull)?;
        finite("angle", args.angle)?;
        let angle = Design::from_f64(args.angle.to_radians());
        draft(model, &namer, &faces, &plane, angle)?;
        Ok(())
    }

    /// The faces and the neutral plane, picked, the angle and the pull
    /// direction.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &DraftArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, DraftArgs> {
        let mut f = EditForm::<Design, DraftArgs>::default();
        f.pick_many(
            "faces",
            "faces",
            EntityRef::face,
            &args.faces,
            &[Role::Plane],
            |edit, names| edit.args.faces = names,
        );
        let neutral = args.neutral.iter().cloned().collect();
        let roles = [Role::Plane];
        f.reference(
            "neutral",
            "neutral plane",
            neutral,
            &roles,
            None,
            false,
            |edit, picked| edit.args.neutral = picked.into_iter().next(),
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

/// The fields of the rim face `face` and its edges `edges`, of a lip or a
/// groove whose arguments `rim` says where they are held: the edges picked
/// go with the face they were picked on.
fn rim_fields<'a, A: 'a>(
    f: &mut EditForm<'a, Design, A>,
    face: &str,
    edges: &[String],
    rim: fn(&mut A) -> (&mut String, &mut Vec<String>),
) {
    let face = EntityRef::of_name(EntityRef::face, face);
    f.reference(
        "face",
        "rim face",
        face,
        &[Role::Plane],
        None,
        false,
        move |edit, picked| {
            let (face, edges) = rim(edit.args);
            let name = EntityRef::last_name(EntityRef::face, &picked).unwrap_or_default();
            if name != *face {
                edges.clear();
            }
            *face = name;
        },
    );
    let edges = EntityRef::of_names(EntityRef::edge, edges);
    f.reference(
        "edges",
        "edges",
        edges,
        &[Role::Edge],
        None,
        true,
        move |edit, picked| {
            *rim(edit.args).1 = EntityRef::names_of(EntityRef::edge, &picked);
        },
    );
    f.form.dialog.optional("edges");
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

impl PartOperation for Lip {
    const INFO: OperationInfo = info(
        "lip",
        "Lip",
        "Raise a lip along the rim of one half of an enclosure.",
    )
    .icon(crate::icons::LIP);
    type Args = LipArgs;
    type Session = ();

    /// No rim yet, a lip 0.05 wide and 0.1 high.
    fn new_args(_: &StepContext) -> LipArgs {
        LipArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &LipArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("lip", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges)?;
        finite("width", args.width)?;
        finite("height", args.height)?;
        let size = LipSize {
            width: Design::from_f64(args.width),
            height: Design::from_f64(args.height),
        };
        lip(model, &namer, step, face, &edges, size)
    }

    /// The rim and its edges, picked, the width and the height.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &LipArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, LipArgs> {
        let mut f = EditForm::<Design, LipArgs>::default();
        rim_fields(&mut f, &args.face, &args.edges, |a| {
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

impl PartOperation for Groove {
    const INFO: OperationInfo = info(
        "groove",
        "Groove",
        "Cut the groove that takes a lip into the rim of the other half.",
    )
    .icon(crate::icons::GROOVE);
    type Args = GrooveArgs;
    type Session = ();

    /// No rim yet, for a lip 0.05 wide and 0.1 high, 0.01 clearance.
    fn new_args(_: &StepContext) -> GrooveArgs {
        GrooveArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
            clearance: 0.01,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &GrooveArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("groove", step)?;
        let (face, edges) = resolve_rim(model, &args.face, &args.edges)?;
        finite("width", args.width)?;
        finite("height", args.height)?;
        finite("clearance", args.clearance)?;
        let size = LipSize {
            width: Design::from_f64(args.width),
            height: Design::from_f64(args.height),
        };
        let clearance = Design::from_f64(args.clearance);
        groove(model, &namer, step, face, &edges, size, clearance)
    }

    /// The rim and its edges, picked, the lip's width and height, and the
    /// clearance.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &GrooveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, GrooveArgs> {
        let mut f = EditForm::<Design, GrooveArgs>::default();
        rim_fields(&mut f, &args.face, &args.edges, |a| {
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

impl PartOperation for Rib {
    const INFO: OperationInfo = info(
        "rib",
        "Rib",
        "Grow a thin wall from an open sketch profile up to a solid's faces, and join it.",
    )
    .icon(crate::icons::RIB);
    type Args = RibArgs;
    type Session = ();

    /// The newest sketch and solid, 0.05 thick, symmetric, parallel to the
    /// sketch.
    fn new_args(context: &StepContext) -> RibArgs {
        let seen = Seen::new(context.before);
        RibArgs {
            sketch: seen.sketch_names().pop().unwrap_or_default(),
            solid: seen.model().newest_solid().unwrap_or_default(),
            thickness: 0.05,
            side: RibSide::Symmetric,
            direction: RibDirection::Parallel,
            flipped: false,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &RibArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("rib", step)?;
        let placed = (Scope { model, reader }.sketch(&args.sketch))?.clone();
        let target = model.solid_id(&args.solid)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<Design>()?;
        let chain = match sketch.shape()? {
            chain @ Shape::Chain(_) => chain,
            Shape::Region(_) => {
                return Err(GeopError::new(format!(
                    "sketch {:?} encloses an area: a rib grows from an open profile",
                    args.sketch
                )));
            }
        };
        let profile = shape_loops(&args.sketch, sketch, &geometry, chain)?
            .remove(0)
            .profile;
        finite("thickness", args.thickness)?;
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
    }

    /// The sketch and the solid, picked, the thickness and its side, which
    /// way it grows.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &RibArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, RibArgs> {
        let mut f = EditForm::<Design, RibArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            EntityRef::sketch,
            &args.sketch,
            &[Role::Sketch],
            |edit, name| edit.args.sketch = name,
        );
        f.pick_one(
            "solid",
            "solid",
            EntityRef::solid,
            &args.solid,
            &[Role::Solid],
            |edit, name| edit.args.solid = name,
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
