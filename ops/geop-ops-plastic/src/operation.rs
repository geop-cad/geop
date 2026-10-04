//! The plastic features as operations of a program: [`Rib`], [`Lip`],
//! [`Groove`] and [`Draft`].

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::{EdgeId, FaceId};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Choice, Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use geop_core_sketch::Shape;
use geop_ops_extrude_revolve::operation::shape_loops;

use crate::{
    draft::draft,
    lip::{LipSize, groove, lip},
    rib::{Growth, rib},
};

/// Faces by name, as a reference field holds them.
fn face_refs(names: &[String]) -> Vec<EntityRef> {
    names
        .iter()
        .map(|name| EntityRef::Face { name: name.clone() })
        .collect()
}

/// The names of the faces a reference field holds.
fn face_names(picked: &[EntityRef]) -> Vec<String> {
    picked
        .iter()
        .filter_map(|e| match e {
            EntityRef::Face { name } => Some(name.clone()),
            _ => None,
        })
        .collect()
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

impl Operation for Draft {
    type Args = DraftArgs;
    type Session = ();

    /// No faces yet, no neutral plane, and three degrees.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> DraftArgs {
        DraftArgs {
            faces: Vec::new(),
            neutral: None,
            angle: 3.0,
            reversed: false,
        }
    }

    /// The faces and the neutral plane, picked, the angle and the pull
    /// direction.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &DraftArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, DraftArgs> {
        let mut f = Form::<S, DraftArgs>::new();
        f.reference(
            "faces",
            "faces",
            face_refs(&args.faces),
            &[Role::Plane],
            None,
            true,
            |e, picked| e.args.faces = face_names(&picked),
        );
        f.reference(
            "neutral",
            "neutral plane",
            args.neutral.iter().cloned().collect(),
            &[Role::Plane],
            None,
            false,
            |e, picked| e.args.neutral = picked.into_iter().next(),
        );
        f.number(
            "angle",
            Number::new("angle", args.angle, Unit::Angle).range(-45.0, 45.0),
            |args, angle| args.angle = angle,
        );
        f.checkbox(
            "reversed",
            "reverse pull direction",
            args.reversed,
            |args, b| args.reversed = b,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &DraftArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("draft({operation_id}, {args:?})");
        let namer = Namer::new("draft", operation_id)?;
        let faces = args
            .faces
            .iter()
            .map(|name| part.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        let Some(neutral) = &args.neutral else {
            return Err(GeopError::new("pick the neutral plane")).with_context(ctx);
        };
        let frame = neutral.resolve_plane(&part).with_context(ctx)?;
        // A face's frame points out of its solid; the pull goes into it.
        let into_solid = matches!(neutral, EntityRef::Face { .. });
        let pull = if into_solid != args.reversed {
            frame.w().neg()
        } else {
            *frame.w()
        };
        let plane = Plane::try_new(*frame.origin(), pull).with_context(ctx)?;
        if !args.angle.is_finite() {
            return Err(GeopError::new("the angle is not a number")).with_context(ctx);
        }
        let angle = S::from_f64(args.angle.to_radians());
        draft(&mut part, &namer, &faces, &plane, angle).with_context(ctx)?;
        Ok(part)
    }
}

/// The rim face and its edges, as the fields of a lip or a groove hold
/// them.
fn rim_fields<'a, S: Scalar, A: 'a>(
    f: &mut Form<'a, S, A>,
    face: &str,
    edges: &[String],
    rim: fn(&mut A) -> (&mut String, &mut Vec<String>),
) {
    let value = (!face.is_empty())
        .then(|| EntityRef::Face { name: face.into() })
        .into_iter()
        .collect();
    f.reference(
        "face",
        "rim face",
        value,
        &[Role::Plane],
        None,
        false,
        move |e, picked| {
            let (face, edges) = rim(e.args);
            let name = face_names(&picked).pop().unwrap_or_default();
            if name != *face {
                edges.clear();
            }
            *face = name;
        },
    );
    f.reference(
        "edges",
        "edges",
        edges
            .iter()
            .map(|name| EntityRef::Edge { name: name.clone() })
            .collect(),
        &[Role::Edge],
        None,
        true,
        move |e, picked| {
            *rim(e.args).1 = picked
                .iter()
                .filter_map(|p| match p {
                    EntityRef::Edge { name } => Some(name.clone()),
                    _ => None,
                })
                .collect();
        },
    );
    f.optional("edges");
}

/// The rim face and edges named, resolved.
fn resolve_rim<S: Scalar>(
    part: &Part<S>,
    face: &str,
    edges: &[String],
) -> GeopResult<(FaceId, Vec<EdgeId>)> {
    if face.is_empty() {
        return Err(GeopError::new("pick the rim face"));
    }
    Ok((
        part.face_id(face)?,
        edges
            .iter()
            .map(|name| part.edge_id(name))
            .collect::<GeopResult<_>>()?,
    ))
}

/// Fails unless a size typed in is a number.
fn finite(what: &str, value: f64) -> GeopResult<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(GeopError::new(format!("the {what} is not a number")))
    }
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

impl Operation for Lip {
    type Args = LipArgs;
    type Session = ();

    /// No rim yet, a lip 0.05 wide and 0.1 high.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> LipArgs {
        LipArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
        }
    }

    /// The rim and its edges, picked, the width and the height.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &LipArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, LipArgs> {
        let mut f = Form::<S, LipArgs>::new();
        rim_fields(&mut f, &args.face, &args.edges, |a| {
            (&mut a.face, &mut a.edges)
        });
        f.number(
            "width",
            Number::new("width", args.width, Unit::Length).range(0.0, 1.0),
            |args, w| args.width = w,
        );
        f.number(
            "height",
            Number::new("height", args.height, Unit::Length).range(0.0, 1.0),
            |args, h| args.height = h,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &LipArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("lip({operation_id}, {args:?})");
        let namer = Namer::new("lip", operation_id)?;
        let (face, edges) = resolve_rim(&part, &args.face, &args.edges).with_context(ctx)?;
        finite("width", args.width).with_context(ctx)?;
        finite("height", args.height).with_context(ctx)?;
        let size = LipSize {
            width: S::from_f64(args.width),
            height: S::from_f64(args.height),
        };
        lip(&mut part, &namer, operation_id, face, &edges, size).with_context(ctx)?;
        Ok(part)
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

impl Operation for Groove {
    type Args = GrooveArgs;
    type Session = ();

    /// No rim yet, for a lip 0.05 wide and 0.1 high, 0.01 clearance.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> GrooveArgs {
        GrooveArgs {
            face: String::new(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.1,
            clearance: 0.01,
        }
    }

    /// The rim and its edges, picked, the lip's width and height, and the
    /// clearance.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &GrooveArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, GrooveArgs> {
        let mut f = Form::<S, GrooveArgs>::new();
        rim_fields(&mut f, &args.face, &args.edges, |a| {
            (&mut a.face, &mut a.edges)
        });
        f.number(
            "width",
            Number::new("lip width", args.width, Unit::Length).range(0.0, 1.0),
            |args, w| args.width = w,
        );
        f.number(
            "height",
            Number::new("lip height", args.height, Unit::Length).range(0.0, 1.0),
            |args, h| args.height = h,
        );
        f.number(
            "clearance",
            Number::new("clearance", args.clearance, Unit::Length).range(0.0, 0.1),
            |args, c| args.clearance = c,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &GrooveArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("groove({operation_id}, {args:?})");
        let namer = Namer::new("groove", operation_id)?;
        let (face, edges) = resolve_rim(&part, &args.face, &args.edges).with_context(ctx)?;
        finite("width", args.width).with_context(ctx)?;
        finite("height", args.height).with_context(ctx)?;
        finite("clearance", args.clearance).with_context(ctx)?;
        let size = LipSize {
            width: S::from_f64(args.width),
            height: S::from_f64(args.height),
        };
        let clearance = S::from_f64(args.clearance);
        groove(
            &mut part,
            &namer,
            operation_id,
            face,
            &edges,
            size,
            clearance,
        )
        .with_context(ctx)?;
        Ok(part)
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

/// Which way a rib grows from its profile: along the sketch plane's normal,
/// or in the sketch plane, square to the chord between the profile's ends.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RibDirection {
    #[default]
    Parallel,
    Normal,
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

impl Operation for Rib {
    type Args = RibArgs;
    type Session = ();

    /// The newest sketch and solid, 0.05 thick, symmetric, parallel to the
    /// sketch.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> RibArgs {
        RibArgs {
            sketch: before.sketch_names().pop().unwrap_or_default(),
            solid: before.solid_names().pop().unwrap_or_default(),
            thickness: 0.05,
            side: RibSide::Symmetric,
            direction: RibDirection::Parallel,
            flipped: false,
        }
    }

    /// The sketch and the solid, picked, the thickness and its side, which
    /// way it grows.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &RibArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, RibArgs> {
        let mut f = Form::<S, RibArgs>::new();
        let one = |entity: Option<EntityRef>| entity.into_iter().collect::<Vec<_>>();
        f.reference(
            "sketch",
            "sketch",
            one((!args.sketch.is_empty()).then(|| EntityRef::Sketch {
                name: args.sketch.clone(),
            })),
            &[Role::Sketch],
            None,
            false,
            |e, picked| {
                e.args.sketch = match picked.as_slice() {
                    [EntityRef::Sketch { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        f.reference(
            "solid",
            "solid",
            one((!args.solid.is_empty()).then(|| EntityRef::Solid {
                name: args.solid.clone(),
            })),
            &[Role::Solid],
            None,
            false,
            |e, picked| {
                e.args.solid = match picked.as_slice() {
                    [EntityRef::Solid { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        f.number(
            "thickness",
            Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0),
            |args, t| args.thickness = t,
        );
        const SIDES: [(RibSide, &str, &str); 3] = [
            (RibSide::Symmetric, "symmetric", "Symmetric"),
            (RibSide::First, "first", "First side"),
            (RibSide::Second, "second", "Second side"),
        ];
        f.select(
            "side",
            "thickness side",
            SIDES.iter().find(|s| s.0 == args.side).map_or("", |s| s.1),
            SIDES.iter().map(|&(_, v, l)| Choice::new(v, l)).collect(),
            false,
            |args, value| {
                if let Some(&(side, ..)) = SIDES.iter().find(|s| s.1 == value) {
                    args.side = side;
                }
            },
        );
        const DIRECTIONS: [(RibDirection, &str, &str); 2] = [
            (RibDirection::Parallel, "parallel", "Parallel to sketch"),
            (RibDirection::Normal, "normal", "Normal to sketch"),
        ];
        f.select(
            "direction",
            "direction",
            DIRECTIONS
                .iter()
                .find(|d| d.0 == args.direction)
                .map_or("", |d| d.1),
            DIRECTIONS
                .iter()
                .map(|&(_, v, l)| Choice::new(v, l))
                .collect(),
            false,
            |args, value| {
                if let Some(&(direction, ..)) = DIRECTIONS.iter().find(|d| d.1 == value) {
                    args.direction = direction;
                }
            },
        );
        f.checkbox("flipped", "flip direction", args.flipped, |args, b| {
            args.flipped = b
        });
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &RibArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("rib({operation_id}, {args:?})");
        let namer = Namer::new("rib", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let target = part.solid_id(&args.solid).with_context(ctx)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
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
        let profile = shape_loops(&args.sketch, sketch, &geometry, chain)
            .with_context(ctx)?
            .remove(0)
            .profile;
        finite("thickness", args.thickness).with_context(ctx)?;
        let t = S::from_f64(args.thickness);
        let thickness = match args.side {
            RibSide::Symmetric => {
                let half = S::from_f64(args.thickness / 2.0);
                (half.neg(), half)
            }
            RibSide::First => (S::ZERO, t),
            RibSide::Second => (t.neg(), S::ZERO),
        };
        let growth = match args.direction {
            RibDirection::Normal => Growth::Normal {
                flipped: args.flipped,
            },
            RibDirection::Parallel => Growth::Parallel {
                flipped: args.flipped,
            },
        };
        rib(
            &mut part,
            &namer,
            operation_id,
            &placed.plane,
            &profile,
            target,
            thickness,
            growth,
        )
        .with_context(ctx)?;
        Ok(part)
    }
}
