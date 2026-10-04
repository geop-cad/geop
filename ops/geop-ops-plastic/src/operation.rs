//! The plastic features as operations of a program: [`Draft`], [`Lip`] and
//! [`Groove`].

use geop_core_geometry::shape::Plane;
use geop_core_topology::{EdgeId, FaceId};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use crate::{
    draft::draft,
    lip::{LipSize, groove, lip},
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
        groove(&mut part, &namer, operation_id, face, &edges, size, clearance)
            .with_context(ctx)?;
        Ok(part)
    }
}
