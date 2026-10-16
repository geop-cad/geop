//! [`TrimSurface`]: a sheet cut back to one side of a face.
//!
//! The face cuts the sheet as it cuts a solid in a split: a copy of it is
//! imprinted onto the sheet (see [`remesh`]), which divides the sheet's
//! faces along where the two cross. Every piece of the sheet then lies on
//! one side of the face's surface — in front of it, where its normal
//! points, or behind it — and the pieces on the side asked for are kept.
//! The copy is deleted again.
//!
//! A piece that reaches both sides means the face does not cut the sheet
//! clean through there: refused, naming the piece.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::Choice,
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::{
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    element::ElementReference, part::StepContext, reader::Reader, values::Design,
};
use geop_core_topology::{Body, FaceId, contains::face::face_interior_point};
use geop_lib_booleans::{RemeshParams, remesh};
use geop_ops_model::{Face, ModelOperation, NamedModel, Namer, build_model, model};
use serde::{Deserialize, Serialize};

/// Which side of the cutting face a [`TrimSurface`] keeps, as its normal
/// points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrimKeep {
    /// Where the face's normal points.
    #[default]
    Front,
    /// Behind it.
    Back,
}

/// Cuts the sheet the face named `face` stands in back to the side `keep`
/// of the face named `tool` — of a solid or standing on its own, left as it
/// is — for the operation `T` (see the module docs). What survives of the
/// sheet keeps its names; what the cut creates is named as a split names
/// it, `trim(T,...)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TrimSurface;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrimSurfaceArgs {
    /// A face of the sheet to trim.
    pub face: ElementReference<NamedModel<Design>, Face>,
    /// The face to cut it with.
    pub tool: ElementReference<NamedModel<Design>, Face>,
    /// The side of the tool to keep.
    #[serde(default)]
    pub keep: TrimKeep,
}

impl TrimKeep {
    fn key(self) -> &'static str {
        match self {
            TrimKeep::Front => "front",
            TrimKeep::Back => "back",
        }
    }
}

part_operation!(TrimSurface(TrimSurfaceArgs));

impl PartOperation for TrimSurface {
    const KIND: &'static str = "trim_surface";
    const INFO: OperationInfo = OperationInfo::new(
        "Trim surface",
        "Cut a face standing on its own back to one side of another face.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::TRIM_SURFACE);
    type Args = TrimSurfaceArgs;
    type Session = ();

    /// Nothing picked yet, keeping the front.
    fn new_args(_: &Context) -> TrimSurfaceArgs {
        TrimSurfaceArgs {
            face: ElementReference::new(model(), ""),
            tool: ElementReference::new(model(), ""),
            keep: TrimKeep::Front,
        }
    }

    fn run(ctx: &mut StepContext, args: &TrimSurfaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sheet and the tool, picked, and the side to keep.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &TrimSurfaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, TrimSurfaceArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, TrimSurfaceArgs>::default();
        f.pick_one("face", "sheet", before, Some(&args.face), |edit, face| {
            edit.args.face = face
        });
        f.pick_one(
            "tool",
            "cut with",
            before,
            Some(&args.tool),
            |edit, tool| edit.args.tool = tool,
        );
        let sides = vec![
            Choice::new("front", "in front of the face"),
            Choice::new("back", "behind the face"),
        ];
        f.select(
            "keep",
            "keep",
            args.keep.key(),
            sides,
            false,
            |edit, choice| {
                edit.args.keep = if choice == "back" {
                    TrimKeep::Back
                } else {
                    TrimKeep::Front
                }
            },
        );
        f
    }
}

impl ModelOperation for TrimSurface {
    fn build(
        model: &mut NamedModel<Design>,
        _: &Reader,
        step: &str,
        args: &TrimSurfaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("trim", step)?;
        let tool = model.face_id(args.tool.name())?;
        trim(model, &namer, args.face.name(), tool, args.keep)
    }
}

/// On which side of `surface` the point `p` lies: `Some(true)` in front,
/// where its normal points, `None` if it could be on it — measured from
/// the foot of `p` on it, refined by Newton from the point of the surface
/// nearest it.
fn in_front<S: Scalar>(surface: &NurbSurface3D<S>, p: &Vector3<S>) -> GeopResult<Option<bool>> {
    let distance = if let Some(plane) = surface.as_plane()? {
        plane.signed_distance(p)
    } else {
        let (u, v, _) = surface.closest_point(p)?;
        let (u, v) = surface.project(*p, u, v, PROJECT_ITERATIONS)?;
        let foot = surface.evaluate(u, v)?;
        p.sub(&foot).prod_dot(&surface.normal(u, v)?)
    };
    Ok(if distance.definitely_greater(S::ZERO) {
        Some(true)
    } else if distance.definitely_less(S::ZERO) {
        Some(false)
    } else {
        None
    })
}

/// Cuts the sheet of the face named `face` back to the side `keep` of the
/// face `tool` (see the module docs).
pub fn trim<S: Scalar>(
    part: &mut NamedModel<S>,
    namer: &Namer,
    face: &str,
    tool: FaceId,
    keep: TrimKeep,
) -> GeopResult<()> {
    let sheet = part.sheet_of_face(part.face_id(face)?, "a sheet is trimmed")?;
    let tool_name = part.name(tool).map(str::to_string)?;
    if part
        .topology()
        .body_faces(Body::Sheet(sheet))?
        .contains(&tool)
    {
        return Err(GeopError::new(format!(
            "face {tool_name} is a face of the sheet to trim: it cannot cut its own sheet"
        )));
    }
    let surface = part.topology().get_face(tool)?.surface.clone();
    let cutter = part
        .copy_faces(&[tool], None, |name| namer.name(&[name, "tool"]))?
        .body()?;
    remesh(
        part,
        namer,
        Body::Sheet(sheet),
        cutter,
        RemeshParams::default(),
    )?;

    let model = part.topology();
    let want = keep == TrimKeep::Front;
    let mut kept = Vec::new();
    let mut cut_away = 0;
    for f in model.body_faces(Body::Sheet(sheet))? {
        let (u, v) = face_interior_point(model, f, MAX_NODES, min_subdivision_size(), 0x7121)?;
        let at = model.get_face(f)?.surface.evaluate(u, v)?;
        let Some(side) = in_front(&surface, &at)? else {
            return Err(GeopError::new(format!(
                "face {} of the sheet lies along face {tool_name}: which side of it it is on is undecided",
                part.name(f).map(str::to_string)?
            )));
        };
        for c in model.iterate_face_coedges(f) {
            let vertex = model.coedge_start_vertex(c)?;
            if in_front(&surface, &vertex.point)? == Some(!side) {
                return Err(GeopError::new(format!(
                    "face {} of the sheet reaches both sides of face {tool_name}: the face does not cut the sheet clean through there",
                    part.name(f).map(str::to_string)?
                )));
            }
        }
        if side == want {
            kept.push(f);
        } else {
            cut_away += 1;
        }
    }
    if kept.is_empty() || cut_away == 0 {
        return Err(GeopError::new(format!(
            "face {tool_name} leaves {} of the sheet on the side to keep: it does not cut the sheet",
            if kept.is_empty() { "nothing" } else { "all" }
        )));
    }
    part.delete_bodies(&[cutter])?;
    part.assemble_sheet(&[Body::Sheet(sheet)], &kept)?;
    Ok(())
}

#[cfg(test)]
mod tests;
