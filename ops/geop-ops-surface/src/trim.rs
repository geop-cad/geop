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

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{Body, FaceId, contains::face::face_interior_point};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Operation, Role},
    ui::{Choice, Form},
};
use geop_ops_booleans::remesh::remesh::{RemeshParams, remesh};
use serde::{Deserialize, Serialize};

use crate::{MAX_NODES, face, min_subdivision_size, name_of, picked_names, refs, sheet_of};

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

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TrimSurfaceArgs {
    /// A face of the sheet to trim.
    pub face: String,
    /// The face to cut it with.
    pub tool: String,
    /// The side of the tool to keep.
    #[serde(default)]
    pub keep: TrimKeep,
}

impl Operation for TrimSurface {
    type Args = TrimSurfaceArgs;
    type Session = ();

    /// Nothing picked yet, keeping the front.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> TrimSurfaceArgs {
        TrimSurfaceArgs::default()
    }

    /// The sheet and the tool, picked, and the side to keep.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &TrimSurfaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, TrimSurfaceArgs> {
        let mut f = Form::<S, TrimSurfaceArgs>::new();
        let one = |name: &String| -> Vec<String> {
            (!name.is_empty()).then(|| name.clone()).into_iter().collect()
        };
        f.reference(
            "face",
            "sheet",
            refs(&one(&args.face), face),
            &[Role::Sheet],
            None,
            false,
            |e, picked| e.args.face = picked_names(&picked).pop().unwrap_or_default(),
        );
        f.reference(
            "tool",
            "cut with",
            refs(&one(&args.tool), face),
            &[Role::Face],
            None,
            false,
            |e, picked| e.args.tool = picked_names(&picked).pop().unwrap_or_default(),
        );
        let key = |k: TrimKeep| match k {
            TrimKeep::Front => "front",
            TrimKeep::Back => "back",
        };
        f.select(
            "keep",
            "keep",
            key(args.keep),
            vec![
                Choice::new("front", "in front of the face"),
                Choice::new("back", "behind the face"),
            ],
            false,
            |args, choice| {
                args.keep = if choice == "back" {
                    TrimKeep::Back
                } else {
                    TrimKeep::Front
                }
            },
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &TrimSurfaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("trim_surface({operation_id}, {args:?})");
        let namer = Namer::new("trim", operation_id)?;
        let tool = part.face_id(&args.tool).with_context(ctx)?;
        trim(&mut part, &namer, &args.face, tool, args.keep).with_context(ctx)?;
        Ok(part)
    }
}

/// On which side of `surface` the point `p` lies: `Some(true)` in front,
/// where its normal points, `None` if it could be on it — measured from
/// the foot of `p` on it, found by Newton from the nearest of a grid of
/// seeds (which seed is a free choice).
fn in_front<S: Scalar>(surface: &NurbSurface3D<S>, p: &Vector3<S>) -> GeopResult<Option<bool>> {
    let distance = if let Some(plane) = surface.as_plane()? {
        plane.signed_distance(p)
    } else {
        const GRID: i64 = 8;
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        let mut best: Option<(f64, S, S)> = None;
        for i in 0..=GRID {
            for j in 0..=GRID {
                let u = S::interpolate(u0, u1, S::from_ratio(i, GRID)?).sharpen();
                let v = S::interpolate(v0, v1, S::from_ratio(j, GRID)?).sharpen();
                let d = surface.evaluate(u, v)?.sub(p).norm_sq().to_f64();
                if best.is_none_or(|(b, _, _)| d < b) {
                    best = Some((d, u, v));
                }
            }
        }
        let (_, u, v) = best.expect("a grid of seeds");
        let (u, v) = surface.project(*p, u, v, 20)?;
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
    part: &mut Part<S>,
    namer: &Namer,
    face: &str,
    tool: FaceId,
    keep: TrimKeep,
) -> GeopResult<()> {
    let (_, sheet) = sheet_of(part, face)?;
    let tool_name = name_of(part, tool)?;
    if part.topology().body_faces(Body::Sheet(sheet))?.contains(&tool) {
        return Err(GeopError::new(format!(
            "face {tool_name} is a face of the sheet to trim: it cannot cut its own sheet"
        )));
    }
    let surface = part.topology().get_face(tool)?.surface.clone();
    let cutter = part.copy_faces(&[tool], None, |name| namer.name(&[name, "tool"]))?;
    let cutter = Body::Sheet(cutter.shells[0]);
    remesh(part, namer, Body::Sheet(sheet), cutter, RemeshParams::default())?;

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
                name_of(part, f)?
            )));
        };
        for c in model.iterate_face_coedges(f) {
            let vertex = model.coedge_start_vertex(c)?;
            if in_front(&surface, &vertex.point)? == Some(!side) {
                return Err(GeopError::new(format!(
                    "face {} of the sheet reaches both sides of face {tool_name}: the face does not cut the sheet clean through there",
                    name_of(part, f)?
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
    part.assemble_sheet(&[cutter], &[])?;
    part.assemble_sheet(&[Body::Sheet(sheet)], &kept)?;
    Ok(())
}

#[cfg(test)]
mod tests;
