//! Direct edits of a solid's faces, its topology kept: [`MoveFace`] shifts
//! faces, [`OffsetFace`] pushes or pulls them along their normals, and
//! [`ReplaceFace`] puts a face on the surface of another. Each puts the
//! faces on their new surfaces and rebuilds the solid around them
//! ([`geop_lib_shell::replace_faces`]): every vertex and edge they reach
//! moves to where the surfaces now meet, and every name is kept.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState, choices, chosen},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{CanvasEvent, Change, Gizmo, Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::{nurb_surface::NurbSurface3D, search::PROJECT_ITERATIONS};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_core_topology::FaceId;
use geop_lib_shell::{Heal, Smooth, delete_faces, replace_faces};
use geop_ops_model::{
    Body, Face, ModelOperation, NamedModel, Namer, build_model, model, read_model, resolve_bodies,
};
use serde::{Deserialize, Serialize};

/// A face of the model, by name.
type FaceRef = ElementReference<NamedModel, Face>;

/// The faces `faces` refers to, in `model`.
fn face_ids(model: &NamedModel, faces: &[FaceRef]) -> GeopResult<Vec<FaceId>> {
    faces.iter().map(|f| model.face_id(f.name())).collect()
}

/// Fails unless a value typed in is a number.
fn finite(what: &str, value: f64) -> GeopResult<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(GeopError::new(format!("the {what} is not a number")))
    }
}

/// The middle of the corners of the faces `faces` in the part `reader`
/// reads, where a handle sits: their average, a free choice.
fn faces_middle(reader: &Reader, faces: &[FaceRef]) -> Option<Vector3> {
    let part = read_model(reader).ok()?;
    let model = part.topology();
    let mut sum = Vector3::zero();
    let mut count = 0usize;
    for face in face_ids(&part, faces).ok()? {
        for edge in model.face_edges(face) {
            let start = model.get_edge(edge).ok()?.start_vertex;
            sum = sum.add(&model.get_vertex(start).ok()?.point);
            count += 1;
        }
    }
    let share = Scalar::from_ratio(1, i64::try_from(count).ok()?).ok()?;
    Some(sum.prod_scalar(share))
}

/// Shifts the faces `faces` of a solid by `translation`, along the world's
/// `x`, `y` and `z`: each is put on its surface shifted, and the solid
/// rebuilt around them — a face pulled out of a box makes it longer, a
/// face of a hole shifted moves the hole. A face takes along the faces it
/// runs smoothly into: the other pieces of a hole's wall, a fillet along
/// it. The solid is consumed and the
/// result named `move_face(M)` for the operation `M`; every face, edge and
/// vertex keeps its name.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MoveFace;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MoveFaceArgs {
    /// The faces to move, all of one solid.
    pub faces: Vec<FaceRef>,
    /// The shift, along the world's `x`, `y` and `z`.
    #[serde(default)]
    pub translation: [f64; 3],
}

/// Pushes or pulls the faces `faces` of a solid `distance` along their
/// outward normals — out of the solid for a positive distance, into it for
/// a negative one: each is put on its surface offset, and the solid
/// rebuilt around them. A face takes along the faces it runs smoothly
/// into, offset as far: the other pieces of a cylinder, a fillet. The
/// solid is consumed and the result named
/// `offset_face(O)` for the operation `O`; every face, edge and vertex
/// keeps its name.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OffsetFace;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OffsetFaceArgs {
    /// The faces to offset, all of one solid.
    pub faces: Vec<FaceRef>,
    /// How far, out of the solid; into it if negative.
    pub distance: f64,
}

/// Puts the face `face` of a solid on the surface of the face `with` — of
/// a sheet, or of another body — and rebuilds the solid around it: its
/// neighbours trimmed or grown to where they meet the new surface. The
/// solid is consumed and the result named `replace_face(R)` for the
/// operation `R`; every face, edge and vertex keeps its name.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ReplaceFace;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReplaceFaceArgs {
    /// The face to put on another surface.
    pub face: FaceRef,
    /// The face whose surface it goes on.
    pub with: FaceRef,
}

/// Takes the faces `faces` out of their solid, and leaves the gap as
/// `heal` says: healed over by the faces around it — a hole, a boss or a
/// pocket filled in, a chamfer or a fillet taken back to its sharp edge, a
/// cut-off corner restored — or open, the rest of the solid a sheet. The
/// solid is consumed; what is left is named `delete_face(D)` for the
/// operation `D`, a sheet not at all, and every face, edge and vertex that
/// stays keeps its name.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeleteFace;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeleteFaceArgs {
    /// The faces to take out, all of one solid.
    pub faces: Vec<FaceRef>,
    /// What to leave where they were.
    #[serde(default)]
    pub heal: HealGap,
    /// Take out the whole feature each face picked is part of — a hole, a
    /// boss, a pocket: every face that fills one hole of the faces around
    /// it — not only the faces picked.
    #[serde(default)]
    pub whole_feature: bool,
}

/// What [`DeleteFace`] leaves where the faces were.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HealGap {
    /// The faces around the gap grown until they meet.
    #[default]
    Heal,
    /// Nothing: the solid left open.
    Open,
}

impl HealGap {
    const ALL: [(HealGap, &'static str, &'static str); 2] = [
        (HealGap::Heal, "heal", "Heal the gap"),
        (HealGap::Open, "open", "Leave it open"),
    ];
}

/// Takes out of the bodies `bodies` the topology they do not need: faces
/// that lie on one surface — two blocks united flush leave two in one
/// plane — made one, and vertices between two edges of the same two
/// faces dropped, the edges made one (see
/// [`geop_ops_model::NamedModel::merge_faces_on_one_surface`]). What is
/// made one keeps the name of the oldest of what it is made of, and the
/// others' names stand for it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MergeFaces;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MergeFacesArgs {
    /// The bodies to simplify: solids, or a face of each sheet.
    pub bodies: Vec<ElementReference<NamedModel, Body>>,
}

part_operation!(
    MergeFaces(MergeFacesArgs),
    MoveFace(MoveFaceArgs),
    OffsetFace(OffsetFaceArgs),
    ReplaceFace(ReplaceFaceArgs),
    DeleteFace(DeleteFaceArgs)
);

/// How the direct edits are offered: features that change a solid.
const fn info(label: &'static str, doc: &'static str) -> OperationInfo {
    OperationInfo::new(label, doc, OperationGroup::Features, OperationTier::Menu)
}

impl PartOperation for MoveFace {
    const KIND: &'static str = "move_face";
    const INFO: OperationInfo = info(
        "Move face",
        "Shift faces of a solid, its neighbours trimmed or grown to meet them.",
    )
    .icon(crate::icons::MOVE_FACE);
    type Args = MoveFaceArgs;
    type Session = ();

    /// Nothing picked yet, not shifted.
    fn new_args(_: &Context) -> MoveFaceArgs {
        MoveFaceArgs {
            faces: Vec::new(),
            translation: [0.0; 3],
        }
    }

    fn run(ctx: &mut StepContext, args: &MoveFaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The faces, picked; the shift along each axis, dragged by a gizmo
    /// where the faces' middle is moved to.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &MoveFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, MoveFaceArgs> {
        let before = &context.before;
        let mut f = EditForm::<MoveFaceArgs>::default();
        let shift = Vector3::from_array(args.translation.map(Scalar::from_f64));
        f.form.gizmo =
            faces_middle(before, &args.faces).map(|c| Gizmo::new(c.add(&shift)).translate());
        f.pick(
            "faces",
            "faces",
            before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        for (k, (key, label)) in [("x", "shift x"), ("y", "shift y"), ("z", "shift z")]
            .into_iter()
            .enumerate()
        {
            let number = Number::new(label, args.translation[k], Unit::Length);
            f.number(key, number, move |edit, v| edit.args.translation[k] = v);
        }
        f
    }

    /// The gizmo dragged: the shift moved by as much.
    fn canvas(_: &Context, edit: Edit<'_, MoveFaceArgs, ()>, event: &CanvasEvent) {
        if let CanvasEvent::Gizmo { drag, .. } = event
            && let Change::Translate(by) = &drag.change
        {
            for (k, shift) in edit.args.translation.iter_mut().enumerate() {
                *shift += by[k].to_f64();
            }
        }
    }
}

impl ModelOperation for MoveFace {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &MoveFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("move_face", step)?;
        for v in args.translation {
            finite("shift", v)?;
        }
        let motion =
            Motion::translation(Vector3::from_array(args.translation.map(Scalar::from_f64)));
        let faces = face_ids(model, &args.faces)?;
        replace_faces(model, &namer, &faces, Smooth::Follow, |_, surface| {
            Ok(surface.transform(&motion))
        })?;
        Ok(())
    }
}

impl PartOperation for OffsetFace {
    const KIND: &'static str = "offset_face";
    const INFO: OperationInfo = info(
        "Offset face",
        "Push or pull faces of a solid along their normals, its neighbours trimmed or grown to meet them.",
    )
    .icon(crate::icons::OFFSET_FACE);
    type Args = OffsetFaceArgs;
    type Session = ();

    /// Nothing picked yet, pulled out by one.
    fn new_args(_: &Context) -> OffsetFaceArgs {
        OffsetFaceArgs {
            faces: Vec::new(),
            distance: 1.0,
        }
    }

    fn run(ctx: &mut StepContext, args: &OffsetFaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The faces, picked, and the distance.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &OffsetFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, OffsetFaceArgs> {
        let mut f = EditForm::<OffsetFaceArgs>::default();
        f.pick(
            "faces",
            "faces",
            &context.before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        let distance = Number::new("distance", args.distance, Unit::Length);
        f.number("distance", distance, |edit, v| edit.args.distance = v);
        f
    }
}

impl ModelOperation for OffsetFace {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &OffsetFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("offset_face", step)?;
        finite("distance", args.distance)?;
        if args.distance == 0.0 {
            return Err(GeopError::new(
                "an offset of no distance leaves the faces where they are: give a distance",
            ));
        }
        let distance = Scalar::from_f64(args.distance);
        let faces = face_ids(model, &args.faces)?;
        replace_faces(model, &namer, &faces, Smooth::Follow, |name, surface| {
            surface
                .offset(distance)
                .map_err(|e| e.with_context(format!("offsetting face {name}")))?
                .ok_or_else(|| {
                    GeopError::new(format!(
                        "face {name} collapses offset by {distance:?}: every point of it passes its centre of curvature"
                    ))
                })
        })?;
        Ok(())
    }
}

impl PartOperation for ReplaceFace {
    const KIND: &'static str = "replace_face";
    const INFO: OperationInfo = info(
        "Replace face",
        "Put a face of a solid on the surface of another face, its neighbours trimmed or grown to meet it.",
    )
    .icon(crate::icons::REPLACE_FACE);
    type Args = ReplaceFaceArgs;
    type Session = ();

    /// Nothing picked yet: no face has the empty name.
    fn new_args(_: &Context) -> ReplaceFaceArgs {
        ReplaceFaceArgs {
            face: ElementReference::new(model(), ""),
            with: ElementReference::new(model(), ""),
        }
    }

    fn run(ctx: &mut StepContext, args: &ReplaceFaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The face, and the face whose surface it goes on, picked.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ReplaceFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ReplaceFaceArgs> {
        let before = &context.before;
        let mut f = EditForm::<ReplaceFaceArgs>::default();
        f.pick_one("face", "face", before, Some(&args.face), |edit, face| {
            edit.args.face = face
        });
        f.pick_one("with", "onto", before, Some(&args.with), |edit, with| {
            edit.args.with = with
        });
        f
    }
}

impl ModelOperation for ReplaceFace {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &ReplaceFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("replace_face", step)?;
        let face = model.face_id(args.face.name())?;
        let with = model.face_id(args.with.name())?;
        if face == with {
            return Err(GeopError::new(
                "a face replaced by its own surface stays where it is: pick another face",
            ));
        }
        let topology = model.topology();
        let corners: Vec<Vector3> = (topology.face_edges(face))
            .map(|e| {
                Ok(topology
                    .get_vertex(topology.get_edge(e)?.start_vertex)?
                    .point)
            })
            .collect::<GeopResult<_>>()?;
        let target = topology.get_face(with)?.surface.covering(&corners)?;
        let anchor = face_point(model, face)?;
        replace_faces(model, &namer, &[face], Smooth::Refuse, |name, old| {
            facing_as(&target, old, anchor)
                .map_err(|e| e.with_context(format!("putting face {name} on another surface")))
        })?;
        Ok(())
    }
}

impl PartOperation for MergeFaces {
    const KIND: &'static str = "merge_faces";
    const INFO: OperationInfo = info(
        "Merge faces",
        "Make faces that lie on one surface one face, and drop the edges and vertices between them.",
    )
    .icon(crate::icons::MERGE_FACES);
    type Args = MergeFacesArgs;
    type Session = ();

    /// The newest solid.
    fn new_args(context: &Context) -> MergeFacesArgs {
        MergeFacesArgs {
            bodies: (read_model(&context.before).ok())
                .and_then(|model| model.newest_solid())
                .map(|name| ElementReference::new(model(), name))
                .into_iter()
                .collect(),
        }
    }

    fn run(ctx: &mut StepContext, args: &MergeFacesArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The bodies, picked.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &MergeFacesArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, MergeFacesArgs> {
        let mut f = EditForm::<MergeFacesArgs>::default();
        f.pick(
            "bodies",
            "bodies",
            &context.before,
            &args.bodies,
            true,
            |edit, picked| edit.args.bodies = picked,
        );
        f
    }
}

impl ModelOperation for MergeFaces {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        _: &str,
        args: &MergeFacesArgs,
    ) -> GeopResult<()> {
        for body in resolve_bodies(reader, &args.bodies)? {
            model.merge_faces_on_one_surface(body, |_, _| true)?;
        }
        Ok(())
    }
}

impl PartOperation for DeleteFace {
    const KIND: &'static str = "delete_face";
    const INFO: OperationInfo = info(
        "Delete face",
        "Take faces out of a solid: holes, bosses, chamfers and fillets healed over by the faces around them, or the solid left open.",
    )
    .icon(crate::icons::DELETE_FACE);
    type Args = DeleteFaceArgs;
    type Session = ();

    /// Nothing picked yet, healed over.
    fn new_args(_: &Context) -> DeleteFaceArgs {
        DeleteFaceArgs {
            faces: Vec::new(),
            heal: HealGap::Heal,
            whole_feature: false,
        }
    }

    fn run(ctx: &mut StepContext, args: &DeleteFaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The faces, picked, and what to leave.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &DeleteFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, DeleteFaceArgs> {
        let mut f = EditForm::<DeleteFaceArgs>::default();
        f.pick(
            "faces",
            "faces",
            &context.before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        let (heal, heals) = choices(&HealGap::ALL, &args.heal);
        f.select("heal", "leave", heal, heals, false, |edit, value| {
            if let Some(heal) = chosen(&HealGap::ALL, value) {
                edit.args.heal = heal;
            }
        });
        f.checkbox(
            "whole_feature",
            "whole feature",
            args.whole_feature,
            |edit, b| edit.args.whole_feature = b,
        );
        f
    }
}

impl ModelOperation for DeleteFace {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &DeleteFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("delete_face", step)?;
        let faces = face_ids(model, &args.faces)?;
        let heal = match args.heal {
            HealGap::Heal => Heal::Extend,
            HealGap::Open => Heal::Open,
        };
        delete_faces(model, &namer, &faces, args.whole_feature, heal)?;
        Ok(())
    }
}

/// A point of the face `face` of `part` and where it is on the face's
/// surface: where its first coedge starts.
fn face_point(part: &NamedModel, face: FaceId) -> GeopResult<(Vector3, (Scalar, Scalar))> {
    let model = part.topology();
    let Some(coedge) = model.iterate_face_coedges(face).next() else {
        return Err(GeopError::new(format!(
            "face {} has no boundary",
            part.name(face)?
        )));
    };
    let uv = model.get_coedge(coedge)?.pcurve.ends()?.0;
    let surface = &model.get_face(face)?.surface;
    Ok((surface.evaluate(uv[0], uv[1])?, (uv[0], uv[1])))
}

/// `surface`, turned around if need be so that its normal points the way
/// `old`'s does at `anchor` — a point of the face on `old`, and where it is
/// on it — where `surface` comes nearest it. Refused where which way it
/// points cannot be told: the two square to each other there.
fn facing_as(
    surface: &NurbSurface3D,
    old: &NurbSurface3D,
    (point, (u, v)): (Vector3, (Scalar, Scalar)),
) -> GeopResult<NurbSurface3D> {
    let normal = old.normal(u, v)?;
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let half = Scalar::from_f64(0.5);
    let (su, sv) = surface.project_in_domain(
        point,
        Scalar::interpolate(u0, u1, half).sharpen(),
        Scalar::interpolate(v0, v1, half).sharpen(),
        PROJECT_ITERATIONS,
    )?;
    let along = surface
        .normal(su.sharpen(), sv.sharpen())?
        .prod_dot(&normal);
    if along.definitely_greater(Scalar::ZERO) {
        Ok(surface.clone())
    } else if along.definitely_less(Scalar::ZERO) {
        Ok(surface.reverse_u())
    } else {
        Err(GeopError::new(format!(
            "the new surface is square to the face at {point:?}: which way it faces cannot be told"
        )))
    }
}
