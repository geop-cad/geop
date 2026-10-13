//! [`ProjectCurve`]: project a sketch's curves onto a face, dividing the
//! face along them.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain,
    primitives::CoordinateSystem,
    scalars::Scalar,
};
use geop_core_program::element::ElementReference;
use geop_core_topology::FaceId;
use geop_ops_booleans::{
    extent::{face_hull, spread_along},
    imprint::imprint_on_face,
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::{extrude::extrude, operation::shape_loops};
use geop_ops_model::{Design, Face, ModelOperation, NamedModel, Namer, build_model, model};
use geop_ops_parameters::{BuildPart, PartReader, StateValue, StepContext};
use geop_ops_sketch::{PlacedSketch, SketchRef};
use serde::{Deserialize, Serialize};

/// Projects the curves of the sketch named `sketch` along its plane's
/// normal onto the face named `face`, and divides the face along what they
/// project to: each curve the face is divided along becomes an edge of it,
/// and the face splits into a piece on either side of a curve that crosses
/// it, or a piece inside one that closes on it.
///
/// The curves are what an extrude as a face would sweep: the outline of the
/// sketch's area, or its one chain enclosing nothing. Their projection is
/// where that sweep, reaching past the face both ways, crosses the face —
/// on a face that bends back over itself, every place it does.
///
/// Named as [`imprint_on_face`] names what it creates, with `N` being
/// `project(P)` for the operation `P`: an edge is named after the face's
/// copy `N(copy,F)` and the swept wall `N(K,X)` of the piece `X` of a curve
/// of the sketch `K` it lies on, the pieces split off the face `F` along
/// an edge `C` are `N(F,C)`, and the face's boundary edges split where a
/// curve ends on them are named as a boolean names an edge split in two.
///
/// Fails unless every curve projected onto the face runs from boundary to
/// boundary of it, or closes on itself: a curve ending inside the face
/// divides nothing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ProjectCurve;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProjectCurveArgs {
    /// The sketch whose curves to project.
    pub sketch: SketchRef,
    /// The face to project them onto.
    pub face: ElementReference<NamedModel<Design>, Face>,
}

part_operation!(ProjectCurve(ProjectCurveArgs));

impl PartOperation for ProjectCurve {
    const KIND: &'static str = "project_curve";
    const INFO: OperationInfo = OperationInfo::new(
        "Project curve",
        "Project a sketch's curves along its plane's normal onto a face, dividing the face along them.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::PROJECT_CURVE);
    type Args = ProjectCurveArgs;
    type Input = Design;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    /// The newest sketch, onto no face yet: no face has the empty name.
    fn new_args(context: &StepContext) -> ProjectCurveArgs {
        let newest = context.before.targets_of::<PlacedSketch<Design>>().pop();
        ProjectCurveArgs {
            sketch: SketchRef::whole(newest.unwrap_or_else(|| geop_ops_sketch::sketch(""))),
            face: ElementReference::new(model(), ""),
        }
    }

    fn run(step: &str, part: &mut BuildPart, args: &ProjectCurveArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The sketch and the face, picked.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &ProjectCurveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ProjectCurveArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, ProjectCurveArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            before,
            Some(&args.sketch),
            |edit, sketch| edit.args.sketch = sketch,
        )
        .pick_one(
            "face",
            "onto face",
            before,
            Some(&args.face),
            |edit, face| edit.args.face = face,
        );
        f
    }
}

impl ModelOperation for ProjectCurve {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &PartReader,
        step: &str,
        args: &ProjectCurveArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("project", step)?;
        let placed = reader.retrieve_element(&args.sketch)?;
        let face = model.face_id(args.face.name())?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<Design>()?;
        let shape = sketch.shape()?;
        let loops = shape_loops(&placed.name, sketch, &geometry, shape)?;
        let (from, to) = reach_across(model, face, &placed.plane)?;
        let (from, to) = (Design::from_f64(from), Design::from_f64(to));
        let swept = extrude(model, &namer, None, &placed.plane, from, to, &loops)?;
        let params = RemeshParams::default();
        imprint_on_face(model, &namer, face, &swept.shells, params)?;
        Ok(())
    }
}

/// How far along `plane`'s normal, back and forth from the plane, a sweep
/// has to reach to cross all of `face`: past the convex hull of its
/// surface's control points — a NURBS surface lies within it — by the
/// hull's diagonal both ways, so the sweep's ends are clear of the face.
fn reach_across<S: Scalar>(
    model: &NamedModel<S>,
    face: FaceId,
    plane: &CoordinateSystem<S>,
) -> GeopResult<(f64, f64)> {
    let hull = face_hull(model.topology(), &[face])?;
    let (near, far) = spread_along(&hull, plane);
    let (lo, hi) = plain::bounds(hull.iter().copied())
        .ok_or_else(|| GeopError::new(format!("face {face:?} has no control points")))?;
    let diagonal = plain::diagonal(lo, hi);
    Ok((near - diagonal, far + diagonal))
}
