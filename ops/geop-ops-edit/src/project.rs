//! [`ProjectCurve`]: project a sketch's curves onto a face, dividing the
//! face along them.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_core_topology::FaceId;
use geop_ops::{
    Design, EntityRef, NamedModel, Namer, ParamValue,
    operation::{Build, BuildReader, Input, Lookup, ModelStep, Role, Scope, Seen, build_model},
};
use geop_ops_booleans::{imprint::imprint_on_face, remesh::remesh::RemeshParams};
use geop_ops_extrude_revolve::{extrude::extrude, operation::shape_loops};
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

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ProjectCurveArgs {
    /// The sketch whose curves to project.
    pub sketch: String,
    /// The face to project them onto.
    pub face: String,
}

impl ModelStep for ProjectCurve {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &ProjectCurveArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("project_curve({step}, {args:?})");
        let namer = Namer::new("project", step)?;
        let placed = (Scope { model, reader })
            .sketch(&args.sketch)
            .with_context(ctx)?
            .clone();
        let face = model.face_id(&args.face).with_context(ctx)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<Design>().with_context(ctx)?;
        let shape = sketch.shape().with_context(ctx)?;
        let loops = shape_loops(&args.sketch, sketch, &geometry, shape).with_context(ctx)?;
        let (from, to) = reach_across(model, face, &placed.plane).with_context(ctx)?;
        let (from, to) = (Design::from_f64(from), Design::from_f64(to));
        let swept =
            extrude(model, &namer, None, &placed.plane, from, to, &loops).with_context(ctx)?;
        let params = RemeshParams::default();
        imprint_on_face(model, &namer, face, &swept.shells, params).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for ProjectCurve {
    const INFO: OperationInfo = OperationInfo::new(
        "project_curve",
        "Project curve",
        "Project a sketch's curves along its plane's normal onto a face, dividing the face along them.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental();
    type Args = ProjectCurveArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest sketch, onto no face yet.
    fn new_args(context: &Context<Input, ParamValue>) -> ProjectCurveArgs {
        ProjectCurveArgs {
            sketch: (Seen::new(context.before).sketch_names().pop()).unwrap_or_default(),
            face: String::new(),
        }
    }

    fn run(step: &str, part: &mut Build, args: &ProjectCurveArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The sketch and the face, picked.
    fn presentation(
        _: &Context<Input, ParamValue>,
        args: &ProjectCurveArgs,
        _: &(),
    ) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let sketch = EntityRef::of_names(EntityRef::sketch, std::slice::from_ref(&args.sketch));
        dialog.reference("sketch", "sketch", sketch, &[Role::Sketch], None, false);
        let face = EntityRef::of_names(EntityRef::face, std::slice::from_ref(&args.face));
        dialog.reference("face", "onto face", face, &[Role::Face], None, false);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &ProjectCurveArgs,
        _: &(),
        event: Event<Design>,
    ) -> (ProjectCurveArgs, ()) {
        let mut args = args.clone();
        let Event::Field {
            key,
            value: Value::Entities(picked),
        } = event
        else {
            return (args, ());
        };
        let one = |kind| EntityRef::names_of(kind, &picked).pop().unwrap_or_default();
        match key.as_str() {
            "sketch" => args.sketch = one(EntityRef::sketch),
            "face" => args.face = one(EntityRef::face),
            _ => {}
        }
        (args, ())
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
    let surface = &model.topology().get_face(face)?.surface;
    let o = [0, 1, 2].map(|k| plane.origin()[k].to_f64());
    let w = [0, 1, 2].map(|k| plane.w()[k].to_f64());
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let (mut near, mut far) = (f64::INFINITY, f64::NEG_INFINITY);
    for cp in &surface.control_points {
        let weight = cp[3].to_f64();
        let p = [0, 1, 2].map(|k| cp[k].to_f64() / weight);
        let along = (0..3).map(|k| (p[k] - o[k]) * w[k]).sum::<f64>();
        near = near.min(along);
        far = far.max(along);
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diagonal = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    Ok((near - diagonal, far + diagonal))
}
