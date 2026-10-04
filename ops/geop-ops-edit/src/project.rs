//! [`ProjectCurve`]: project a sketch's curves onto a face, dividing the
//! face along them.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    with_context,
};
use geop_core_topology::FaceId;
use geop_ops::{
    Context, EntityRef, Library, Namer, Part,
    operation::{Operation, Role},
    ui::Form,
};
use geop_ops_booleans::{imprint::imprint_on_face, remesh::remesh::RemeshParams};
use geop_ops_extrude_revolve::{extrude::extrude, operation::shape_loops};
use serde::{Deserialize, Serialize};

use crate::{face_name, face_ref};

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

impl Operation for ProjectCurve {
    type Args = ProjectCurveArgs;
    type Session = ();

    /// The newest sketch, onto no face yet.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> ProjectCurveArgs {
        ProjectCurveArgs {
            sketch: before.sketch_names().pop().unwrap_or_default(),
            face: String::new(),
        }
    }

    /// The sketch and the face, picked.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ProjectCurveArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ProjectCurveArgs> {
        let mut f = Form::<S, ProjectCurveArgs>::new();
        let sketch = if args.sketch.is_empty() {
            Vec::new()
        } else {
            vec![EntityRef::Sketch {
                name: args.sketch.clone(),
            }]
        };
        f.reference(
            "sketch",
            "sketch",
            sketch,
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
            "face",
            "onto face",
            face_ref(&args.face),
            &[Role::Face],
            None,
            false,
            |e, p| e.args.face = face_name(&p),
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ProjectCurveArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("project_curve({operation_id}, {args:?})");
        let namer = Namer::new("project", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let face = part.face_id(&args.face).with_context(ctx)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let loops = shape_loops(
            &args.sketch,
            sketch,
            &geometry,
            sketch.shape().with_context(ctx)?,
        )
        .with_context(ctx)?;
        let (from, to) = reach_across(&part, face, &placed.plane).with_context(ctx)?;
        let swept = extrude(
            &mut part,
            &namer,
            None,
            &placed.plane,
            S::from_f64(from),
            S::from_f64(to),
            &loops,
        )
        .with_context(ctx)?;
        imprint_on_face(
            &mut part,
            &namer,
            face,
            &swept.shells,
            RemeshParams::default(),
        )
        .with_context(ctx)?;
        Ok(part)
    }
}

/// How far along `plane`'s normal, back and forth from the plane, a sweep
/// has to reach to cross all of `face`: past the convex hull of its
/// surface's control points — a NURBS surface lies within it — by the
/// hull's diagonal both ways, so the sweep's ends are clear of the face.
fn reach_across<S: Scalar>(
    part: &Part<S>,
    face: FaceId,
    plane: &CoordinateSystem<S>,
) -> GeopResult<(f64, f64)> {
    let surface = &part.topology().get_face(face)?.surface;
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
