//! [`ProjectCurve`]: project a sketch's curves onto a face, along the
//! sketch's normal or each point to its nearest on the face, dividing the
//! face along them.

use geop_core_editor::{
    edit_form::{EditForm, FormState, choices, chosen},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_core_topology::{
    Body, FaceId, Sense, ShellId,
    build::{BodySpec, CoedgeOn, EdgeSpec, FaceSpec},
};
use geop_lib_booleans::{RemeshParams, imprint_on_face};
use geop_lib_sweep::{SweepLoop, extrude};
use geop_ops_model::sketch::{PlacedSketch, SketchRef, shape_profiles};
use geop_ops_model::{
    BodyNames, Chain, Face, ModelOperation, NamedModel, Namer, build_model, model,
};
use geop_ops_model::{face_hull, spread_along};
use serde::{Deserialize, Serialize};

/// Projects the curves of the sketch named `sketch` onto the face named
/// `face`, and divides the face along what they project to: each curve the
/// face is divided along becomes an edge of it, and the face splits into a
/// piece on either side of a curve that crosses it, or a piece inside one
/// that closes on it.
///
/// The curves are what an extrude as a face would sweep: the outline of the
/// sketch's area, or its one chain enclosing nothing. Projected
/// [`Along::SketchNormal`], their projection is where that sweep, reaching
/// past the face both ways, crosses the face — on a face that bends back
/// over itself, every place it does. Projected [`Along::FaceNormal`], each
/// point goes to the point of the face's surface nearest it (see
/// [`normal_sheets`]).
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
    pub face: ElementReference<NamedModel, Face>,
    /// Which way they are projected.
    #[serde(default, skip_serializing_if = "Along::is_sketch_normal")]
    pub along: Along,
}

/// Which way [`ProjectCurve`] projects a sketch's curves onto a face.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Along {
    /// Along the sketch plane's normal, as an extrude sweeps them.
    #[default]
    SketchNormal,
    /// Each point to the point of the face's surface nearest it: along the
    /// face's normal there.
    FaceNormal,
}

impl Along {
    fn is_sketch_normal(&self) -> bool {
        *self == Along::SketchNormal
    }

    /// Each way, as the dialog offers it: `(way, value, label)`.
    const OPTIONS: [(Along, &'static str, &'static str); 2] = [
        (Along::SketchNormal, "sketch_normal", "the sketch's normal"),
        (Along::FaceNormal, "face_normal", "the face's normal"),
    ];
}

part_operation!(ProjectCurve(ProjectCurveArgs));

impl PartOperation for ProjectCurve {
    const KIND: &'static str = "project_curve";
    const INFO: OperationInfo = OperationInfo::new(
        "Project curve",
        "Project a sketch's curves onto a face, along the sketch's normal or the face's, dividing the face along them.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::PROJECT_CURVE);
    type Args = ProjectCurveArgs;
    type Session = ();

    /// The newest sketch, onto no face yet: no face has the empty name.
    fn new_args(context: &Context) -> ProjectCurveArgs {
        let newest = context.before.targets_of::<PlacedSketch>().pop();
        ProjectCurveArgs {
            sketch: SketchRef::whole(newest.unwrap_or_else(|| geop_ops_model::sketch::sketch(""))),
            face: ElementReference::new(model(), ""),
            along: Along::SketchNormal,
        }
    }

    fn run(ctx: &mut StepContext, args: &ProjectCurveArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sketch and the face, picked, and which way to project.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ProjectCurveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ProjectCurveArgs> {
        let before = &context.before;
        let mut f = EditForm::<ProjectCurveArgs>::default();
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
        let (way, options) = choices(&Along::OPTIONS, &args.along);
        f.select("along", "along", way, options, false, |edit, way| {
            if let Some(along) = chosen(&Along::OPTIONS, way) {
                edit.args.along = along;
            }
        });
        f
    }
}

impl ModelOperation for ProjectCurve {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &ProjectCurveArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("project", step)?;
        let placed = reader.retrieve_element(&args.sketch)?;
        let face = model.face_id(args.face.name())?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose()?;
        let shape = sketch.shape()?;
        let loops: Vec<_> = (shape_profiles(&placed.name, sketch, &geometry, shape)?.into_iter())
            .map(SweepLoop::plain)
            .collect();
        let shells = match args.along {
            Along::SketchNormal => {
                let (from, to) = reach_across(model, face, &placed.plane)?;
                let (from, to) = (Scalar::from_f64(from), Scalar::from_f64(to));
                extrude(model, &namer, None, &placed.plane, &[from, to], &loops)?.shells
            }
            Along::FaceNormal => {
                let plane = &placed.plane;
                let mut chains = Vec::with_capacity(loops.len());
                for lp in &loops {
                    let curves = (lp.profile.curves.iter())
                        .map(|c| c.embed(plane.origin(), plane.u(), plane.v()))
                        .collect::<GeopResult<_>>()?;
                    chains.push(Chain {
                        name: lp.profile.name.clone(),
                        curves,
                        curve_names: lp.profile.curve_names.clone(),
                        joint_names: lp.profile.joint_names.clone(),
                    });
                }
                normal_sheets(model, &namer, face, &chains)?
            }
        };
        let params = RemeshParams::default();
        let tools: Vec<Body> = shells.into_iter().map(Body::Sheet).collect();
        imprint_on_face(model, &namer, face, &tools, params)?;
        Ok(())
    }
}

/// Points of a curve, per knot span of it, at which [`normal_sheets`] finds
/// where the curve is nearest the face.
const FEET_PER_KNOT_SPAN: usize = 8;

/// The sheets whose crossing with `face` is the projection of `chains`
/// onto it, each point to the point of the face's surface nearest it: per
/// chain a sheet of ruled faces, one per curve, through the line along the
/// surface's normal at each foot point, reaching a sixteenth of the face's
/// extent either side of it — a short reach, so that the sheet crosses
/// the face there and nowhere else.
///
/// No NURBS is the projection of a curve onto a curved surface: the rails
/// of each face are cubics through the ends of those lines, at the curve's
/// own parameters of the points they were found for — `FEET_PER_KNOT_SPAN`
/// per knot span, free choices — and the projection is where the sheet
/// crosses the face, exactly. The feet at a joint of a chain are found once,
/// so the faces either side share the line there.
///
/// Named after `namer`, each face after the curve `X` it is projected from,
/// `N(X)`, as an extrude names its walls; its rails `N(X,near)` and
/// `N(X,far)`, the line at joint `P` `N(P)` and its ends `N(P,near)` and
/// `N(P,far)`.
fn normal_sheets(
    model: &mut NamedModel,
    namer: &Namer,
    face: FaceId,
    chains: &[Chain<NurbCurve3D>],
) -> GeopResult<Vec<ShellId>> {
    let surface = model.topology().get_face(face)?.surface.clone();
    let hull = face_hull(model.topology(), &[face])?;
    let (lo, hi) = plain::bounds(hull.iter().map(|p| p.to_f64_array()))
        .ok_or_else(|| GeopError::new(format!("face {face:?} has no control points")))?;
    let reach = Scalar::from_f64(plain::diagonal(lo, hi) / 16.0);
    // Either end of the line along the normal at the point nearest `p`.
    let ends = |p: &Vector3| -> GeopResult<[Vector3; 2]> {
        let (u, v, foot) = surface.closest_point(p)?;
        let normal = surface.normal(u, v)?.prod_scalar(reach);
        Ok([foot.add(&normal), foot.sub(&normal)])
    };
    let mut shells = Vec::with_capacity(chains.len());
    for chain in chains {
        let n = chain.curves.len();
        let closed = chain.is_closed();
        let joints = chain.joints()?;
        let mut spec = BodySpec::new(false);
        let mut names = BodyNames::default();
        for (p, name) in joints.iter().zip(&chain.joint_names) {
            let [near, far] = ends(p)?;
            spec.vertices.extend([near, far]);
            names
                .vertices
                .extend(["near", "far"].map(|end| namer.name(&[name, end])));
            spec.edges.push(EdgeSpec {
                curve: NurbCurve3D::line(near, far)?,
                start: spec.vertices.len() - 2,
                end: spec.vertices.len() - 1,
            });
            names.edges.push(namer.name(&[name]));
        }
        let joint = |k: usize| if closed { k % n } else { k };
        for (k, (curve, name)) in chain.curves.iter().zip(&chain.curve_names).enumerate() {
            let (a, b) = (joint(k), joint(k + 1));
            let (t0, t1) = curve.domain();
            let count = FEET_PER_KNOT_SPAN * curve.breakpoints().len().saturating_sub(1).max(1);
            let mut rails: [Vec<Vector4>; 2] = Default::default();
            let mut params = Vec::with_capacity(count + 1);
            for i in 0..=count {
                // Where along the curve is a free choice: sharp.
                let f = Scalar::from_f64(i as f64 / count as f64);
                let at = if i == 0 {
                    [spec.vertices[2 * a], spec.vertices[2 * a + 1]]
                } else if i == count {
                    [spec.vertices[2 * b], spec.vertices[2 * b + 1]]
                } else {
                    ends(&curve.evaluate(Scalar::interpolate(t0, t1, f).sharpen())?)?
                };
                for (rail, p) in rails.iter_mut().zip(at) {
                    rail.push(p.homogeneous(Scalar::ONE));
                }
                params.push(f);
            }
            let [near, far] =
                rails.map(|rail| NurbCurve3D::interpolate_homogeneous(&rail, &params, 3));
            let (near, far) = (near?, far?);
            let surface = NurbSurface3D::ruled(&near, &far)?;
            for (rail, (from, to), end) in [
                (near, (2 * a, 2 * b), "near"),
                (far, (2 * a + 1, 2 * b + 1), "far"),
            ] {
                spec.edges.push(EdgeSpec {
                    curve: rail,
                    start: from,
                    end: to,
                });
                names.edges.push(namer.name(&[name, end]));
            }
            let rail = spec.edges.len() - 2;
            spec.faces.push(FaceSpec::on_unit_square(
                surface,
                [
                    CoedgeOn::Edge(rail, Sense::Forward),
                    CoedgeOn::Edge(b, Sense::Forward),
                    CoedgeOn::Edge(rail + 1, Sense::Reversed),
                    CoedgeOn::Edge(a, Sense::Reversed),
                ],
            )?);
            names.faces.push(namer.name(&[name]));
        }
        spec.shells.push((0..spec.faces.len()).collect());
        shells.extend(model.build_body(spec, names)?.shells);
    }
    Ok(shells)
}

/// How far along `plane`'s normal, back and forth from the plane, a sweep
/// has to reach to cross all of `face`: past the convex hull of its
/// surface's control points — a NURBS surface lies within it — by the
/// hull's diagonal both ways, so the sweep's ends are clear of the face.
fn reach_across(
    model: &NamedModel,
    face: FaceId,
    plane: &CoordinateSystem,
) -> GeopResult<(f64, f64)> {
    let hull = face_hull(model.topology(), &[face])?;
    let (near, far) = spread_along(&hull, plane);
    let (lo, hi) = plain::bounds(hull.iter().map(|p| p.to_f64_array()))
        .ok_or_else(|| GeopError::new(format!("face {face:?} has no control points")))?;
    let diagonal = plain::diagonal(lo, hi);
    Ok((near - diagonal, far + diagonal))
}
