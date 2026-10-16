//! [`SheetCut`]: holes and notches cut through a sheet-metal body along a
//! sketch, across its bends unrolled.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    element::ElementReference, part::StepContext, reader::Reader, values::Design,
};
use geop_core_sketch::Shape;
use geop_ops_model::{ModelOperation, NamedModel, Namer, Plane, read_model};
use geop_ops_sketch::shape_profiles;
use geop_ops_sketch::{PlacedSketch, SketchRef};
use serde::{Deserialize, Serialize};

use crate::{
    body::{self, Made, Making, SheetBody, SheetStep, sheet_body},
    sheet::{Cut, FlatEdge, Placement, Sheet},
};

/// Cuts through a sheet-metal body along a sketch, for the operation `C`:
/// the body is consumed and rebuilt as `sheet_cut(C)`, every face, edge and
/// vertex it had keeping its name except those the cut splits or takes
/// away.
///
/// Every area of `sketch` is projected onto the flat face `face`, along
/// its normal, and cut out straight through the sheet: a hole where it
/// lies inside the face, a notch where it reaches over the face's outline,
/// and where it reaches past the face into a bend — and on into the next
/// flat — it is cut through the bend as the bend lies unrolled, so that
/// the flat pattern shows it exactly as drawn. The cut is recorded with the
/// body's bends: the body still unfolds.
///
/// The edges the cut makes are named after the sketch's pieces: piece `X`
/// of sketch `K` gives `sheet_cut(C,K,X)` — `...,0`, `...,1` along it where
/// the body's edges split it — with its edges `...,a` and `...,b` and its
/// wall as [`crate::thicken`] says. An edge of the body the cut splits
/// becomes `E,0`, `E,1`, ... along it.
///
/// Refused, by name: a sketch not parallel to the face, an area with a
/// hole in it (whose island would fall out), a cut that crosses an edge at
/// its end, touches one or runs along one, crosses a curve that is no line
/// or arc, splits a face in two or takes one away, or misses the sheet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SheetCut;

/// A flat face of the part, by name.
pub type FlatFaceRef = ElementReference<NamedModel<Design>, Plane>;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SheetCutArgs {
    /// The sketch: one area or several, none with a hole.
    pub sketch: SketchRef,
    /// A flat face of a sheet-metal body — either side — parallel to the
    /// sketch. None: the flat face the sketch lies on.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<FlatFaceRef>,
}

impl SheetStep for SheetCut {
    fn plan(reader: &Reader, step: &str, args: &SheetCutArgs) -> GeopResult<Made> {
        let ctx = with_context!("sheet_cut({step}, {args:?})");
        let namer = Namer::new("sheet_cut", step)?;
        let model = read_model(reader).with_context(ctx)?;
        let placed = reader.retrieve_element(&args.sketch).with_context(ctx)?;
        let face = args.face.as_ref().map(ElementReference::name);
        let (solid, mut sheet, f) =
            target(&model, reader, face, &placed.plane).with_context(ctx)?;
        let loops = cut_loops(&namer, placed, &sheet.flats[f].place).with_context(ctx)?;
        sheet.cuts.push(Cut {
            flat: f,
            loops,
            name: namer.clone(),
        });
        Ok(Made {
            consumes: Some(solid),
            solid: namer.root(),
            making: Making::Bent(sheet),
        })
    }
}

part_operation!(SheetCut(SheetCutArgs));

impl PartOperation for SheetCut {
    const KIND: &'static str = "sheet_cut";
    const INFO: OperationInfo = OperationInfo::new(
        "Sheet cut",
        "Cut holes and notches through a sheet-metal body along a sketch, across its bends unrolled.",
        OperationGroup::SheetMetal,
        OperationTier::Small,
    )
    .icon(crate::icons::SHEET_CUT);
    type Args = SheetCutArgs;
    type Session = ();

    /// The newest sketch, on the face it lies on.
    fn new_args(context: &Context) -> SheetCutArgs {
        let newest = context.before.targets_of::<PlacedSketch<Design>>().pop();
        SheetCutArgs {
            sketch: SketchRef::whole(newest.unwrap_or_else(|| geop_ops_sketch::sketch(""))),
            face: None,
        }
    }

    fn run(ctx: &mut StepContext, args: &SheetCutArgs) -> GeopResult<()> {
        let _step = ctx.step();
        body::run::<Self>(ctx, args)
    }

    /// The sketch, picked, and the face, if not the one the sketch lies on.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &SheetCutArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, SheetCutArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, SheetCutArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            before,
            Some(&args.sketch),
            |edit, sketch| edit.args.sketch = sketch,
        );
        let face = args.face.as_slice();
        f.pick(
            "face",
            "onto face",
            before,
            face,
            false,
            |edit, mut face| edit.args.face = face.pop(),
        );
        // The face the sketch lies on, unless one is picked.
        f.form.dialog.optional("face");
        f
    }
}

impl ModelOperation for SheetCut {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        _: &SheetCutArgs,
    ) -> GeopResult<()> {
        body::build(model, reader, step)
    }
}

/// The sheet-metal body to cut, its sheet, and the flat the sketch on the
/// plane `plane` is projected onto: the one with the face `face`, or, if
/// none is named, the one the plane lies on.
fn target(
    model: &NamedModel<Design>,
    reader: &Reader,
    face: Option<&str>,
    plane: &CoordinateSystem<Design>,
) -> GeopResult<(String, Sheet<Design>, usize)> {
    type S = Design;
    let parallel = |place: &Placement<S>| {
        plane
            .w()
            .prod_cross(&place.n)
            .norm()
            .could_be_equal(S::ZERO)
    };
    let mut found = Vec::new();
    for solid in model.solid_names() {
        let Some(sheet) = sheet_body(reader, &solid).and_then(SheetBody::bent) else {
            continue;
        };
        for (f, flat) in sheet.flats.iter().enumerate() {
            let Some(face) = face else {
                // On the A side's plane or the B side's.
                let t = S::from_f64(sheet.rules.thickness);
                let on = |place: &Placement<S>| {
                    parallel(place)
                        && plane
                            .origin()
                            .sub(&place.origin)
                            .prod_dot(&place.n)
                            .could_be_equal(S::ZERO)
                };
                if on(&flat.place) || on(&flat.place.offset(t)) {
                    found.push((solid.clone(), sheet.clone(), f));
                }
                continue;
            };
            if flat.name.name(&["a"]) == face || flat.name.name(&["b"]) == face {
                if !parallel(&flat.place) {
                    return Err(GeopError::new(format!(
                        "the sketch's plane is not parallel to face {face}: a sketch is cut through a sheet along the normal of a flat face it is parallel to"
                    )));
                }
                return Ok((solid.clone(), sheet.clone(), f));
            }
        }
    }
    if let Some(face) = face {
        return Err(GeopError::new(format!(
            "{face:?} is no flat face of a sheet-metal body: a cut is projected onto one of those"
        )));
    }
    match found.len() {
        1 => Ok(found.remove(0)),
        0 => Err(GeopError::new(
            "the sketch lies on no flat face of a sheet-metal body: pick the face to project it onto",
        )),
        _ => Err(GeopError::new(format!(
            "the sketch lies on {} flat faces of sheet-metal bodies: pick the face to project it onto",
            found.len()
        ))),
    }
}

/// The areas of the sketch `placed` projected into the sheet coordinates of
/// the flat at `place`: each a loop counter-clockwise there, its curves
/// named `C,K,X`.
fn cut_loops<S: Scalar>(
    namer: &Namer,
    placed: &PlacedSketch<S>,
    place: &Placement<S>,
) -> GeopResult<Vec<Vec<FlatEdge<S>>>> {
    let (name, sketch) = (&placed.name, &placed.sketch);
    let geometry = sketch.enclose::<S>()?;
    let plane = &placed.plane;
    // Sketch coordinates to sheet coordinates: affine, so exact on the
    // homogeneous control points.
    let d = plane.origin().sub(&place.origin);
    let row = |e: &Vector3<S>| [plane.u().prod_dot(e), plane.v().prod_dot(e), d.prod_dot(e)];
    let m = [row(&place.e1), row(&place.e2)];
    let flips = m[0][0]
        .mul(m[1][1])
        .sub(m[0][1].mul(m[1][0]))
        .definitely_less(S::ZERO);
    let map = |curve: &NurbCurve2D<S>| -> GeopResult<NurbCurve2D<S>> {
        let control_points = curve
            .control_points
            .iter()
            .map(|cp| {
                let image = |r: &[S; 3]| r[0].mul(cp[0]).add(r[1].mul(cp[1])).add(r[2].mul(cp[2]));
                Vector3::from_array([image(&m[0]), image(&m[1]), cp[2]])
            })
            .collect();
        let mapped = NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())?;
        Ok(if flips { mapped.reverse() } else { mapped })
    };
    let mut loops = Vec::new();
    for region in sketch.regions()? {
        if !region.holes.is_empty() {
            return Err(GeopError::new(format!(
                "an area of sketch {name} has a hole in it: cut out, the island inside would fall out — draw the area without it"
            )));
        }
        for lp in shape_profiles(name, sketch, &geometry, Shape::Region(region))? {
            let mut edges = lp
                .curves
                .iter()
                .zip(&lp.curve_names)
                .map(|(curve, piece)| {
                    Ok(FlatEdge {
                        curve: map(curve)?,
                        name: namer.scoped(piece),
                    })
                })
                .collect::<GeopResult<Vec<_>>>()?;
            if flips {
                edges.reverse();
            }
            loops.push(edges);
        }
    }
    Ok(loops)
}
