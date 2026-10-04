//! [`SheetCut`]: holes and notches cut through a sheet-metal body along a
//! sketch, across its bends unrolled.

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::Shape;
use geop_core_topology::Body;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Tone},
};
use geop_ops_extrude_revolve::operation::shape_loops;
use serde::{Deserialize, Serialize};

use crate::{
    sheet::{Cut, FlatEdge, Placement, Sheet},
    thicken::thicken,
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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SheetCutArgs {
    /// The sketch: one area or several, none with a hole.
    pub sketch: String,
    /// A flat face of a sheet-metal body — either side — parallel to the
    /// sketch. Empty: the flat face the sketch lies on.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub face: String,
}

impl Operation for SheetCut {
    type Args = SheetCutArgs;
    type Session = ();

    /// The newest sketch, on the face it lies on.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> SheetCutArgs {
        SheetCutArgs {
            sketch: before.sketch_names().pop().unwrap_or_default(),
            face: String::new(),
        }
    }

    /// The sketch, picked, and the face, if not the one the sketch lies on.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &SheetCutArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, SheetCutArgs> {
        let mut f = Form::<S, SheetCutArgs>::new();
        if context.before.sketches().next().is_none() {
            f.text("sketch", "No sketch yet — add one first.", Tone::Hint);
        } else {
            let sketch = (!args.sketch.is_empty())
                .then(|| EntityRef::Sketch {
                    name: args.sketch.clone(),
                })
                .into_iter()
                .collect();
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
        }
        let face = (!args.face.is_empty())
            .then(|| EntityRef::Face {
                name: args.face.clone(),
            })
            .into_iter()
            .collect();
        f.reference(
            "face",
            "onto face",
            face,
            &[Role::Plane],
            None,
            false,
            |e, picked| {
                e.args.face = match picked.as_slice() {
                    [EntityRef::Face { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &SheetCutArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("sheet_cut({operation_id}, {args:?})");
        let namer = Namer::new("sheet_cut", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let (solid, mut sheet, f) = target(&part, &args.face, &placed.plane).with_context(ctx)?;
        let loops =
            cut_loops(&namer, &args.sketch, &placed, &sheet.flats[f].place).with_context(ctx)?;
        sheet.cuts.push(Cut {
            flat: f,
            loops,
            name: namer.clone(),
        });
        let folded = sheet.folded().with_context(ctx)?;
        let id = part.solid_id(&solid)?;
        part.assemble_sheet(&[Body::Solid(id)], &[])
            .with_context(ctx)?;
        let name = namer.root();
        thicken(&mut part, &folded, &name, &|n| n).with_context(ctx)?;
        part.set_body_data(&name, sheet).with_context(ctx)?;
        Ok(part)
    }
}

/// The sheet-metal body to cut, its sheet, and the flat the sketch on the
/// plane `plane` is projected onto: the one with the face `face`, or, if
/// none is named, the one the plane lies on.
fn target<S: Scalar>(
    part: &Part<S>,
    face: &str,
    plane: &CoordinateSystem<S>,
) -> GeopResult<(String, Sheet<S>, usize)> {
    let parallel = |place: &Placement<S>| {
        plane
            .w()
            .prod_cross(&place.n)
            .norm()
            .could_be_equal(S::ZERO)
    };
    let mut found = Vec::new();
    for solid in part.solid_names() {
        let Some(sheet) = part.body_data::<Sheet<S>>(&solid) else {
            continue;
        };
        for (f, flat) in sheet.flats.iter().enumerate() {
            let named = !face.is_empty()
                && (flat.name.name(&["a"]) == face || flat.name.name(&["b"]) == face);
            if face.is_empty() {
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
            } else if named {
                if !parallel(&flat.place) {
                    return Err(GeopError::new(format!(
                        "the sketch's plane is not parallel to face {face}: a sketch is cut through a sheet along the normal of a flat face it is parallel to"
                    )));
                }
                return Ok((solid.clone(), sheet.clone(), f));
            }
        }
    }
    if !face.is_empty() {
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

/// The areas of the sketch `name`, placed at `placed`, projected into the
/// sheet coordinates of the flat at `place`: each a loop counter-clockwise
/// there, its curves named `C,K,X`.
fn cut_loops<S: Scalar>(
    namer: &Namer,
    name: &str,
    placed: &geop_ops::PlacedSketch<S>,
    place: &Placement<S>,
) -> GeopResult<Vec<Vec<FlatEdge<S>>>> {
    let sketch = &placed.sketch;
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
        for lp in shape_loops(name, sketch, &geometry, Shape::Region(region))? {
            let mut edges = lp
                .profile
                .curves
                .iter()
                .zip(&lp.profile.curve_names)
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
