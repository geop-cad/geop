//! [`FlatPattern`]: a sheet-metal body unfolded into the flat blank it is
//! cut from.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::Sketch;
use geop_core_topology::Body;
use geop_ops::{
    Context, Design, Library, Namer, Part, PlacedSketch,
    operation::{EntityRef, Operation, Role},
    ui::Form,
};
use serde::{Deserialize, Serialize};

use crate::{sheet::Sheet, thicken::thicken};

/// Unfolds the sheet-metal body named `solid` into its flat pattern, for
/// the operation `P`: every bend laid flat as a strip as wide as its
/// developed length (`angle * (radius + k_factor * thickness)`, see
/// [`crate::SheetMetalRules`]), every flat face moved with it, the body's
/// first face where it was. The flat body is the solid `flat_pattern(P)`,
/// and each of its entities is named after the one it is the unfolded
/// copy of, `flat_pattern(P,X)` for `X` — so a bend's faces are
/// `flat_pattern(P,X,a)` for its faces `X,a`, and the lines it is bent
/// along the edges between it and its flat faces. The body is replaced by
/// its flat pattern unless `keep`.
///
/// The bend lines — down the middle of each bend's strip — are also the
/// sketch `flat_pattern(P,bend_lines)`, on the flat pattern's A side, one
/// line per bend in the order the bends were made; and what each bend is
/// recorded on the flat body as [`FlatPatternData`].
///
/// Only a body whose bends were recorded as it was built — by a base
/// flange and its flanges — unfolds, and only while it is still made of
/// just the faces they built: a body some other step has changed is
/// refused, naming the faces that are not sheet metal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FlatPattern;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlatPatternArgs {
    /// The sheet-metal body.
    pub solid: String,
    /// Keep the bent body beside the flat one.
    #[serde(default)]
    pub keep: bool,
}

/// One bend of a flat pattern: the line down its middle, its ends in space
/// on the flat pattern's A side, and how it is bent.
#[derive(Clone, Debug, PartialEq)]
pub struct BendLine {
    /// The bend's name, `X` for its faces `X,a` and `X,b`.
    pub bend: String,
    pub start: [f64; 3],
    pub end: [f64; 3],
    /// How far it turns, in degrees.
    pub angle: f64,
    /// Its inner radius.
    pub radius: f64,
    /// Whether it turns towards the B side — up, seen from the B side.
    pub toward_b: bool,
}

/// What a flat pattern records on its body, for drawings and cutting:
/// the sheet's thickness and the bends.
#[derive(Clone, Debug, PartialEq)]
pub struct FlatPatternData {
    pub thickness: f64,
    pub bends: Vec<BendLine>,
}

impl Operation for FlatPattern {
    type Args = FlatPatternArgs;
    type Session = ();

    /// The newest sheet-metal body.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> FlatPatternArgs {
        FlatPatternArgs {
            solid: before
                .solid_names()
                .into_iter()
                .rev()
                .find(|s| before.body_data::<Sheet<S>>(s).is_some())
                .unwrap_or_default(),
            keep: false,
        }
    }

    /// The body, picked, and whether to keep it.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &FlatPatternArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, FlatPatternArgs> {
        let mut f = Form::<S, FlatPatternArgs>::new();
        let solid = (!args.solid.is_empty())
            .then(|| EntityRef::Solid {
                name: args.solid.clone(),
            })
            .into_iter()
            .collect();
        f.reference(
            "solid",
            "body",
            solid,
            &[Role::Solid],
            None,
            false,
            |e, picked| {
                e.args.solid = match picked.as_slice() {
                    [EntityRef::Solid { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        f.checkbox("keep", "keep bent body", args.keep, |args, b| args.keep = b);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &FlatPatternArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("flat_pattern({operation_id}, {args:?})");
        let namer = Namer::new("flat_pattern", operation_id)?;
        let id = part.solid_id(&args.solid).with_context(ctx)?;
        let sheet = part
            .body_data::<Sheet<S>>(&args.solid)
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{} is not a sheet-metal body: a flat pattern unfolds a body built by a base flange and its flanges, which record its bends",
                    args.solid
                ))
            })
            .with_context(ctx)?
            .clone();
        check_unchanged(&part, &args.solid, &sheet).with_context(ctx)?;
        let unfolded = sheet.unfolded().with_context(ctx)?;
        if !args.keep {
            part.assemble_sheet(&[Body::Solid(id)], &[])
                .with_context(ctx)?;
        }
        let name = namer.root();
        thicken(&mut part, &unfolded, &name, &|n| namer.name(&[&n])).with_context(ctx)?;
        let data = bend_lines(&mut part, &namer, &sheet).with_context(ctx)?;
        part.set_body_data(&name, data).with_context(ctx)?;
        Ok(part)
    }
}

/// Checks that the solid `solid` is still made of exactly the faces
/// `sheet` built: refuses, naming them, faces it does not know — another
/// step's — and faces it built that are gone.
fn check_unchanged<S: Scalar>(part: &Part<S>, solid: &str, sheet: &Sheet<S>) -> GeopResult<()> {
    let built: BTreeSet<String> = sheet.folded()?.face_names()?.into_iter().collect();
    let faces = part.topology().solid_faces(part.solid_id(solid)?)?;
    let present: BTreeSet<String> = faces
        .iter()
        .map(|&f| part.name_of(f).unwrap_or("?").to_string())
        .collect();
    let foreign: Vec<&String> = present.difference(&built).collect();
    let missing: Vec<&String> = built.difference(&present).collect();
    if foreign.is_empty() && missing.is_empty() {
        return Ok(());
    }
    Err(GeopError::new(format!(
        "{solid} is no longer only sheet metal, so it cannot be unfolded: its faces {foreign:?} are not faces its flanges built, and {missing:?} of those are gone"
    )))
}

/// The bend lines of the flat pattern of `sheet`: added to `part` as the
/// sketch `P(bend_lines)`, and returned with what each bend is.
fn bend_lines<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    sheet: &Sheet<S>,
) -> GeopResult<FlatPatternData> {
    let shifts = sheet.flat_shifts()?;
    let place = &sheet.flats[0].place;
    let mut sketch = Sketch::<Design>::new();
    let mut bends = Vec::new();
    for bend in &sheet.bends {
        let (f, k) = sheet
            .outer_edge(&bend.parent_edge)
            .expect("a bend's parent edge is on its flat");
        let [a, b] = sheet.flats[f].outer[k]
            .line()
            .expect("a bend starts at a straight edge")?;
        // Halfway across the strip, which reaches out of the parent.
        let tau = b.sub(&a).normalize()?;
        let out = Vector2::from_array([tau[1], tau[0].neg()]);
        let half = out.prod_scalar(sheet.developed_length(bend).div(S::TWO)?);
        let shift = shifts[bend.parent].add(&half);
        let (a, b) = (a.add(&shift), b.add(&shift));
        let design = |x: S| Design::from_f64(x.to_f64());
        let p = sketch.add_point(design(a[0]), design(a[1]));
        let q = sketch.add_point(design(b[0]), design(b[1]));
        sketch.add_line(p, q);
        let at = |v: Vector2<S>| place.point(&v).to_array().map(|c| c.to_f64());
        bends.push(BendLine {
            bend: bend.name.root(),
            start: at(a),
            end: at(b),
            angle: bend.angle.to_f64().to_degrees(),
            radius: bend.radius.to_f64(),
            toward_b: bend.toward_b,
        });
    }
    part.add_sketch(
        PlacedSketch {
            plane: place.coordinate_system()?,
            sketch,
        },
        namer.name(&["bend_lines"]),
    )?;
    Ok(FlatPatternData {
        thickness: sheet.rules.thickness,
        bends,
    })
}
