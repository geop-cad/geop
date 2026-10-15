//! [`FlatPattern`]: a sheet-metal body unfolded into the flat blank it is
//! cut from.

use std::collections::BTreeSet;

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_program::element::ElementReference;
use geop_core_sketch::Sketch;
use geop_core_topology::Body;
use geop_ops_model::{Design, ModelOperation, NamedModel, Namer, Solid, model, read_model};
use geop_ops_output::sheet::Sheet as Paper;
use geop_ops_parameters::{Setting, BuildPart, Runner, PartReader, StateValue, StepContext};
use geop_ops_sketch::PlacedSketch;
use serde::{Deserialize, Serialize};

use crate::{
    body::{self, Made, Making, SheetBody, SheetStep, Unfolded, sheet_body},
    cutting::cutting_sheet,
    sheet::Sheet,
    thicken::thicken,
};

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
/// is recorded for the flat body as [`FlatPatternData`] (see
/// [`crate::body::sheets`]).
///
/// Only a body whose bends were recorded as it was built — by a base
/// flange and its flanges — unfolds, and only while it is still made of
/// just the faces they built: a body some other step has changed is
/// refused, naming the faces that are not sheet metal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FlatPattern;

/// A solid of the part, by name.
pub type SolidRef = ElementReference<NamedModel<Design>, Solid>;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlatPatternArgs {
    /// The sheet-metal body.
    pub solid: SolidRef,
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

/// What a flat pattern records of its body, for drawings and cutting:
/// the sheet's thickness, the bends, and the pattern laid out for cutting
/// (see [`cutting_sheet`]).
#[derive(Clone, Debug, PartialEq)]
pub struct FlatPatternData {
    pub thickness: f64,
    pub bends: Vec<BendLine>,
    pub pattern: Paper,
}

impl SheetStep for FlatPattern {
    fn plan(reader: &PartReader, step: &str, args: &FlatPatternArgs) -> GeopResult<Made> {
        let ctx = with_context!("flat_pattern({step}, {args:?})");
        let namer = Namer::new("flat_pattern", step)?;
        let solid = args.solid.name();
        reader.retrieve_element(&args.solid).with_context(ctx)?;
        let model = read_model(reader).with_context(ctx)?;
        let sheet = (sheet_body(reader, solid).and_then(SheetBody::bent))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{solid} is not a sheet-metal body: a flat pattern unfolds a body built by a base flange and its flanges, which record its bends"
                ))
            })
            .with_context(ctx)?
            .clone();
        check_unchanged(&model, solid, &sheet).with_context(ctx)?;
        let (bend_lines, data) =
            bend_lines(&sheet, &namer.name(&["bend_lines"])).with_context(ctx)?;
        Ok(Made {
            consumes: (!args.keep).then(|| solid.to_string()),
            solid: namer.root(),
            making: Making::Flat(Box::new(Unfolded {
                sheet,
                bend_lines,
                data,
            })),
        })
    }
}

/// Builds the flat pattern of `sheet` into `model`, named by `namer`, in
/// place of the solid `consumes`, if any.
pub(crate) fn build(
    model: &mut NamedModel<Design>,
    sheet: &Sheet<Design>,
    consumes: Option<&str>,
    namer: &Namer,
) -> GeopResult<()> {
    let unfolded = sheet.unfolded()?;
    if let Some(consumed) = consumes {
        let id = model.solid_id(consumed)?;
        model.delete_bodies(&[Body::Solid(id)])?;
    }
    thicken(model, &unfolded, &namer.root(), &|n| namer.name(&[&n]))?;
    Ok(())
}

part_operation!(FlatPattern(FlatPatternArgs));

impl PartOperation for FlatPattern {
    const KIND: &'static str = "flat_pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Flat pattern",
        "Unfold a sheet-metal body into the flat blank it is cut from, with its bend lines.",
        OperationGroup::SheetMetal,
        OperationTier::Small,
    )
    .icon(crate::icons::FLAT_PATTERN);
    type Args = FlatPatternArgs;
    type Input = Setting;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    /// The newest sheet-metal body.
    fn new_args(context: &StepContext) -> FlatPatternArgs {
        let before = &context.before;
        let solids = read_model(before).map(|model| model.solid_names());
        let newest = (solids.unwrap_or_default().into_iter().rev())
            .find(|s| sheet_body(before, s).is_some_and(|b| b.bent().is_some()));
        FlatPatternArgs {
            solid: ElementReference::new(model(), newest.unwrap_or_default()),
            keep: false,
        }
    }

    /// Unfolds the body, and draws its bend lines as a sketch.
    fn run(step: &str, _: &Runner, part: &mut BuildPart, args: &FlatPatternArgs) -> GeopResult<()> {
        body::run::<Self>(step, part, args)?;
        let namer = Namer::new("flat_pattern", step)?;
        body::add_bend_lines(part, step, &namer.name(&["bend_lines"]))
    }

    /// The body, picked, and whether to keep it.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &FlatPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, FlatPatternArgs> {
        let mut f = EditForm::<Design, FlatPatternArgs>::default();
        f.pick_one(
            "solid",
            "body",
            &context.before,
            Some(&args.solid),
            |edit, solid| edit.args.solid = solid,
        )
        .checkbox("keep", "keep bent body", args.keep, |edit, b| {
            edit.args.keep = b
        });
        f
    }
}

impl ModelOperation for FlatPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &PartReader,
        step: &str,
        _: &FlatPatternArgs,
    ) -> GeopResult<()> {
        body::build(model, reader, step)
    }
}

/// Checks that the solid `solid` is still made of exactly the faces
/// `sheet` built: refuses, naming them, faces it does not know — another
/// step's — and faces it built that are gone.
fn check_unchanged<S: Scalar>(
    model: &NamedModel<S>,
    solid: &str,
    sheet: &Sheet<S>,
) -> GeopResult<()> {
    let built: BTreeSet<String> = sheet.folded()?.face_names()?.into_iter().collect();
    let faces = model.topology().solid_faces(model.solid_id(solid)?)?;
    let present: BTreeSet<String> = faces
        .iter()
        .map(|&f| model.name_of(f).unwrap_or("?").to_string())
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

/// The bend lines of the flat pattern of `sheet`: the sketch `name` of
/// them, on its A side, and what each bend is, with the pattern laid out
/// for cutting.
fn bend_lines<S: Scalar>(
    sheet: &Sheet<S>,
    name: &str,
) -> GeopResult<(PlacedSketch<S>, FlatPatternData)> {
    let place = &sheet.flats[0].place;
    let mut sketch = Sketch::<Design>::new();
    let mut bends = Vec::new();
    for (bend, [a, b]) in sheet.bends.iter().zip(sheet.bend_lines()?) {
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
    let sketch = PlacedSketch {
        name: name.to_string(),
        plane: place.coordinate_system()?,
        sketch,
    };
    let data = FlatPatternData {
        thickness: sheet.rules.thickness,
        bends,
        pattern: cutting_sheet(sheet)?,
    };
    Ok((sketch, data))
}
