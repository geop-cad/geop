//! [`FlatPattern`]: a sheet-metal body unfolded into the flat blank it is
//! cut from.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_core_sketch::Sketch;
use geop_core_topology::Body;
use geop_ops::{
    Design, NamedModel, Namer, ParamValue, PlacedSketch,
    model::model_or_empty,
    operation::{Build, BuildReader, EntityRef, Input, Lookup, LegacyModelStep, Role, Seen},
    output::sheet::Sheet as Paper,
};
use serde::{Deserialize, Serialize};

use crate::{
    body::{self, Made, Making, SheetBody, SheetStep, Unfolded, sheets, sheets_of},
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
    fn plan(reader: &mut BuildReader, step: &str, args: &FlatPatternArgs) -> GeopResult<Made> {
        let ctx = with_context!("flat_pattern({step}, {args:?})");
        let namer = Namer::new("flat_pattern", step)?;
        let sheets = sheets_of(reader);
        let model = model_or_empty(reader);
        model.solid_id(&args.solid).with_context(ctx)?;
        let sheet = (sheets.get(&args.solid).and_then(SheetBody::bent))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{} is not a sheet-metal body: a flat pattern unfolds a body built by a base flange and its flanges, which record its bends",
                    args.solid
                ))
            })
            .with_context(ctx)?
            .clone();
        check_unchanged(&model, &args.solid, &sheet).with_context(ctx)?;
        let (bend_lines, data) = bend_lines(&sheet).with_context(ctx)?;
        Ok(Made {
            consumes: (!args.keep).then(|| args.solid.clone()),
            solid: namer.root(),
            making: Making::Flat(Box::new(Unfolded {
                sheet,
                bend_lines,
                data,
            })),
        })
    }
}

impl LegacyModelStep for FlatPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        _: &FlatPatternArgs,
    ) -> GeopResult<()> {
        body::build(model, reader, step)
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
        model.assemble_sheet(&[Body::Solid(id)], &[])?;
    }
    thicken(model, &unfolded, &namer.root(), &|n| namer.name(&[&n]))?;
    Ok(())
}

impl Operation for FlatPattern {
    const INFO: OperationInfo = OperationInfo::new(
        "flat_pattern",
        "Flat pattern",
        "Unfold a sheet-metal body into the flat blank it is cut from, with its bend lines.",
        OperationGroup::SheetMetal,
        OperationTier::Small,
    )
    .icon(crate::icons::FLAT_PATTERN);
    type Args = FlatPatternArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest sheet-metal body.
    fn new_args(context: &Context<Input, ParamValue>) -> FlatPatternArgs {
        let seen = Seen::new(context.before);
        let sheets = context.before.retrieve_target(&sheets()).ok();
        FlatPatternArgs {
            solid: (seen.model().solid_names().into_iter().rev())
                .find(|s| {
                    sheets
                        .and_then(|sheets| sheets.get(s))
                        .is_some_and(|b| b.bent().is_some())
                })
                .unwrap_or_default(),
            keep: false,
        }
    }

    /// Unfolds the body, and draws its bend lines as a sketch.
    fn run(step: &str, part: &mut Build, args: &FlatPatternArgs) -> GeopResult<()> {
        body::run::<Self>(step, part, args)?;
        let namer = Namer::new("flat_pattern", step)?;
        body::add_bend_lines(part, step, &namer.name(&["bend_lines"]))
    }

    /// The body, picked, and whether to keep it.
    fn presentation(
        _: &Context<Input, ParamValue>,
        args: &FlatPatternArgs,
        _: &(),
    ) -> Form<Design> {
        let mut form = Form::default();
        let solid = EntityRef::of_name(EntityRef::solid, &args.solid);
        (form
            .dialog
            .reference("solid", "body", solid, &[Role::Solid], None, false))
        .checkbox("keep", "keep bent body", args.keep);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &FlatPatternArgs,
        _: &(),
        event: Event<Design>,
    ) -> (FlatPatternArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("solid", Value::Entities(picked)) => {
                args.solid = EntityRef::last_name(EntityRef::solid, &picked).unwrap_or_default();
            }
            ("keep", Value::Bool(b)) => args.keep = b,
            _ => {}
        }
        (args, ())
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

/// The bend lines of the flat pattern of `sheet`: the sketch of them, on
/// its A side, and what each bend is, with the pattern laid out for
/// cutting.
fn bend_lines<S: Scalar>(sheet: &Sheet<S>) -> GeopResult<(PlacedSketch<S>, FlatPatternData)> {
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
