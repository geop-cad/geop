//! [`Hem`]: the edge of a sheet folded right back on itself.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Choice, Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    part::{Part, Reader},
    program_runner::ProgramRunner,
    values::Design,
};
use geop_ops_model::{ModelOperation, NamedModel, Namer, read_model};
use serde::{Deserialize, Serialize};

use crate::{
    body::{self, Made, Making, SheetStep, with_edge},
    edge_flange::{
        Corner, EdgeRef, FlangeShape, add_flange, check_offsets, edge_field, offset_fields,
    },
    sheet::SheetMetalRules,
};

/// How tight a hem is folded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HemKind {
    /// As tight as the body's bends go: its inner radius the body's bend
    /// radius. A fold of no radius at all, the hem pressed flat onto the
    /// sheet, would leave a solid touching itself, which is not one.
    #[default]
    Closed,
    /// Folded round with a gap between the hem and the sheet: its inner
    /// radius half the gap.
    Open,
}

/// Folds the straight edge `edge` of a sheet-metal body right back on
/// itself, for the operation `H`: a bend of half a turn and a flat lying
/// back over the sheet, the fold's outside where the edge was — the sheet
/// set back by the fold's outer radius — and the hem `length` long from
/// there. The body is consumed and rebuilt as `hem(H)`, its entities named
/// as [`crate::EdgeFlange`] names a flange's, `hem(H,bend)`,
/// `hem(H,flange)`, ...
///
/// `edge` is an edge of a flat face on the body's outline, and the hem
/// folds towards that face's side. `kind` says how tight (`gap` is the
/// space between an open hem and the sheet), and `offset_start` /
/// `offset_end` how far in from the edge's ends it runs, the body's relief
/// beside it where it does not reach them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Hem;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HemArgs {
    /// The edge, of a flat face of a sheet-metal body; none yet, for a new
    /// hem waiting for one to be picked.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge: Option<EdgeRef>,
    #[serde(default)]
    pub kind: HemKind,
    /// How far the hem reaches back over the sheet, from the fold's
    /// outside.
    pub length: f64,
    /// The space between an open hem and the sheet.
    #[serde(default)]
    pub gap: f64,
    #[serde(default)]
    pub offset_start: f64,
    #[serde(default)]
    pub offset_end: f64,
}

impl HemArgs {
    /// The hem these arguments describe, on a body of `rules`: a flange
    /// turning half a turn, set back by its outer radius.
    fn shape<S: Scalar>(&self, rules: &SheetMetalRules) -> GeopResult<FlangeShape<S>> {
        let radius = match self.kind {
            HemKind::Closed => rules.bend_radius,
            HemKind::Open => {
                if !(self.gap.is_finite() && self.gap > 0.0) {
                    return Err(GeopError::new(format!(
                        "an open hem's gap must be positive, not {}: for the tightest hem, close it",
                        self.gap
                    )));
                }
                self.gap / 2.0
            }
        };
        check_offsets(self.offset_start, self.offset_end)?;
        let s = S::from_f64;
        let r = s(radius);
        let outer = r.add(s(rules.thickness));
        let flat = s(self.length).sub(outer);
        if !flat.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "a hem {} long is no longer than its fold, {outer:?} round the outside: it leaves nothing flat",
                self.length
            )));
        }
        Ok(FlangeShape {
            angle: S::PI,
            radius: r,
            set_back: Some(outer),
            flat,
            offset_start: self.offset_start,
            offset_end: self.offset_end,
            corner: Corner::Open,
        })
    }
}

impl SheetStep for Hem {
    fn plan(reader: &Reader, step: &str, args: &HemArgs) -> GeopResult<Made> {
        let ctx = with_context!("hem({step}, {args:?})");
        let namer = Namer::new("hem", step)?;
        let model = read_model(reader).with_context(ctx)?;
        let (solid, mut sheet, (f, k, toward_b)) =
            with_edge(&model, reader, args.edge.as_ref(), "a hem").with_context(ctx)?;
        let shape = args.shape(&sheet.rules).with_context(ctx)?;
        add_flange(&mut sheet, &namer, f, k, toward_b, &shape).with_context(ctx)?;
        Ok(Made {
            consumes: Some(solid),
            solid: namer.root(),
            making: Making::Bent(sheet),
        })
    }
}

part_operation!(Hem(HemArgs));

impl PartOperation for Hem {
    const KIND: &'static str = "hem";
    const INFO: OperationInfo = OperationInfo::new(
        "Hem",
        "Fold the straight edge of a sheet-metal body right back on itself.",
        OperationGroup::SheetMetal,
        OperationTier::Small,
    )
    .icon(crate::icons::HEM);
    type Args = HemArgs;
    type Session = ();

    /// No edge yet: a closed hem, a third of a unit long.
    fn new_args(_: &Context) -> HemArgs {
        HemArgs {
            edge: None,
            kind: HemKind::Closed,
            length: 0.3,
            gap: 0.1,
            offset_start: 0.0,
            offset_end: 0.0,
        }
    }

    fn run(step: &str, _: &ProgramRunner, part: &mut Part, args: &HemArgs) -> GeopResult<()> {
        body::run::<Self>(step, part, args)
    }

    /// The edge, picked; how tight, and the gap if open; how long; and how
    /// far in from the edge's ends.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &HemArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, HemArgs> {
        let mut f = EditForm::<Design, HemArgs>::default();
        edge_field(&mut f, &context.before, args.edge.as_ref(), |edit, edge| {
            edit.args.edge = Some(edge)
        });
        f.select(
            "kind",
            "hem",
            match args.kind {
                HemKind::Closed => "closed",
                HemKind::Open => "open",
            },
            vec![Choice::new("closed", "Closed"), Choice::new("open", "Open")],
            false,
            |edit, choice| {
                edit.args.kind = match choice {
                    "open" => HemKind::Open,
                    _ => HemKind::Closed,
                }
            },
        );
        if args.kind == HemKind::Open {
            let gap = Number::new("gap", args.gap, Unit::Length).range(0.0, 1.0);
            f.number("gap", gap, |edit, g| edit.args.gap = g);
        }
        let length = Number::new("length", args.length, Unit::Length).range(0.0, 5.0);
        f.number("length", length, |edit, l| edit.args.length = l);
        offset_fields(
            &mut f,
            args.offset_start,
            args.offset_end,
            |args, [start, end]| {
                args.offset_start = start;
                args.offset_end = end;
            },
        );
        f
    }
}

impl ModelOperation for Hem {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        _: &HemArgs,
    ) -> GeopResult<()> {
        body::build(model, reader, step)
    }
}
