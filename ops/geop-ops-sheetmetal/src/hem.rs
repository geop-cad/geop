//! [`Hem`]: the edge of a sheet folded right back on itself.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Choice, Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use crate::{
    edge_flange::{Corner, FlangeShape, add_flange, check_offsets},
    sheet::{Sheet, SheetMetalRules},
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
    /// The edge, of a flat face of a sheet-metal body.
    pub edge: String,
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

impl Operation for Hem {
    type Args = HemArgs;
    type Session = ();

    /// No edge yet: a closed hem, a third of a unit long.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> HemArgs {
        HemArgs {
            edge: String::new(),
            kind: HemKind::Closed,
            length: 0.3,
            gap: 0.1,
            offset_start: 0.0,
            offset_end: 0.0,
        }
    }

    /// The edge, picked; how tight, and the gap if open; how long; and how
    /// far in from the edge's ends.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &HemArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, HemArgs> {
        let mut f = Form::<S, HemArgs>::new();
        let edge = (!args.edge.is_empty())
            .then(|| EntityRef::Edge {
                name: args.edge.clone(),
            })
            .into_iter()
            .collect();
        f.reference(
            "edge",
            "edge",
            edge,
            &[Role::Line],
            None,
            false,
            |e, picked| {
                e.args.edge = match picked.as_slice() {
                    [EntityRef::Edge { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        f.select(
            "kind",
            "hem",
            match args.kind {
                HemKind::Closed => "closed",
                HemKind::Open => "open",
            },
            vec![Choice::new("closed", "Closed"), Choice::new("open", "Open")],
            false,
            |args, choice| {
                args.kind = match choice {
                    "open" => HemKind::Open,
                    _ => HemKind::Closed,
                }
            },
        );
        if args.kind == HemKind::Open {
            f.number(
                "gap",
                Number::new("gap", args.gap, Unit::Length).range(0.0, 1.0),
                |args, g| args.gap = g,
            );
        }
        f.number(
            "length",
            Number::new("length", args.length, Unit::Length).range(0.0, 5.0),
            |args, l| args.length = l,
        );
        f.number(
            "offset_start",
            Number::new("offset at start", args.offset_start, Unit::Length).range(0.0, 5.0),
            |args, o| args.offset_start = o,
        );
        f.number(
            "offset_end",
            Number::new("offset at end", args.offset_end, Unit::Length).range(0.0, 5.0),
            |args, o| args.offset_end = o,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &HemArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("hem({operation_id}, {args:?})");
        let namer = Namer::new("hem", operation_id)?;
        let (solid, mut sheet, (f, k, toward_b)) =
            Sheet::with_edge(&part, &args.edge, "a hem").with_context(ctx)?;
        let shape = args.shape(&sheet.rules).with_context(ctx)?;
        add_flange(&mut sheet, &namer, f, k, toward_b, &shape).with_context(ctx)?;
        sheet
            .replace(&mut part, &solid, &namer.root())
            .with_context(ctx)?;
        Ok(part)
    }
}
