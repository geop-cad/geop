//! Editing a datum step: picking what it is built from, choosing how among
//! the constructions that fit that, setting the values the construction
//! takes — and dragging offsets as handles.

use std::collections::BTreeMap;

use geop_core_math::{primitives::DatumKind, scalars::Scalar, vector::Vector3};
use geop_ops::{
    Part,
    operation::Role,
    parameters::Formula,
    ui::{Action, Form, Number, Tone, Track},
};

use crate::{
    AddDatumArgs, CONSTRUCTIONS, Construction,
    add_datum::{ConstructionSchema, ParamKind, fitting_constructions},
};

/// What a datum can be built from: whatever can be an input of some
/// construction.
const SELECTION_ROLES: &[Role] = &[
    Role::Point,
    Role::Line,
    Role::Plane,
    Role::Edge,
    Role::Circle,
    Role::Round,
];

/// How far out along its axis an offset point's handle sits, in world
/// units, so the three of one point can each be grabbed.
const POINT_HANDLE_OUT: f64 = 0.3;

/// What a group of constructions is called: by what they build.
fn group(result: DatumKind) -> &'static str {
    match result {
        DatumKind::Point => "Point",
        DatumKind::Axis => "Axis",
        DatumKind::Plane => "Plane",
        DatumKind::Frame => "Coordinate system",
    }
}

/// What `construction` needs selected, in words: `a point and a plane`.
fn needs(construction: &ConstructionSchema) -> String {
    let roles: Vec<&str> = construction.inputs.iter().map(|&r| r.describe()).collect();
    roles.join(" and ")
}

/// Where the offsets of what `args` builds in `part` are dragged, by the
/// name of their value: an offset plane's distance on the plane, along its
/// normal; an offset point's `x`, `y`, `z` a little out along each axis,
/// along it. None for a step that does not build.
fn handles<S: Scalar>(part: &Part<S>, args: &AddDatumArgs) -> BTreeMap<&'static str, Track<S>> {
    let Ok(built) = args.inputs(part).and_then(|inputs| {
        args.construction
            .build(&inputs, |f| f.peek(part.inputs()))
    }) else {
        return BTreeMap::new();
    };
    let track = |d: &Vector3<S>, out: f64| Track {
        at: built.origin().add(&d.prod_scalar(S::from_f64(out))),
        direction: *d,
    };
    match args.construction {
        Construction::Offset { .. } => BTreeMap::from([("distance", track(built.w(), 0.0))]),
        Construction::Point { .. } => BTreeMap::from([
            ("x", track(built.u(), POINT_HANDLE_OUT)),
            ("y", track(built.v(), POINT_HANDLE_OUT)),
            ("z", track(built.w(), POINT_HANDLE_OUT)),
        ]),
        _ => BTreeMap::new(),
    }
}

/// Keeps the construction fitting the selection: one that no longer fits
/// gives way to the first that does, if any does.
fn follow_selection<S: Scalar>(part: &Part<S>, args: &mut AddDatumArgs) {
    let fits = fitting_constructions(part, &args.selection);
    if !fits.contains(&args.construction.schema().method)
        && let Some(first) = CONSTRUCTIONS.iter().find(|c| fits.contains(&c.method))
    {
        args.construction = Construction::default_of(first);
    }
}

/// The selection to pick — which the construction follows (see
/// [`follow_selection`]) — the constructions, those that do not fit it
/// saying what they need, and the construction's values, offsets with
/// handles.
pub(crate) fn form<'a, S: Scalar>(
    part: &'a Part<S>,
    args: &AddDatumArgs,
) -> Form<'a, S, AddDatumArgs> {
    let mut f = Form::<S, AddDatumArgs>::new();
    let fits = fitting_constructions(part, &args.selection);
    f.reference(
        "selection",
        "selection",
        args.selection.clone(),
        SELECTION_ROLES,
        None,
        true,
        move |edit, selection| {
            edit.args.selection = selection;
            follow_selection(part, edit.args);
        },
    );

    let chosen = args.construction.schema();
    f.actions(
        "construction",
        CONSTRUCTIONS
            .iter()
            .map(|c| {
                let action = Action::new(c.method, c.label)
                    .group(group(c.result))
                    .active(c.method == chosen.method);
                if fits.contains(&c.method) {
                    action.title(c.doc)
                } else {
                    action.disabled(format!("{}\n\nNeeds {} selected.", c.doc, needs(c)))
                }
            })
            .collect(),
        |edit, method| {
            if let Some(schema) = CONSTRUCTIONS.iter().find(|c| c.method == method) {
                edit.args.construction = Construction::default_of(schema);
            }
        },
    );
    f.text("construction_doc", chosen.doc, Tone::Hint);
    if !fits.contains(&chosen.method) {
        f.text(
            "construction_needs",
            format!("Needs {} selected.", needs(chosen)),
            Tone::Error,
        );
    }
    let mut handles = handles(part, args);
    for param in chosen.params {
        let name = param.name;
        let set = move |args: &mut AddDatumArgs, value: serde_json::Value| {
            if let Some(next) = args.construction.with_param(name, value) {
                args.construction = next;
            }
        };
        let value = args.construction.param(name);
        match param.kind {
            ParamKind::Number { min, max, unit, .. } => {
                let formula = value
                    .and_then(|v| serde_json::from_value::<Formula>(v).ok())
                    .unwrap_or(Formula::Plain(0.0));
                f.formula(
                    name,
                    Number::formula(name, &formula, part.inputs(), unit)
                        .range(min, max)
                        .handle(handles.remove(name)),
                    move |args, v| {
                        if let Ok(v) = serde_json::to_value(v) {
                            set(args, v)
                        }
                    },
                );
            }
            ParamKind::Bool { .. } => {
                let v = value.and_then(|v| v.as_bool()).unwrap_or_default();
                f.checkbox(name, name, v, move |args, b| set(args, b.into()));
            }
        }
    }
    f
}
