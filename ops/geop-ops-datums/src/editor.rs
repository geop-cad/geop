//! Editing a datum step: picking what it is built from, choosing how among
//! the constructions that fit that, setting the values the construction
//! takes — and dragging offsets as handles.

use geop_core_math::{primitives::DatumKind, scalars::Scalar, vector::Vector3};
use geop_ops::{
    Part,
    ui::{Choice, Dialog, Form, ListItem, Shape, Style, Target, Tone, Value, Visual},
};

use crate::{
    AddDatumArgs, CONSTRUCTIONS, Construction,
    add_datum::{ConstructionSchema, ParamKind, inspect_selection},
};

/// What a datum can be built from.
const SELECTION_TARGETS: &[Target] = &[
    Target::Vertex,
    Target::Edge,
    Target::Face,
    Target::Datum(DatumKind::Point),
    Target::Datum(DatumKind::Axis),
    Target::Datum(DatumKind::Plane),
    Target::Datum(DatumKind::Frame),
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

/// The offsets of what `args` builds in `part`, as handles of their fields:
/// an offset plane's distance on the plane, sliding along its normal; an
/// offset point's `x`, `y`, `z` a little out along each axis, sliding along
/// it. None for a step that does not build.
fn handles<S: Scalar>(part: &Part<S>, args: &AddDatumArgs) -> Vec<Visual<S>> {
    let Ok(built) = args
        .inputs(part)
        .and_then(|inputs| args.construction.build(&inputs))
    else {
        return Vec::new();
    };
    let handle = |param: &str, d: &Vector3<S>, out: f64| {
        Visual::new(
            format!("param:{param}"),
            Shape::Handle {
                at: built.origin().add(&d.prod_scalar(S::from_f64(out))),
                direction: *d,
            },
            Style::Handle,
        )
    };
    match args.construction {
        Construction::Offset { .. } => vec![handle("distance", built.w(), 0.0)],
        Construction::Point { .. } => vec![
            handle("x", built.u(), POINT_HANDLE_OUT),
            handle("y", built.v(), POINT_HANDLE_OUT),
            handle("z", built.w(), POINT_HANDLE_OUT),
        ],
        _ => Vec::new(),
    }
}

/// Keeps the construction fitting the selection: one that no longer fits
/// gives way to the first that does, if any does.
fn follow_selection<S: Scalar>(part: &Part<S>, args: &mut AddDatumArgs) {
    let fits = inspect_selection(part, &args.selection).fits;
    if !fits.contains(&args.construction.schema().method)
        && let Some(first) = CONSTRUCTIONS.iter().find(|c| fits.contains(&c.method))
    {
        args.construction = Construction::default_of(first);
    }
}

/// Sets the field `key` to `value`: a pick adds the entity to the
/// selection, or takes it out again; the construction follows what the
/// selection fits.
pub(crate) fn set<S: Scalar>(part: &Part<S>, args: &mut AddDatumArgs, key: &str, value: Value) {
    match (key, value) {
        ("selection", Value::Entity(entity)) => {
            match args.selection.iter().position(|e| *e == entity) {
                Some(i) => {
                    args.selection.remove(i);
                }
                None => args.selection.push(entity),
            }
            follow_selection(part, args);
        }
        ("clear", _) => {
            args.selection.clear();
            follow_selection(part, args);
        }
        ("construction", Value::Choice(method)) => {
            if let Some(schema) = CONSTRUCTIONS.iter().find(|c| c.method == method) {
                args.construction = Construction::default_of(schema);
            }
        }
        (key, Value::Remove) => {
            if let Some(i) = key
                .strip_prefix("selection:")
                .and_then(|i| i.parse::<usize>().ok())
                && i < args.selection.len()
            {
                args.selection.remove(i);
                follow_selection(part, args);
            }
        }
        (key, value) => {
            let value = match value {
                Value::Number(v) => serde_json::Value::from(v),
                Value::Bool(b) => serde_json::Value::from(b),
                _ => return,
            };
            if let Some(name) = key.strip_prefix("param:")
                && let Some(next) = args.construction.with_param(name, value)
            {
                args.construction = next;
            }
        }
    }
}

/// The selection to pick, what each selected entity can be used as, the
/// constructions that fit — and the construction's values, as fields and
/// as handles.
pub(crate) fn form<S: Scalar>(part: &Part<S>, args: &AddDatumArgs) -> Form<S> {
    let mut d = Dialog::new();
    let fit = inspect_selection(part, &args.selection);
    d.pick(
        "selection",
        "selection",
        args.selection.clone(),
        SELECTION_TARGETS,
        true,
    );
    d.list(
        "selected",
        args.selection
            .iter()
            .enumerate()
            .map(|(i, entity)| {
                let mut item = ListItem::new(format!("selection:{i}"), entity.label());
                let roles: Vec<String> = fit.roles[i]
                    .iter()
                    .map(|r| format!("{r:?}").to_lowercase())
                    .collect();
                item.detail = Some(if roles.is_empty() {
                    "not found".into()
                } else {
                    roles.join(" · ")
                });
                item.tone = if roles.is_empty() {
                    Tone::Error
                } else {
                    Tone::Normal
                };
                item.removable = true;
                item
            })
            .collect(),
        "Nothing selected yet.",
    );
    if !args.selection.is_empty() {
        d.button("clear", "Clear");
    }

    let chosen = args.construction.schema();
    d.select(
        "construction",
        "construction",
        chosen.method,
        CONSTRUCTIONS
            .iter()
            .map(|c| {
                let fits = fit.fits.contains(&c.method);
                let mut choice = Choice::new(c.method, c.label);
                choice.enabled = fits;
                choice.group = Some(group(c.result).into());
                choice.title = Some(if fits {
                    c.doc.into()
                } else {
                    format!("{}\n\nNeeds {} selected.", c.doc, needs(c))
                });
                choice
            })
            .collect(),
    );
    d.text("construction_doc", chosen.doc, Tone::Hint);
    if !fit.fits.contains(&chosen.method) {
        d.text(
            "construction_needs",
            format!("Needs {} selected.", needs(chosen)),
            Tone::Error,
        );
    }
    for param in chosen.params {
        let key = format!("param:{}", param.name);
        let value = args.construction.param(param.name);
        match param.kind {
            ParamKind::Number { min, max, .. } => {
                let v = value.and_then(|v| v.as_f64()).unwrap_or_default();
                d.slider(&key, param.name, v, min, max);
            }
            ParamKind::Bool { .. } => {
                let v = value.and_then(|v| v.as_bool()).unwrap_or_default();
                d.checkbox(&key, param.name, v);
            }
        }
    }
    Form {
        visuals: handles(part, args),
        ..Form::dialog(d)
    }
}
