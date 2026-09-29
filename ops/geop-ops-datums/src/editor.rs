//! Editing a datum step: picking what it is built from, choosing how among
//! the constructions that fit that, setting the values the construction
//! takes — and dragging offsets as handles.

use geop_core_math::{primitives::DatumKind, scalars::Scalar, vector::Vector3};
use geop_ops::{
    EditContext, Edited, Part,
    operation::EntityRef,
    ui::{
        Choice, Dialog, DialogValue, Dragging, Event, ListItem, Picked, Picking, Presentation,
        SelectStyle, Shape, Style, Target, Tone, Visual,
    },
};
use serde::{Deserialize, Serialize};

use crate::{
    AddDatumArgs, CONSTRUCTIONS, Construction,
    add_datum::{ConstructionSchema, ParamKind, describe_role, inspect_selection},
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

/// The temporary state of editing a datum step.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DatumSession {
    /// Whether the edit has begun: a step with nothing selected yet starts
    /// by picking.
    started: bool,
    pick: Picking,
    drag: Dragging,
    /// Whether the pointer is over a handle.
    over_handle: bool,
}

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
    let roles: Vec<&str> = construction
        .inputs
        .iter()
        .map(|&r| describe_role(r))
        .collect();
    roles.join(" and ")
}

/// The offsets of what `args` builds in `part`, as handles: an offset
/// plane's distance on the plane, sliding along its normal; an offset
/// point's `x`, `y`, `z` a little out along each axis, sliding along it.
/// None for a step that does not build.
fn handles<S: Scalar>(part: &Part<S>, args: &AddDatumArgs) -> Vec<Visual<S>> {
    let Ok(built) = args
        .inputs(part)
        .and_then(|inputs| args.construction.build(&inputs))
    else {
        return Vec::new();
    };
    let handle = |key: &str, d: &Vector3<S>, out: f64| {
        Visual::new(
            key,
            Shape::Handle {
                at: built.origin().add(&d.prod_scalar(S::from_f64(out))),
                direction: Some(*d),
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

fn event<S: Scalar>(
    ctx: &EditContext<S>,
    args: &mut AddDatumArgs,
    s: &mut DatumSession,
    event: &Event<S>,
) {
    match event.dialog() {
        Some(("selection", _)) => s.pick.toggle("selection"),
        Some(("clear", _)) => {
            args.selection.clear();
            follow_selection(ctx.part, args);
        }
        Some(("construction", DialogValue::Choice(method))) => {
            if let Some(schema) = CONSTRUCTIONS.iter().find(|c| c.method == method) {
                args.construction = Construction::default_of(schema);
            }
        }
        Some((key, value)) => {
            if let Some(i) = key
                .strip_prefix("selection:")
                .and_then(|i| i.parse::<usize>().ok())
                && *value == DialogValue::Remove
                && i < args.selection.len()
            {
                args.selection.remove(i);
                follow_selection(ctx.part, args);
            }
            if let Some(name) = key.strip_prefix("param:") {
                let value = match value {
                    DialogValue::Number(v) => Some(serde_json::Value::from(*v)),
                    DialogValue::Bool(b) => Some(serde_json::Value::from(*b)),
                    _ => None,
                };
                if let Some(next) = value.and_then(|v| args.construction.with_param(name, v)) {
                    args.construction = next;
                }
            }
        }
        None => {}
    }
    if let Event::Key { key } = event
        && key == "Escape"
    {
        s.pick.disarm();
    }
    // A pick adds the entity to the selection, or takes it out again; the
    // pick stays armed for the next.
    if let Picked::Picked { entity, .. } = s.pick.handle(ctx.view, event, SELECTION_TARGETS) {
        match args.selection.iter().position(|e| *e == entity) {
            Some(i) => {
                args.selection.remove(i);
            }
            None => args.selection.push(entity),
        }
        follow_selection(ctx.part, args);
    }
    let handles = handles(ctx.part, args);
    let value_of = |key: &str| {
        let value = args.construction.param(key)?.as_f64()?;
        Some((value, 1.0))
    };
    if let Some((key, value)) = s.drag.linear(&handles, event, value_of)
        && let Some(next) = args
            .construction
            .with_param(&key, serde_json::Value::from(value))
    {
        args.construction = next;
    }
    if let Event::Hover { pointer } = event {
        s.over_handle = Dragging::over_handle(&handles, pointer);
    }
}

fn dialog<S: Scalar>(part: &Part<S>, args: &AddDatumArgs, s: &DatumSession) -> Dialog {
    let mut d = Dialog::new();
    let fit = inspect_selection(part, &args.selection);
    let picking = s.pick.is("selection");
    d.pick_button(
        "selection",
        if picking {
            "Done picking"
        } else {
            "Pick selection…"
        },
        picking,
    );
    if picking {
        d.text(
            "selection_hint",
            "Click points, edges, faces and planes — and the origin's — to add them, or again to remove them…",
            Tone::Hint,
        );
    }
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
        SelectStyle::Radio,
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
    d
}

/// Edits a datum step: see the module docs.
pub(crate) fn edit<S: Scalar>(
    ctx: &EditContext<S>,
    mut args: AddDatumArgs,
    mut s: DatumSession,
    event_: Option<&Event<S>>,
) -> Edited<AddDatumArgs, DatumSession, S> {
    if !s.started {
        s.started = true;
        if args.selection.is_empty() {
            s.pick.arm("selection");
        }
    }
    if let Some(e) = event_ {
        event(ctx, &mut args, &mut s, e);
    }
    let mut highlights: Vec<EntityRef> = args.selection.clone();
    highlights.extend(s.pick.hover.clone());
    let presentation = Presentation {
        dialog: dialog(ctx.part, &args, &s),
        visuals: handles(ctx.part, &args),
        highlights,
        pickable: if s.pick.field.is_some() {
            SELECTION_TARGETS.to_vec()
        } else {
            Vec::new()
        },
        focus: None,
        grab: s.over_handle,
    };
    Edited {
        args,
        session: s,
        presentation,
    }
}
