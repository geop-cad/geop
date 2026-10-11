//! Editing a datum step: picking what it is built from, choosing how among
//! the constructions that fit that, setting the values the construction
//! takes — and dragging offsets: a plane's by a handle, a point's by a
//! gizmo.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{CoordinateSystem, DatumKind},
    scalars::Scalar,
    vector::Vector3,
};
use geop_ops::{
    Design,
    operation::{Role, Seen},
    parameters::Formula,
    ui::{Action, CanvasEvent, Change, EditForm, Gizmo, Tone, Track, formula_number},
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
    Role::Curve,
    Role::Circle,
    Role::Round,
];

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
    let roles: Vec<String> = construction.inputs.iter().map(|&r| r.describe()).collect();
    roles.join(" and ")
}

/// What `args` builds as `seen` has the part, its numbers as the form
/// shows them.
fn built(seen: &Seen, args: &AddDatumArgs) -> Option<CoordinateSystem<Design>> {
    let mut seen = seen;
    let inputs = args.inputs(&mut seen).ok()?;
    (args.construction)
        .build(&inputs, |f| f.evaluate(&mut seen))
        .ok()
}

/// Where the offsets of what `args` builds are dragged, by the name of
/// their value: an offset plane's distance on the plane, along its normal.
/// None for a step that does not build.
fn handles(seen: &Seen, args: &AddDatumArgs) -> BTreeMap<&'static str, Track<Design>> {
    match (&args.construction, built(seen, args)) {
        (Construction::Offset { .. }, Some(built)) => BTreeMap::from([(
            "distance",
            Track {
                at: *built.origin(),
                direction: *built.w(),
            },
        )]),
        _ => BTreeMap::new(),
    }
}

/// The unit axes of `frame`.
fn unit_axes(frame: &CoordinateSystem<Design>) -> Option<[Vector3<Design>; 3]> {
    Some([
        frame.u().normalize().ok()?,
        frame.v().normalize().ok()?,
        frame.w().normalize().ok()?,
    ])
}

/// The gizmo an offset point is moved by: at the point, along the axes its
/// offsets are measured along — while they are plain numbers, not
/// formulas, which a drag would overwrite.
fn gizmo(seen: &Seen, args: &AddDatumArgs) -> Option<Gizmo<Design>> {
    let Construction::Point { x, y, z } = &args.construction else {
        return None;
    };
    [x, y, z]
        .iter()
        .all(|f| f.plain().is_some())
        .then_some(())?;
    let built = built(seen, args)?;
    Some(
        Gizmo::new(*built.origin())
            .translate()
            .local(unit_axes(&built)?),
    )
}

/// The gizmo dragged: an offset point's offsets moved by as much along
/// their axes.
pub(crate) fn event(seen: &Seen, args: &mut AddDatumArgs, event: &CanvasEvent<Design>) {
    let CanvasEvent::Gizmo { drag, .. } = event else {
        return;
    };
    let (Change::Translate(by), Some(axes)) =
        (&drag.change, built(seen, args).as_ref().and_then(unit_axes))
    else {
        return;
    };
    if let Construction::Point { x, y, z } = &mut args.construction {
        for (offset, axis) in [x, y, z].into_iter().zip(&axes) {
            if let Some(v) = offset.plain() {
                *offset = Formula::Plain(v + by.prod_dot(axis).to_f64());
            }
        }
    }
}

/// Keeps the construction fitting the selection: one that no longer fits
/// gives way to the first that does, if any does.
fn follow_selection(seen: &Seen, args: &mut AddDatumArgs) {
    let fits = fitting_constructions(&mut { seen }, &args.selection);
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
pub(crate) fn form<'a>(
    seen: &'a Seen<'a>,
    args: &AddDatumArgs,
) -> EditForm<'a, Design, AddDatumArgs> {
    let mut f = EditForm::<Design, AddDatumArgs>::default();
    let fits = fitting_constructions(&mut { seen }, &args.selection);
    f.reference(
        "selection",
        "selection",
        args.selection.clone(),
        SELECTION_ROLES,
        None,
        true,
        move |edit, selection| {
            edit.args.selection = selection;
            follow_selection(seen, edit.args);
        },
    );

    let chosen = args.construction.schema();
    let actions = (CONSTRUCTIONS.iter())
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
        .collect();
    f.actions("construction", actions, |edit, method| {
        if let Some(schema) = CONSTRUCTIONS.iter().find(|c| c.method == method) {
            edit.args.construction = Construction::default_of(schema);
        }
    });
    f.form
        .dialog
        .text("construction_doc", chosen.doc, Tone::Hint);
    if !fits.contains(&chosen.method) {
        let text = format!("Needs {} selected.", needs(chosen));
        f.form.dialog.text("construction_needs", text, Tone::Error);
    }
    let mut handles = handles(seen, args);
    f.form.gizmo = gizmo(seen, args);
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
                let number = formula_number(name, &formula, &mut { seen }, unit)
                    .range(min, max)
                    .handle(handles.remove(name));
                f.formula(name, number, move |args, v| {
                    if let Ok(v) = serde_json::to_value(v) {
                        set(args, v)
                    }
                });
            }
            ParamKind::Bool { .. } => {
                let v = value.and_then(|v| v.as_bool()).unwrap_or_default();
                f.checkbox(name, name, v, move |args, b| set(args, b.into()));
            }
        }
    }
    f
}
