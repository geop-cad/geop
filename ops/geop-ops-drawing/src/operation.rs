//! [`Drawing`]: a step describing a 2-D drawing of the part as the steps
//! before it built it — its views, a section, the dimensions asked for and
//! the title block. It builds nothing; the drawing is made from it when it
//! is exported ([`crate::drawing::compose`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Choice, Form, ListItem, Tone, Value},
};
use serde::{Deserialize, Serialize};

use crate::{
    drawing::{
        Dimension, DrawingArgs, Projection, SCALES, SheetSize, placed_edge, placed_vertex,
        scale_label,
    },
    view::ViewKind,
};

/// Describes a drawing of the part: see [`DrawingArgs`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Drawing;

/// What editing a drawing keeps between events: the first vertex of a
/// distance being picked.
#[derive(Default)]
pub struct DrawingSession {
    pending: Vec<EntityRef>,
}

fn label(kind: ViewKind) -> &'static str {
    match kind {
        ViewKind::Front => "Front",
        ViewKind::Top => "Top",
        ViewKind::Right => "Right",
        ViewKind::Left => "Left",
        ViewKind::Bottom => "Bottom",
        ViewKind::Back => "Back",
        ViewKind::Iso => "Isometric",
    }
}

impl Dimension {
    /// How a list shows it.
    fn label(&self) -> String {
        match self {
            Dimension::Distance { from, to } => format!("Distance {from} – {to}"),
            Dimension::Radius { edge } => format!("Radius of {edge}"),
            Dimension::Diameter { edge } => format!("Diameter of {edge}"),
        }
    }
}

/// The name of the one circular edge in `picked`, if that is what it is.
fn picked_edge(picked: &[EntityRef]) -> Option<String> {
    match picked {
        [EntityRef::Edge { name }] => Some(name.clone()),
        _ => None,
    }
}

impl Operation for Drawing {
    type Args = DrawingArgs;
    type Session = DrawingSession;

    /// Front, top, right and isometric views, third angle, on A3.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> DrawingArgs {
        DrawingArgs::default()
    }

    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &DrawingArgs,
        session: &DrawingSession,
        _: &[String],
    ) -> Form<'a, S, DrawingArgs, DrawingSession> {
        let mut f = Form::<S, DrawingArgs, DrawingSession>::new();
        f.heading("views_heading", "Views");
        for kind in ViewKind::ALL {
            f.checkbox(
                &format!("view:{}", kind.name()),
                label(kind),
                args.views.contains(&kind),
                move |args: &mut DrawingArgs, on| {
                    args.views.retain(|k| *k != kind);
                    if on {
                        args.views.push(kind);
                        args.views
                            .sort_by_key(|k| ViewKind::ALL.iter().position(|a| a == k));
                    }
                },
            );
        }
        f.select(
            "projection",
            "projection",
            match args.projection {
                Projection::ThirdAngle => "third_angle",
                Projection::FirstAngle => "first_angle",
            },
            vec![
                Choice::new("third_angle", "Third angle"),
                Choice::new("first_angle", "First angle"),
            ],
            false,
            |args, value| {
                args.projection = match value {
                    "first_angle" => Projection::FirstAngle,
                    _ => Projection::ThirdAngle,
                }
            },
        );
        f.checkbox(
            "hidden_lines",
            "Hidden lines",
            args.hidden_lines,
            |args, on| args.hidden_lines = on,
        );
        f.checkbox(
            "tangent_edges",
            "Tangent edges",
            args.tangent_edges,
            |args, on| args.tangent_edges = on,
        );
        f.reference(
            "section",
            "section plane",
            args.section.iter().cloned().collect(),
            &[Role::Plane],
            None,
            false,
            |edit, picked| edit.args.section = picked.into_iter().next(),
        );
        f.optional("section");

        f.heading("sheet_heading", "Sheet");
        f.select(
            "sheet",
            "paper",
            args.sheet.name(),
            SheetSize::ALL
                .iter()
                .map(|s| Choice::new(s.name(), s.name().to_uppercase()))
                .collect(),
            false,
            |args, value| {
                if let Some(s) = SheetSize::ALL.into_iter().find(|s| s.name() == value) {
                    args.sheet = s;
                }
            },
        );
        let mut scales = vec![Choice::new("fit", "To fit")];
        scales.extend(
            SCALES
                .iter()
                .map(|&s| Choice::new(scale_label(s), scale_label(s))),
        );
        f.select(
            "scale",
            "scale",
            args.scale.map(scale_label).unwrap_or_else(|| "fit".into()),
            scales,
            false,
            |args, value| {
                args.scale = SCALES.iter().copied().find(|&s| scale_label(s) == value);
            },
        );
        f.checkbox("bom", "Bill of materials", args.bom, |args, on| {
            args.bom = on
        });
        let mut title = Vec::new();
        for (key, caption, value) in [
            ("title:name", "Part name", &args.name),
            ("title:material", "Material", &args.material),
        ] {
            f.on(key, move |edit, value| {
                if let Value::Text(text) = value {
                    match key {
                        "title:name" => edit.args.name = text,
                        _ => edit.args.material = text,
                    }
                }
            });
            let mut item = ListItem::new(key, caption);
            item.text = Some(value.clone());
            title.push(item);
        }
        f.list("title", title, "");

        f.heading("dimensions_heading", "Dimensions");
        let items = args
            .dimensions
            .iter()
            .enumerate()
            .map(|(i, dimension)| {
                let key = format!("dimension:{i}");
                f.on(key.clone(), move |edit, value| {
                    if matches!(value, Value::Remove) && i < edit.args.dimensions.len() {
                        edit.args.dimensions.remove(i);
                    }
                });
                let mut item = ListItem::new(key, dimension.label());
                item.removable = true;
                item
            })
            .collect();
        f.list("dimensions", items, "Only the overall sizes.");
        f.reference(
            "distance",
            "distance between vertices",
            session.pending.clone(),
            &[Role::Point],
            None,
            true,
            |edit, picked| {
                let vertices: Vec<String> = picked
                    .iter()
                    .filter_map(|e| match e {
                        EntityRef::Vertex { name } => Some(name.clone()),
                        _ => None,
                    })
                    .collect();
                if let [from, to] = vertices.as_slice() {
                    edit.args.dimensions.push(Dimension::Distance {
                        from: from.clone(),
                        to: to.clone(),
                    });
                    edit.session.pending.clear();
                } else {
                    edit.session.pending = picked
                        .into_iter()
                        .filter(|e| matches!(e, EntityRef::Vertex { .. }))
                        .collect();
                }
            },
        );
        f.optional("distance");
        f.reference(
            "radius",
            "radius of a circle",
            Vec::new(),
            &[Role::Circle],
            None,
            false,
            |edit, picked: Vec<EntityRef>| {
                if let Some(edge) = picked_edge(&picked) {
                    edit.args.dimensions.push(Dimension::Radius { edge });
                }
            },
        );
        f.optional("radius");
        f.reference(
            "diameter",
            "diameter of a circle",
            Vec::new(),
            &[Role::Circle],
            None,
            false,
            |edit, picked: Vec<EntityRef>| {
                if let Some(edge) = picked_edge(&picked) {
                    edit.args.dimensions.push(Dimension::Diameter { edge });
                }
            },
        );
        f.optional("diameter");
        f.text(
            "export_hint",
            "Export the drawing as SVG or DXF from the program's menu.",
            Tone::Hint,
        );
        f
    }

    /// Checks that what the drawing refers to is there; it changes nothing.
    fn apply<S: Scalar>(
        &self,
        part: Part<S>,
        operation_id: &str,
        args: &DrawingArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("drawing({operation_id})");
        if args.views.is_empty() && args.section.is_none() {
            return Err(GeopError::new("a drawing needs a view")).with_context(ctx);
        }
        if let Some(plane) = &args.section {
            plane.resolve_plane(&part).with_context(ctx)?;
        }
        if let Some(scale) = args.scale
            && !(scale.is_finite() && scale > 0.0)
        {
            return Err(GeopError::new(format!(
                "the scale {scale} is not a positive number"
            )))
            .with_context(ctx);
        }
        for dimension in &args.dimensions {
            match dimension {
                Dimension::Distance { from, to } => {
                    placed_vertex(&part, from).with_context(ctx)?;
                    placed_vertex(&part, to).with_context(ctx)?;
                }
                Dimension::Radius { edge } | Dimension::Diameter { edge } => {
                    let curve = placed_edge(&part, edge).with_context(ctx)?;
                    if curve.as_arc().with_context(ctx)?.is_none() {
                        return Err(GeopError::new(format!(
                            "the edge {edge} is not circular: only a circle has a radius"
                        )))
                        .with_context(ctx);
                    }
                }
            }
        }
        Ok(part)
    }
}
