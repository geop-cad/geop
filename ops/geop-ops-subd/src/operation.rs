//! [`Subd`]: a freeform body shaped by its control cage, built as the
//! cage's Catmull–Clark limit surface — a solid for a closed cage, a sheet
//! for an open one.
//!
//! The step holds the cage itself, not a history of edits: edits are made
//! in the editor and leave the cage they made behind. Its vertices, edges
//! and faces are drawn, and picked by clicking them; what is selected is
//! moved, turned and scaled about its centre with the gizmo there — along
//! the world's axes, or, for faces, their own — or by typing its centre's
//! coordinates, a scale and turns; and edited with the actions: extruding
//! faces, inserting an edge loop across an edge, creasing edges or
//! smoothing them, deleting faces. An element not selected is dragged on
//! its own, in the plane facing the eye.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::{Ring, Scalar},
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    operation::{OperationGroup, OperationInfo, OperationTier},
    step_editor::DRAG_SNAP,
};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{BuildReader, PartOperation, StepContext},
    part_operation,
    ui::{
        Action, CanvasEvent, Edit, EditForm, FormState, Gizmo, Number, Shape, Style, Tone, Unit,
        Visual, choices, chosen,
    },
};
use serde::{Deserialize, Serialize};

use crate::{
    brep::limit_body,
    cage::{Cage, Mesh, Mirror, edge_key, face_key, vertex_key},
    edit::Element,
};

/// Builds the limit surface of `cage` — mirrored in `mirror`'s plane, if
/// any — as the step `S`: a solid named `subd(S)`, or a sheet. What it is
/// made of is named after the cage elements, see [`crate::brep`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Subd;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SubdArgs {
    pub cage: Cage,
    /// The plane the cage is the half of a symmetric body on.
    #[serde(default)]
    pub mirror: Mirror,
}

/// What a subd step keeps while it is edited.
pub type SubdEditing = FormState<SubdSession>;

/// What a scale or turn of the selection is applied to: the cage as it was
/// when the selection started being scaled or turned, so that a new value
/// replaces the old one rather than adding to it.
#[derive(Clone, Debug)]
struct Relative {
    selection: Vec<String>,
    base: Cage,
    scale: f64,
    /// About `x`, `y` and `z`, in degrees, in that order.
    angles: [f64; 3],
}

/// What editing a subd step keeps besides its arguments and the keys of
/// the cage elements selected.
#[derive(Clone, Debug, Default)]
pub struct SubdSession {
    relative: Option<Relative>,
    /// How far an extrude goes; [`DEFAULT_DISTANCE`] until set.
    distance: Option<f64>,
    /// Why the last edit could not be made.
    problem: Option<String>,
}

const DEFAULT_DISTANCE: f64 = 0.5;

/// The primitives a cage starts from, by the value of their action.
fn primitive(name: &str) -> Option<Cage> {
    Some(match name {
        "box" => Cage::cuboid([2.0, 2.0, 2.0]),
        "cylinder" => Cage::cylinder(1.0, 2.0, 8),
        "sphere" => Cage::sphere(1.0),
        "plane" => Cage::plane(2.0, 3),
        _ => return None,
    })
}

fn vector<S: Scalar>(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// `p` turned by `angles` degrees about `x`, then `y`, then `z`.
fn rotate(p: [f64; 3], angles: [f64; 3]) -> [f64; 3] {
    let mut p = p;
    for (axis, angle) in angles.iter().enumerate() {
        let (s, c) = angle.to_radians().sin_cos();
        let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
        let (a, b) = (p[i], p[j]);
        p[i] = c * a - s * b;
        p[j] = s * a + c * b;
    }
    p
}

/// Keeps only the keys of elements `cage` still has.
fn prune(selection: &mut Vec<String>, cage: &Cage) {
    selection.retain(|key| Element::parse(key).is_some_and(|e| cage.has(e)));
}

/// The edges and faces `selection` names.
fn selected(selection: &[String]) -> (Vec<[u32; 2]>, BTreeSet<u32>) {
    let mut edges = Vec::new();
    let mut faces = BTreeSet::new();
    for key in selection {
        match Element::parse(key) {
            Some(Element::Edge(a, b)) => edges.push([a, b]),
            Some(Element::Face(f)) => {
                faces.insert(f);
            }
            _ => {}
        }
    }
    (edges, faces)
}

/// The cage drawn: every vertex, edge and face of the half the step holds,
/// selectable and draggable, creased edges drawn fixed; the mirror image,
/// if any, as reference.
fn cage_visuals<S: Scalar>(args: &SubdArgs) -> Vec<Visual<S>> {
    let cage = &args.cage;
    let mut visuals = Vec::new();
    let at = |v: u32| cage.vertex(v).map(|x| x.at).ok();
    for face in &cage.faces {
        let points: Option<Vec<[f64; 3]>> = face.vertices.iter().map(|&v| at(v)).collect();
        let Some(points) = points else { continue };
        let triangles = (1..points.len().saturating_sub(1))
            .map(|k| [points[0], points[k], points[k + 1]].map(vector))
            .collect();
        visuals.push(
            Visual::new(
                face_key(face.id),
                Shape::Triangles { triangles },
                Style::Region,
            )
            .selectable()
            .draggable_from_grab(),
        );
    }
    for [a, b] in cage.edges() {
        let (Some(p), Some(q)) = (at(a), at(b)) else {
            continue;
        };
        let style = if cage.is_crease(a, b) {
            Style::Fixed
        } else {
            Style::Free
        };
        visuals.push(
            Visual::new(
                edge_key(a, b),
                Shape::Polyline {
                    points: vec![vector(p), vector(q)],
                },
                style,
            )
            .selectable()
            .draggable_from_grab(),
        );
    }
    for v in &cage.vertices {
        visuals.push(
            Visual::new(
                vertex_key(v.id),
                Shape::Point { at: vector(v.at) },
                Style::Free,
            )
            .selectable()
            .draggable_from_grab(),
        );
    }
    if let Some(axis) = args.mirror.axis()
        && let Ok(mesh) = Mesh::new(cage, args.mirror)
    {
        for (f, face) in mesh.faces.iter().enumerate() {
            if !mesh.face_names[f].ends_with('m') {
                continue;
            }
            let n = face.len();
            for k in 0..n {
                let (a, b) = (face[k], face[(k + 1) % n]);
                // Each mirrored edge once, from its smaller index.
                if a < b || mesh.positions[a][axis] == 0.0 && mesh.positions[b][axis] == 0.0 {
                    visuals.push(Visual::new(
                        format!("mirror:{}", mesh.edge_name(a, b)),
                        Shape::Polyline {
                            points: vec![vector(mesh.positions[a]), vector(mesh.positions[b])],
                        },
                        Style::Reference,
                    ));
                }
            }
        }
    }
    visuals
}

impl Relative {
    /// The scale and turn the session holds for `selection` on `cage`, or
    /// a fresh one from the cage as it is.
    fn of(session: &mut SubdSession, selection: &[String], cage: &Cage) -> Relative {
        match session.relative.take() {
            Some(r) if r.selection == selection => r,
            _ => Relative {
                selection: selection.to_vec(),
                base: cage.clone(),
                scale: 1.0,
                angles: [0.0; 3],
            },
        }
    }

    /// The base cage, the selection scaled and turned about its centre.
    fn apply(&self, mirror: Mirror) -> Cage {
        let mut cage = self.base.clone();
        let vertices = cage.vertices_of(&self.selection);
        if let Some(centre) = cage.centre(&vertices) {
            let (scale, angles) = (self.scale, self.angles);
            cage.transform(&vertices, mirror, |p| {
                let d = rotate([0, 1, 2].map(|c| p[c] - centre[c]), angles);
                [0, 1, 2].map(|c| centre[c] + scale * d[c])
            });
        }
        cage
    }
}

/// The dialog of a subd step: the cage to start from, the mirror, what is
/// selected and how to move, turn and scale it, and the edits; the cage
/// drawn.
fn form<'a>(
    args: &SubdArgs,
    session: &SubdSession,
    selection: &[String],
) -> EditForm<'a, Design, SubdArgs, SubdSession> {
    let mut f = EditForm::<Design, SubdArgs, SubdSession>::default();
    let start = ["box", "cylinder", "sphere", "plane"]
        .into_iter()
        .map(|p| {
            let label = format!("{}{}", p[..1].to_uppercase(), &p[1..]);
            Action::new(p, label)
                .group("Start from")
                .title("Replace the cage with this one")
        })
        .collect();
    f.actions("start", start, |edit, value| {
        let Some(mut cage) = primitive(value) else {
            return;
        };
        edit.session.problem = None;
        if let Some(axis) = edit.args.mirror.axis()
            && let Err(e) = cage.halve(axis)
        {
            edit.session.problem = Some(e.root_message().to_string());
            return;
        }
        edit.args.cage = cage;
        edit.session.relative = None;
        edit.selection.clear();
    });
    // Each choice of mirror: what the dialog sends, and what it shows.
    const MIRRORS: [(Mirror, &str, &str); 4] = [
        (Mirror::None, "none", "none"),
        (Mirror::X, "x", "in x = 0"),
        (Mirror::Y, "y", "in y = 0"),
        (Mirror::Z, "z", "in z = 0"),
    ];
    let (mirror, options) = choices(&MIRRORS, &args.mirror);
    f.select(
        "mirror",
        "mirror",
        mirror,
        options,
        false,
        move |edit, value| {
            let mirror = chosen(&MIRRORS, value).unwrap_or_default();
            if mirror == edit.args.mirror {
                return;
            }
            let changed = edit
                .args
                .cage
                .unmirrored(edit.args.mirror)
                .and_then(|mut cage| {
                    if let Some(axis) = mirror.axis() {
                        cage.halve(axis)?;
                    }
                    Ok(cage)
                });
            match changed {
                Ok(cage) => {
                    edit.args.cage = cage;
                    edit.args.mirror = mirror;
                    edit.session.problem = None;
                    edit.session.relative = None;
                    prune(edit.selection, &edit.args.cage);
                }
                Err(e) => edit.session.problem = Some(e.root_message().to_string()),
            }
        },
    );

    let vertices = args.cage.vertices_of(selection);
    let (edges, faces) = selected(selection);
    let count = |n: usize, one: &str, many: &str| match n {
        0 => None,
        1 => Some(format!("1 {one}")),
        n => Some(format!("{n} {many}")),
    };
    let vertex_count = selection
        .iter()
        .filter(|k| matches!(Element::parse(k), Some(Element::Vertex(_))))
        .count();
    let parts: Vec<String> = [
        count(vertex_count, "vertex", "vertices"),
        count(edges.len(), "edge", "edges"),
        count(faces.len(), "face", "faces"),
    ]
    .into_iter()
    .flatten()
    .collect();
    if parts.is_empty() {
        f.form.dialog.text(
                "selection",
                "Click vertices, edges and faces of the cage to select them, then move, turn or scale them with the gizmo; drag one to move it alone.",
                Tone::Hint,
            );
    } else {
        f.form.dialog.text(
            "selection",
            format!("{} selected", parts.join(", ")),
            Tone::Normal,
        );
    }
    if let Some(problem) = &session.problem {
        f.form.dialog.text("problem", problem.clone(), Tone::Error);
    }

    if let Some(centre) = args.cage.centre(&vertices) {
        let mut gizmo = Gizmo::new(vector(centre)).translate().rotate().scale();
        if let Some(axes) = args.cage.frame(&faces) {
            gizmo = gizmo.local(axes.map(vector));
        }
        f.form.gizmo = Some(gizmo);
        for (axis, key) in ["x", "y", "z"].into_iter().enumerate() {
            let number = Number::new(key, centre[axis], Unit::Length);
            f.number(key, number, move |edit, to| {
                let cage = &mut edit.args.cage;
                let vertices = cage.vertices_of(edit.selection);
                let Some(centre) = cage.centre(&vertices) else {
                    return;
                };
                let by = to - centre[axis];
                cage.transform(&vertices, edit.args.mirror, |mut p| {
                    p[axis] += by;
                    p
                });
                edit.session.relative = None;
            });
        }
        let relative = session
            .relative
            .as_ref()
            .filter(|r| r.selection == selection);
        let scale = relative.map_or(1.0, |r| r.scale);
        let scale_number = Number::new("scale", scale, Unit::Fraction);
        f.number("scale", scale_number, |edit, scale| {
            let mut relative = Relative::of(edit.session, edit.selection, &edit.args.cage);
            relative.scale = scale;
            edit.args.cage = relative.apply(edit.args.mirror);
            edit.session.relative = Some(relative);
        });
        for (axis, key) in ["turn_x", "turn_y", "turn_z"].into_iter().enumerate() {
            let angle = relative.map_or(0.0, |r| r.angles[axis]);
            let label = format!("turn about {}", ["x", "y", "z"][axis]);
            let number = Number::new(label, angle, Unit::Angle);
            f.number(key, number, move |edit, angle| {
                let mut relative = Relative::of(edit.session, edit.selection, &edit.args.cage);
                relative.angles[axis] = angle;
                edit.args.cage = relative.apply(edit.args.mirror);
                edit.session.relative = Some(relative);
            });
        }
    }

    if !faces.is_empty() {
        let distance = session.distance.unwrap_or(DEFAULT_DISTANCE);
        let number = Number::new("extrude distance", distance, Unit::Length);
        f.number("distance", number, |edit, d| {
            edit.session.distance = Some(d)
        });
    }
    let action = |value: &str, label: &str, enabled: bool, why: &str, title: &str| {
        let action = Action::new(value, label).group("Edit");
        if enabled {
            action.title(title)
        } else {
            action.disabled(why)
        }
    };
    let creased = edges
        .iter()
        .filter(|&&[a, b]| args.cage.is_crease(a, b))
        .count();
    let edits = vec![
        action(
            "extrude",
            "Extrude",
            !faces.is_empty(),
            "Select faces to extrude",
            "Extrude the selected faces along their normals",
        ),
        action(
            "insert_loop",
            "Insert loop",
            !edges.is_empty(),
            "Select an edge to insert a loop across",
            "Insert an edge loop across each selected edge",
        ),
        action(
            "crease",
            "Crease",
            creased < edges.len(),
            "Select smooth edges to crease",
            "Keep the selected edges sharp",
        ),
        action(
            "smooth",
            "Smooth",
            creased > 0,
            "Select creased edges to smooth",
            "Make the selected edges smooth again",
        ),
        action(
            "delete",
            "Delete",
            !faces.is_empty(),
            "Select faces to delete",
            "Delete the selected faces, opening the cage",
        ),
    ];
    f.actions("edit", edits, |edit, value| {
        let Edit {
            args,
            session,
            selection,
            ..
        } = edit;
        let (edges, faces) = selected(selection);
        let mut cage = args.cage.clone();
        let done: GeopResult<()> = match value {
            "extrude" => cage.extrude(
                &faces,
                session.distance.unwrap_or(DEFAULT_DISTANCE),
                args.mirror,
            ),
            "insert_loop" => edges.iter().try_for_each(|&[a, b]| cage.insert_loop(a, b)),
            "crease" => {
                cage.set_crease(&edges, true);
                Ok(())
            }
            "smooth" => {
                cage.set_crease(&edges, false);
                Ok(())
            }
            "delete" => {
                cage.delete_faces(&faces);
                Ok(())
            }
            _ => return,
        };
        match done {
            Ok(()) => {
                args.cage = cage;
                session.problem = None;
                session.relative = None;
                prune(selection, &args.cage);
            }
            Err(e) => session.problem = Some(e.root_message().to_string()),
        }
    });
    f.form.visuals = cage_visuals(args);
    f.form.selection = selection.to_vec();
    f
}

/// A cage element dragged: what is selected, if it is, else it alone,
/// moved as far as the pointer from where it was grabbed, snapped to
/// [`DRAG_SNAP`] unless shift is held — to the cage as it was then, which
/// the editor gives each event of the drag (see
/// [`geop_ops::ui::Visual::from_grab`]). The gizmo dragged: what is selected moved, turned or scaled as the
/// drag did. Escape clears the selection; Delete deletes the faces
/// selected.
fn on_canvas(edit: Edit<'_, SubdArgs, SubdSession>, event: &CanvasEvent<Design>) {
    let Edit {
        args,
        session,
        selection,
        ..
    } = edit;
    match event {
        CanvasEvent::Move {
            key,
            from,
            to,
            shift,
            ..
        } => {
            let keys = if selection.contains(key) {
                selection.clone()
            } else {
                vec![key.clone()]
            };
            let by = [0, 1, 2].map(|c| {
                // Where the pointer is: a free choice, made by whoever
                // moved it.
                let d = to[c].sub(from[c]).to_f64();
                if *shift {
                    d
                } else {
                    (d / DRAG_SNAP).round() * DRAG_SNAP
                }
            });
            let vertices = args.cage.vertices_of(&keys);
            (args.cage).transform(&vertices, args.mirror, |p| [0, 1, 2].map(|c| p[c] + by[c]));
            session.relative = None;
        }
        CanvasEvent::Gizmo { drag, .. } => {
            let vertices = args.cage.vertices_of(selection);
            args.cage
                .transform(&vertices, args.mirror, |p| drag.apply_f64(p));
            session.relative = None;
        }
        CanvasEvent::Delete => {
            let (_, faces) = selected(selection);
            if !faces.is_empty() {
                args.cage.delete_faces(&faces);
                session.relative = None;
                prune(selection, &args.cage);
            }
        }
        _ => {}
    }
}

/// Builds the limit surface of `args`' cage into `model` as the step
/// `step` (see [`Subd`]): in any scalar, as a test builds it.
pub fn build_subd<S: Scalar>(
    model: &mut NamedModel<S>,
    step: &str,
    args: &SubdArgs,
) -> GeopResult<()> {
    let ctx = with_context!("subd({step})");
    let namer = Namer::new("subd", step)?;
    let mesh = Mesh::new(&args.cage, args.mirror).with_context(ctx)?;
    let (spec, names) = limit_body::<S>(&mesh, &namer).with_context(ctx)?;
    model.build_body(spec, names).with_context(ctx)?;
    Ok(())
}

part_operation!(Subd(SubdArgs));

impl PartOperation for Subd {
    const INFO: OperationInfo = OperationInfo::new(
        "subd",
        "SubD",
        "Shape a freeform body by dragging the vertices, edges and faces of a control cage, built as its smooth subdivision surface.",
        OperationGroup::Surface,
        OperationTier::Big,
    )
    .icon(crate::icons::SUBD);
    type Args = SubdArgs;
    type Session = SubdSession;

    /// A box cage, two units a side, around the origin.
    fn new_args(_: &StepContext) -> SubdArgs {
        SubdArgs {
            cage: Cage::cuboid([2.0, 2.0, 2.0]),
            mirror: Mirror::None,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &SubdArgs,
    ) -> GeopResult<()> {
        build_subd(model, step, args)
    }

    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &SubdArgs,
        editing: &SubdEditing,
    ) -> EditForm<'a, Design, SubdArgs, SubdSession> {
        form(args, &editing.session, &editing.selection)
    }

    fn canvas(_: &StepContext, edit: Edit<'_, SubdArgs, SubdSession>, event: &CanvasEvent<Design>) {
        on_canvas(edit, event)
    }
}
