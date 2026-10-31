//! [`Subd`]: a freeform body shaped by its control cage, built as the
//! cage's Catmull–Clark limit surface — a solid for a closed cage, a sheet
//! for an open one.
//!
//! The step holds the cage itself, not a history of edits: edits are made
//! in the editor and leave the cage they made behind. A new cage is a
//! basic shape — a box, a cylinder, a sphere or a plane — [`SIZE`] across,
//! whose width, depth and height (or radius) are typed in the dialog while
//! the cage is still that shape (see [`Basic::of`]). Its vertices, edges
//! and faces are drawn, and picked by clicking them — an edge's loop by
//! double-clicking it, its ring with shift held, everything in a window
//! dragged after `b`; what is selected is moved, turned and scaled about
//! its centre with the gizmo there — along the world's axes, or, for
//! faces, their own — or by typing its centre's coordinates, a scale and
//! turns; and edited with the actions: extruding and insetting faces,
//! bridging two of them, bevelling edges, cutting a loop across them,
//! sliding them, dissolving and merging, filling a hole, creasing edges
//! sharp or soft, deleting. An element not selected is dragged on its
//! own, in the plane facing the eye. The limit surface is drawn as the
//! part, the cage over it; "cage only" hides the surface.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState, choices, chosen},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{
        Action, CanvasEvent, DRAG_SNAP, Gizmo, InHand, Number, Pointer, Shape, Style, Tone, Unit,
        Visual,
        hit::{Window, hit_visuals, no_instances},
    },
    part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{part::StepContext, reader::Reader};
use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops_model::{ModelOperation, NamedModel, Namer, build_model};
use serde::{Deserialize, Serialize};

use crate::{
    brep::{face_names, limit_body},
    cage::{Cage, Mesh, Mirror, edge_key, face_key, undirected, vertex_key},
    edit::Element,
};

/// Builds the limit surface of `cage` — mirrored in `mirror`'s plane, if
/// any — as the step `Scalar`: a solid named `subd(Scalar)`, or a sheet. What it is
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

/// How far each edit goes, as typed; the defaults until set.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Amounts {
    /// How far an extrude goes.
    pub distance: f64,
    /// How far in an inset goes.
    pub thickness: f64,
    /// How wide a bevel is.
    pub width: f64,
    /// Where across its ring a loop is cut, as a fraction.
    pub cut: f64,
    /// How far a loop slides, as a fraction of the edges it slides along.
    pub slide: f64,
}

impl Default for Amounts {
    /// For a cage [`SIZE`] across: an extrude a quarter of it, an inset and
    /// a bevel a tenth.
    fn default() -> Self {
        Self {
            distance: SIZE / 4.0,
            thickness: SIZE / 10.0,
            width: SIZE / 10.0,
            cut: 0.5,
            slide: 0.25,
        }
    }
}

/// What editing a subd step keeps besides its arguments and the keys of
/// the cage elements selected.
#[derive(Clone, Debug, Default)]
pub struct SubdSession {
    relative: Option<Relative>,
    pub amounts: Amounts,
    /// Whether the next drag selects everything within its window.
    pub box_select: bool,
    /// The window being dragged, to draw.
    window: Option<(Pointer, Pointer)>,
    /// Whether the limit surface is hidden, to see the cage alone.
    pub cage_only: bool,
    /// Why the last edit could not be made.
    problem: Option<String>,
}

/// How large a basic shape a cage starts from is: across, in each
/// direction.
pub const SIZE: f64 = 100.0;

/// How many quads round a basic cylinder's cage.
const CYLINDER_SEGMENTS: usize = 8;

/// How many quads along each side of a basic plane's cage.
const PLANE_QUADS: usize = 3;

/// A basic shape a cage starts from, with its dimensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Basic {
    Box { width: f64, depth: f64, height: f64 },
    Cylinder { radius: f64, height: f64 },
    Sphere { radius: f64 },
    Plane { width: f64 },
}

impl Basic {
    /// The basic shape the action `name` starts from, [`SIZE`] across.
    pub fn named(name: &str) -> Option<Basic> {
        Some(match name {
            "box" => Basic::Box {
                width: SIZE,
                depth: SIZE,
                height: SIZE,
            },
            "cylinder" => Basic::Cylinder {
                radius: SIZE / 2.0,
                height: SIZE,
            },
            "sphere" => Basic::Sphere { radius: SIZE / 2.0 },
            "plane" => Basic::Plane { width: SIZE },
            _ => return None,
        })
    }

    /// Its cage, around the origin.
    pub fn cage(self) -> Cage {
        match self {
            Basic::Box {
                width,
                depth,
                height,
            } => Cage::cuboid([width, depth, height]),
            Basic::Cylinder { radius, height } => Cage::cylinder(radius, height, CYLINDER_SEGMENTS),
            Basic::Sphere { radius } => Cage::sphere(radius),
            Basic::Plane { width } => Cage::plane(width, PLANE_QUADS),
        }
    }

    /// Its cage, mirrored in `mirror`'s plane: the half the step holds.
    fn cage_in(self, mirror: Mirror) -> GeopResult<Cage> {
        let mut cage = self.cage();
        if let Some(axis) = mirror.axis() {
            cage.halve(axis)?;
        }
        Ok(cage)
    }

    /// The basic shape the cage `cage` — the half `mirror` mirrors, if it
    /// does — is, as it was started from: each kind built with the
    /// dimensions the whole cage's extent gives it, and compared. A cage
    /// edited since is none: its dimensions are no longer the shape's to
    /// set.
    fn of(cage: &Cage, mirror: Mirror) -> Option<Basic> {
        let whole = cage.unmirrored(mirror).ok()?;
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for v in &whole.vertices {
            for k in 0..3 {
                lo[k] = lo[k].min(v.at[k]);
                hi[k] = hi[k].max(v.at[k]);
            }
        }
        let [w, d, h] = [0, 1, 2].map(|k| hi[k] - lo[k]);
        [
            Basic::Box {
                width: w,
                depth: d,
                height: h,
            },
            Basic::Cylinder {
                radius: w / 2.0,
                height: h,
            },
            Basic::Sphere { radius: w / 2.0 },
            Basic::Plane { width: w },
        ]
        .into_iter()
        .find(|basic| basic.cage_in(mirror).is_ok_and(|c| c == *cage))
    }

    /// Its dimensions: each one's key in the dialog, its label and its
    /// value.
    fn dimensions(self) -> Vec<(&'static str, &'static str, f64)> {
        match self {
            Basic::Box {
                width,
                depth,
                height,
            } => vec![
                ("shape_width", "width", width),
                ("shape_depth", "depth", depth),
                ("shape_height", "height", height),
            ],
            Basic::Cylinder { radius, height } => vec![
                ("shape_radius", "radius", radius),
                ("shape_height", "height", height),
            ],
            Basic::Sphere { radius } => vec![("shape_radius", "radius", radius)],
            Basic::Plane { width } => vec![("shape_width", "width", width)],
        }
    }

    /// It with the dimension keyed `key` set to `value`.
    fn with(mut self, key: &str, value: f64) -> Basic {
        match (&mut self, key) {
            (Basic::Box { width, .. } | Basic::Plane { width }, "shape_width") => *width = value,
            (Basic::Box { depth, .. }, "shape_depth") => *depth = value,
            (Basic::Box { height, .. } | Basic::Cylinder { height, .. }, "shape_height") => {
                *height = value
            }
            (Basic::Cylinder { radius, .. } | Basic::Sphere { radius }, "shape_radius") => {
                *radius = value
            }
            _ => {}
        }
        self
    }
}

fn vector(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
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

/// What `selection` names: the vertices named as such, the edges, and the
/// faces.
#[derive(Clone, Debug, Default)]
struct Selected {
    vertices: BTreeSet<u32>,
    edges: Vec<[u32; 2]>,
    faces: BTreeSet<u32>,
}

fn selected(selection: &[String]) -> Selected {
    let mut s = Selected::default();
    for key in selection {
        match Element::parse(key) {
            Some(Element::Vertex(v)) => {
                s.vertices.insert(v);
            }
            Some(Element::Edge(a, b)) => s.edges.push([a, b]),
            Some(Element::Face(f)) => {
                s.faces.insert(f);
            }
            None => {}
        }
    }
    s
}

/// Adds the keys `keys` to `selection`, each once.
fn extend(selection: &mut Vec<String>, keys: impl IntoIterator<Item = String>) {
    for key in keys {
        if !selection.contains(&key) {
            selection.push(key);
        }
    }
}

/// The cage drawn: every vertex, edge and face of the half the step holds,
/// selectable and draggable, creased edges drawn fixed; the mirror image,
/// if any, as reference.
fn cage_visuals(args: &SubdArgs) -> Vec<Visual> {
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

/// The window `from`–`to` drawn where the cage is: a rectangle at the
/// depth of the cage's centre.
fn window_visual(args: &SubdArgs, from: &Pointer, to: &Pointer) -> Option<Visual> {
    let window = Window::new(from, to)?;
    let all: BTreeSet<u32> = args.cage.vertices.iter().map(|v| v.id).collect();
    let centre = vector(args.cage.centre(&all)?);
    let depth = window.depth(&centre);
    let mut points = window.corners(depth).to_vec();
    points.push(points[0]);
    Some(Visual::new(
        "window",
        Shape::Polyline { points },
        Style::Draft,
    ))
}

/// The names of what the step `step` builds from `args`, to hide: its
/// solid, and, for an open cage, each of its faces.
fn built_names(step: &str, args: &SubdArgs) -> Vec<String> {
    let Ok(namer) = Namer::new("subd", step) else {
        return Vec::new();
    };
    let mut names = vec![namer.root()];
    if let Ok(mesh) = Mesh::new(&args.cage, args.mirror) {
        names.extend(face_names(&mesh, &namer));
    }
    names
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

/// Makes the edit `f` of the cage: kept if it succeeds, with the selection
/// pruned to what is left; else its reason is shown.
fn apply(edit: &mut Edit<'_, SubdArgs, SubdSession>, f: impl FnOnce(&mut Cage) -> GeopResult<()>) {
    let mut cage = edit.args.cage.clone();
    match f(&mut cage) {
        Ok(()) => {
            edit.args.cage = cage;
            edit.session.problem = None;
            edit.session.relative = None;
            prune(edit.selection, &edit.args.cage);
        }
        Err(e) => edit.session.problem = Some(e.root_message().to_string()),
    }
}

/// An edit of the cage, ready to make.
type CageEdit = Box<dyn FnOnce(&mut Cage) -> GeopResult<()>>;

/// The edits by the value of their action: what each does to the cage
/// for the selection `s` with the amounts `amounts`, or `None` for a value
/// that is no edit.
fn edit_action(
    value: &str,
    s: &Selected,
    all_vertices: &BTreeSet<u32>,
    amounts: Amounts,
    mirror: Mirror,
) -> Option<CageEdit> {
    let s = s.clone();
    let all_vertices = all_vertices.clone();
    Some(match value {
        "extrude" => Box::new(move |cage| cage.extrude(&s.faces, amounts.distance, mirror)),
        "inset" => Box::new(move |cage| cage.inset(&s.faces, amounts.thickness, mirror)),
        "bridge" => Box::new(move |cage| {
            let mut faces = s.faces.iter();
            match (faces.next(), faces.next(), faces.next()) {
                (Some(&a), Some(&b), None) => cage.bridge(a, b, mirror),
                _ => Err(geop_core_math::geop_error::GeopError::new(
                    "bridge: select exactly two faces",
                )),
            }
        }),
        "loop_cut" => Box::new(move |cage| {
            // Two selected edges of one ring are one cut: the first cut
            // split the second edge already.
            for &[a, b] in &s.edges {
                if cage.has(Element::Edge(a, b)) {
                    cage.insert_loop(a, b, amounts.cut)?;
                }
            }
            Ok(())
        }),
        "slide" => Box::new(move |cage| cage.edge_slide(&s.edges, amounts.slide, mirror)),
        "bevel" => Box::new(move |cage| cage.bevel(&s.edges, amounts.width, mirror)),
        "dissolve" => Box::new(move |cage| {
            if !s.edges.is_empty() {
                cage.dissolve_edges(&s.edges, mirror)?;
            }
            if !s.vertices.is_empty() {
                cage.dissolve_vertices(&s.vertices, mirror)?;
            }
            Ok(())
        }),
        "merge" => Box::new(move |cage| cage.merge_vertices(&all_vertices, mirror)),
        "fill" => Box::new(move |cage| {
            let open = (s.edges.iter()).find(|&&[a, b]| cage.is_boundary(a, b));
            match open {
                Some(&[a, b]) => cage.fill_hole(a, b, mirror),
                None => Err(geop_core_math::geop_error::GeopError::new(
                    "fill: select an edge on the hole's rim",
                )),
            }
        }),
        "delete" => Box::new(move |cage| {
            let mut faces = s.faces.clone();
            faces.extend(cage.faces_along(&s.edges));
            faces.extend(cage.faces_around(&s.vertices));
            if faces.is_empty() {
                return Err(geop_core_math::geop_error::GeopError::new(
                    "delete: nothing selected",
                ));
            }
            cage.delete_faces(&faces);
            Ok(())
        }),
        _ => return None,
    })
}

/// The keys of the selected edges' loops, or their rings.
fn chains(cage: &Cage, edges: &[[u32; 2]], ring: bool) -> GeopResult<Vec<String>> {
    let mut keys = Vec::new();
    for &[a, b] in edges {
        let found: Vec<[u32; 2]> = if ring {
            (cage.edge_ring(a, b)?.edges.iter())
                .map(|&(x, y)| undirected(x, y))
                .collect()
        } else {
            cage.edge_loop(a, b)?
        };
        keys.extend(found.into_iter().map(|[x, y]| edge_key(x, y)));
    }
    Ok(keys)
}

/// The dialog of a subd step: the cage to start from, the mirror, what is
/// selected and how to move, turn and scale it, and the edits; the cage
/// drawn.
fn form<'a>(
    step: &str,
    args: &SubdArgs,
    session: &SubdSession,
    selection: &[String],
) -> EditForm<'a, SubdArgs, SubdSession> {
    let mut f = EditForm::<SubdArgs, SubdSession>::default();
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
        let Some(basic) = Basic::named(value) else {
            return;
        };
        match basic.cage_in(edit.args.mirror) {
            Ok(cage) => {
                edit.args.cage = cage;
                edit.session.problem = None;
                edit.session.relative = None;
                edit.selection.clear();
            }
            Err(e) => edit.session.problem = Some(e.root_message().to_string()),
        }
    });
    // While the cage is the basic shape it started from, its dimensions
    // are set: the shape built again with them.
    if let Some(basic) = Basic::of(&args.cage, args.mirror) {
        for (key, label, value) in basic.dimensions() {
            let number = Number::new(label, value, Unit::Length);
            f.number(key, number, move |edit, value| {
                if value <= 0.0 {
                    edit.session.problem = Some(format!("the {label} has to be more than 0"));
                    return;
                }
                match basic.with(key, value).cage_in(edit.args.mirror) {
                    Ok(cage) => {
                        edit.args.cage = cage;
                        edit.session.problem = None;
                        edit.session.relative = None;
                        prune(edit.selection, &edit.args.cage);
                    }
                    Err(e) => edit.session.problem = Some(e.root_message().to_string()),
                }
            });
        }
    }
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
    f.checkbox("cage_only", "cage only", session.cage_only, |edit, on| {
        edit.session.cage_only = on;
    });

    let vertices = args.cage.vertices_of(selection);
    let s = selected(selection);
    let count = |n: usize, one: &str, many: &str| match n {
        0 => None,
        1 => Some(format!("1 {one}")),
        n => Some(format!("{n} {many}")),
    };
    let parts: Vec<String> = [
        count(s.vertices.len(), "vertex", "vertices"),
        count(s.edges.len(), "edge", "edges"),
        count(s.faces.len(), "face", "faces"),
    ]
    .into_iter()
    .flatten()
    .collect();
    if parts.is_empty() {
        f.form.dialog.text(
            "selection",
            "Click vertices, edges and faces of the cage to select them; double-click an edge for its loop, with shift for its ring; press b and drag a window around several. Move, turn or scale the selection with the gizmo; drag one element to move it alone.",
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
    let select_action = |value: &str, label: &str, title: &str| {
        let action = Action::new(value, label).group("Select").title(title);
        if s.edges.is_empty() && value != "box" {
            action.disabled("Select an edge first")
        } else {
            action
        }
    };
    let select = vec![
        select_action(
            "loop",
            "Loop",
            "Select the edge loop through each selected edge",
        ),
        select_action(
            "ring",
            "Ring",
            "Select the edge ring across each selected edge",
        ),
        select_action(
            "box",
            "Box",
            "Drag a window to select what is within it (b)",
        )
        .active(session.box_select),
    ];
    f.actions("select", select, |edit, value| match value {
        "box" => {
            edit.session.box_select = !edit.session.box_select;
            edit.session.window = None;
        }
        "loop" | "ring" => {
            let s = selected(edit.selection);
            match chains(&edit.args.cage, &s.edges, value == "ring") {
                Ok(keys) => {
                    extend(edit.selection, keys);
                    edit.session.problem = None;
                }
                Err(e) => edit.session.problem = Some(e.root_message().to_string()),
            }
        }
        _ => {}
    });

    if let Some(centre) = args.cage.centre(&vertices) {
        let mut gizmo = Gizmo::new(vector(centre)).translate().rotate().scale();
        if let Some(axes) = args.cage.frame(&s.faces) {
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

    let amounts = session.amounts;
    if !s.faces.is_empty() {
        let number = Number::new("extrude distance", amounts.distance, Unit::Length);
        f.number("distance", number, |edit, d| {
            edit.session.amounts.distance = d
        });
        let number = Number::new("inset thickness", amounts.thickness, Unit::Length);
        f.number("thickness", number, |edit, d| {
            edit.session.amounts.thickness = d
        });
    }
    if !s.edges.is_empty() {
        let sharpness = args.cage.sharpness(s.edges[0][0], s.edges[0][1]);
        let number = Number::new("crease sharpness", sharpness, Unit::Fraction).range(0.0, 1.0);
        f.number("sharpness", number, |edit, sharpness| {
            let s = selected(edit.selection);
            edit.args.cage.set_sharpness(&s.edges, sharpness);
            edit.session.problem = None;
        });
        let number = Number::new("bevel width", amounts.width, Unit::Length);
        f.number("width", number, |edit, w| edit.session.amounts.width = w);
        let number = Number::new("loop cut at", amounts.cut, Unit::Fraction).range(0.0, 1.0);
        f.number("cut", number, |edit, t| edit.session.amounts.cut = t);
        let number = Number::new("slide by", amounts.slide, Unit::Fraction).range(-1.0, 1.0);
        f.number("slide", number, |edit, t| edit.session.amounts.slide = t);
    }
    let action = |value: &str, label: &str, enabled: bool, why: &str, title: &str| {
        let action = Action::new(value, label).group("Edit");
        if enabled {
            action.title(title)
        } else {
            action.disabled(why)
        }
    };
    let on_boundary = (s.edges.iter()).any(|&[a, b]| args.cage.is_boundary(a, b));
    let edits = vec![
        action(
            "extrude",
            "Extrude",
            !s.faces.is_empty(),
            "Select faces to extrude",
            "Extrude the selected faces along their normals (e)",
        ),
        action(
            "inset",
            "Inset",
            !s.faces.is_empty(),
            "Select faces to inset",
            "Inset the selected faces: a ring of new faces inside their rim (i)",
        ),
        action(
            "bridge",
            "Bridge",
            s.faces.len() == 2,
            "Select two faces to bridge",
            "Join the two selected faces by a tunnel between their rims",
        ),
        action(
            "loop_cut",
            "Loop cut",
            !s.edges.is_empty(),
            "Select an edge to cut a loop across",
            "Cut an edge loop across each selected edge, where \"loop cut at\" says",
        ),
        action(
            "slide",
            "Slide",
            !s.edges.is_empty(),
            "Select the edges of a loop to slide",
            "Slide the selected edges along the edges leaving them, by \"slide by\"",
        ),
        action(
            "bevel",
            "Bevel",
            !s.edges.is_empty(),
            "Select edges to bevel",
            "Replace each selected edge by a strip \"bevel width\" wide",
        ),
        action(
            "dissolve",
            "Dissolve",
            !s.edges.is_empty() || !s.vertices.is_empty(),
            "Select edges or vertices to dissolve",
            "Remove the selected edges and vertices, merging the faces around them",
        ),
        action(
            "merge",
            "Merge",
            vertices.len() >= 2,
            "Select at least two vertices to merge",
            "Merge the vertices of the selection into one at their centre",
        ),
        action(
            "fill",
            "Fill",
            on_boundary,
            "Select an edge on an open rim to fill",
            "Fill the hole the selected edge is on with one face",
        ),
        action(
            "delete",
            "Delete",
            !selection.is_empty(),
            "Select what to delete",
            "Delete the selected faces, and the faces along the selected edges and vertices",
        ),
    ];
    f.actions("edit", edits, move |mut edit, value| {
        let s = selected(edit.selection);
        let all = edit.args.cage.vertices_of(edit.selection);
        let mirror = edit.args.mirror;
        if let Some(run) = edit_action(value, &s, &all, edit.session.amounts, mirror) {
            apply(&mut edit, run);
        }
    });
    let mut visuals = cage_visuals(args);
    if let Some((from, to)) = &session.window {
        visuals.extend(window_visual(args, from, to));
    }
    f.form.visuals = visuals;
    f.form.selection = selection.to_vec();
    f.form.tool = if session.box_select {
        InHand::Strokes
    } else {
        InHand::Nothing
    };
    if session.cage_only {
        f.form.hidden = built_names(step, args);
    }
    f
}

/// A cage element dragged: what is selected, if it is, else it alone,
/// moved as far as the pointer from where it was grabbed, snapped to
/// [`DRAG_SNAP`] unless shift is held — to the cage as it was then, which
/// the editor gives each event of the drag (see
/// [`Visual::from_grab`]). The gizmo dragged: what is selected moved,
/// turned or scaled as the drag did. A double click on an edge selects its
/// loop, with shift its ring. With box select on (`b`), a drag selects
/// everything within its window, shift adding to the selection, and a click
/// selects as usual. `e` extrudes and `i` insets the selected faces.
/// Escape clears the selection and puts box select down; Delete deletes
/// what is selected.
fn on_canvas(mut edit: Edit<'_, SubdArgs, SubdSession>, event: &CanvasEvent) {
    match event {
        CanvasEvent::Move {
            key,
            from,
            to,
            shift,
            ..
        } => {
            let keys = if edit.selection.contains(key) {
                edit.selection.clone()
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
            let mirror = edit.args.mirror;
            let vertices = edit.args.cage.vertices_of(&keys);
            (edit.args.cage).transform(&vertices, mirror, |p| [0, 1, 2].map(|c| p[c] + by[c]));
            edit.session.relative = None;
        }
        CanvasEvent::Gizmo { drag, .. } => {
            let mirror = edit.args.mirror;
            let vertices = edit.args.cage.vertices_of(edit.selection);
            (edit.args.cage).transform(&vertices, mirror, |p| drag.apply_f64(p));
            edit.session.relative = None;
        }
        CanvasEvent::Delete => {
            let s = selected(edit.selection);
            let all = edit.args.cage.vertices_of(edit.selection);
            if let Some(run) =
                edit_action("delete", &s, &all, edit.session.amounts, edit.args.mirror)
            {
                apply(&mut edit, run);
            }
        }
        CanvasEvent::Click {
            pointer,
            double,
            shift,
        } => {
            let visuals = cage_visuals(edit.args);
            let hit = hit_visuals(&visuals, pointer, no_instances, |v| v.selectable)
                .map(|hit| hit.visual.key.clone());
            let Some(key) = hit else {
                return;
            };
            if *double {
                if let Some(Element::Edge(a, b)) = Element::parse(&key) {
                    match chains(&edit.args.cage, &[[a, b]], *shift) {
                        Ok(keys) => {
                            extend(edit.selection, keys);
                            edit.session.problem = None;
                        }
                        Err(e) => edit.session.problem = Some(e.root_message().to_string()),
                    }
                }
            } else if edit.session.box_select {
                // The editor leaves clicks to a tool in hand: select as it
                // would.
                match edit.selection.iter().position(|k| *k == key) {
                    Some(i) => {
                        edit.selection.remove(i);
                    }
                    None => edit.selection.push(key),
                }
            }
        }
        CanvasEvent::Stroke {
            from,
            to,
            done,
            shift,
        } => {
            if !edit.session.box_select {
                return;
            }
            if !*done {
                edit.session.window = Some((*from, *to));
                return;
            }
            edit.session.window = None;
            edit.session.box_select = false;
            let Some(window) = Window::new(from, to) else {
                return;
            };
            let cage = &edit.args.cage;
            let within = |v: u32| cage.vertex(v).is_ok_and(|x| window.contains(&vector(x.at)));
            let mut keys: Vec<String> = Vec::new();
            keys.extend(
                (cage.vertices.iter())
                    .filter(|v| within(v.id))
                    .map(|v| vertex_key(v.id)),
            );
            keys.extend(
                (cage.edges().into_iter())
                    .filter(|&[a, b]| within(a) && within(b))
                    .map(|[a, b]| edge_key(a, b)),
            );
            keys.extend(
                (cage.faces.iter())
                    .filter(|f| f.vertices.iter().all(|&v| within(v)))
                    .map(|f| face_key(f.id)),
            );
            if !*shift {
                edit.selection.clear();
            }
            extend(edit.selection, keys);
            edit.session.relative = None;
        }
        CanvasEvent::Key { key } => {
            let value = match key.as_str() {
                "b" => {
                    edit.session.box_select = !edit.session.box_select;
                    edit.session.window = None;
                    return;
                }
                "e" => "extrude",
                "i" => "inset",
                _ => return,
            };
            let s = selected(edit.selection);
            let all = edit.args.cage.vertices_of(edit.selection);
            if let Some(run) = edit_action(value, &s, &all, edit.session.amounts, edit.args.mirror)
            {
                apply(&mut edit, run);
            }
        }
        CanvasEvent::Cancel => {
            edit.session.box_select = false;
            edit.session.window = None;
        }
        _ => {}
    }
}

/// Builds the limit surface of `args`' cage into `model` as the step
/// `step` (see [`Subd`]), as a test builds it.
pub fn build_subd(model: &mut NamedModel, step: &str, args: &SubdArgs) -> GeopResult<()> {
    let ctx = with_context!("subd({step})");
    let namer = Namer::new("subd", step)?;
    let mesh = Mesh::new(&args.cage, args.mirror).with_context(ctx)?;
    let (spec, names) = limit_body(&mesh, &namer).with_context(ctx)?;
    model.build_body(spec, names).with_context(ctx)?;
    Ok(())
}

part_operation!(Subd(SubdArgs));

impl PartOperation for Subd {
    const KIND: &'static str = "subd";
    const INFO: OperationInfo = OperationInfo::new(
        "SubD",
        "Shape a freeform body by dragging the vertices, edges and faces of a control cage, built as its smooth subdivision surface.",
        OperationGroup::Surface,
        OperationTier::Big,
    )
    .icon(crate::icons::SUBD);
    type Args = SubdArgs;
    type Session = SubdSession;

    /// A box cage, [`SIZE`] a side, around the origin: its dimensions
    /// set in the dialog.
    fn new_args(_: &Context) -> SubdArgs {
        SubdArgs {
            cage: Basic::Box {
                width: SIZE,
                depth: SIZE,
                height: SIZE,
            }
            .cage(),
            mirror: Mirror::None,
        }
    }

    fn run(ctx: &mut StepContext, args: &SubdArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    fn form<'a>(
        ctx: &'a Context<'a>,
        args: &SubdArgs,
        editing: &SubdEditing,
    ) -> EditForm<'a, SubdArgs, SubdSession> {
        form(ctx.step, args, &editing.session, &editing.selection)
    }

    fn canvas(_: &Context, edit: Edit<'_, SubdArgs, SubdSession>, event: &CanvasEvent) {
        on_canvas(edit, event)
    }
}

impl ModelOperation for Subd {
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &SubdArgs) -> GeopResult<()> {
        build_subd(model, step, args)
    }
}
