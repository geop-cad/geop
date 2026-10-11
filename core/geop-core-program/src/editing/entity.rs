//! [`EntityRef`]: an entity of a part, by name, as a step's arguments and
//! the editor refer to it, and [`Role`]: what a step needs it to be. What
//! an entity is in a part is the model's to resolve.

use geop_core_math::primitives::DatumComponent;
use geop_core_sketch::{CurveId, PointId};
use serde::{Deserialize, Serialize};

/// What separates an instance's name from the name of an entity of the part
/// it places: `bolt/extrude(head,end)` is the face `extrude(head,end)` of the
/// part placed as `bolt` — where it is placed — and `asm/bolt/...` reaches
/// into an instance of an instance. Instances are named by their step's id,
/// which cannot contain it (see the program's step ids), so the
/// first one always ends the instance's name.
pub const INSTANCE_SEPARATOR: char = '/';

/// What the part placed as `instance` names `name`, as the part it is
/// placed in names it: `bolt/head`.
pub fn instance_path(instance: &str, name: &str) -> String {
    format!("{instance}{INSTANCE_SEPARATOR}{name}")
}

/// Something picked in the viewport, to build on or to use.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum EntityRef {
    Vertex {
        name: String,
    },
    Edge {
        name: String,
    },
    /// A face: planar, its plane — normal pointing out of its solid, sketch
    /// origin the world origin's projection onto it and sketch `x` the world
    /// axis most parallel to it, projected, so sketches on parallel faces
    /// line up. Round, its axis.
    Face {
        name: String,
    },
    /// A datum — or, of a frame, one of its axes or planes (see
    /// [`geop_core_math::primitives::Datum::component`]). Every part has
    /// the frame the frame `origin`.
    Datum {
        name: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        component: Option<DatumComponent>,
    },
    /// A solid, as a whole.
    Solid {
        name: String,
    },
    /// A sketch: its plane.
    Sketch {
        name: String,
    },
    /// A 3-D sketch, as a whole: its curves.
    Sketch3d {
        name: String,
    },
    /// A curve of a sketch, by its id in the sketch.
    SketchCurve {
        sketch: String,
        curve: CurveId,
    },
    /// A point of a sketch, by its id in the sketch.
    SketchPoint {
        sketch: String,
        point: PointId,
    },
    /// What a step did by combining tools with a solid — a cut, a boss, a
    /// hole — by the step's id: picked by a
    /// face it made.
    Feature {
        name: String,
    },
    /// A reference frame on an entity, for what is placed or moved by it —
    /// a mate's joint turns about the `z` axis of one: a planar face's at
    /// its middle, `z` along its normal; a cylinder's on its axis, half way
    /// along it; a circular edge's at its center, `z` along its axis; a
    /// straight edge's at its middle, `z` along it; a point's, with the
    /// world's axes. `at` says where else on `on` it sits: at a vertex of
    /// it, or at the middle of an edge of it — a face's frame at a corner,
    /// or half way along a side, with its `z` still the face's — or at an
    /// end of a straight edge.
    Frame {
        on: Box<EntityRef>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        at: Option<Box<EntityRef>>,
    },
}

impl EntityRef {
    /// A datum, as a whole.
    pub fn datum(name: impl Into<String>) -> Self {
        EntityRef::Datum {
            name: name.into(),
            component: None,
        }
    }

    /// A component of the frame datum `name`.
    pub fn datum_component(name: impl Into<String>, component: DatumComponent) -> Self {
        EntityRef::Datum {
            name: name.into(),
            component: Some(component),
        }
    }

    /// How the entity is shown: its name, and which component of a frame.
    pub fn label(&self) -> String {
        match self {
            EntityRef::Datum {
                name,
                component: Some(component),
            } => format!("{name} {component}"),
            EntityRef::SketchCurve { sketch, curve } => format!("{sketch} {curve}"),
            EntityRef::SketchPoint { sketch, point } => format!("{sketch} {point}"),
            EntityRef::Frame { on, at: None } => format!("frame on {}", on.label()),
            EntityRef::Frame { on, at: Some(at) } => {
                format!("frame on {} at {}", on.label(), at.label())
            }
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Datum { name, .. }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name }
            | EntityRef::Sketch3d { name }
            | EntityRef::Feature { name } => name.clone(),
        }
    }
}

impl EntityRef {
    /// The name it is found by: its own, or its sketch's. A frame has none
    /// of its own: it is found by those of its entity and where it sits.
    fn name_mut(&mut self) -> Option<&mut String> {
        match self {
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Datum { name, .. }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name }
            | EntityRef::Sketch3d { name }
            | EntityRef::Feature { name } => Some(name),
            EntityRef::SketchCurve { sketch, .. } | EntityRef::SketchPoint { sketch, .. } => {
                Some(sketch)
            }
            EntityRef::Frame { .. } => None,
        }
    }

    /// If it lies in a part placed in this one: the instance's name, and the
    /// entity as the placed part names it. A frame lies in the part its
    /// entity does, and so does where it sits.
    pub fn split_instance(&self) -> Option<(String, EntityRef)> {
        if let EntityRef::Frame { on, at } = self {
            let (instance, on) = on.split_instance()?;
            let at = match at {
                Some(at) => match at.split_instance() {
                    Some((of, at)) if of == instance => Some(Box::new(at)),
                    _ => return None,
                },
                None => None,
            };
            return Some((
                instance,
                EntityRef::Frame {
                    on: Box::new(on),
                    at,
                },
            ));
        }
        let mut inner = self.clone();
        let name = inner.name_mut()?;
        let (instance, rest) = name.split_once(INSTANCE_SEPARATOR)?;
        let instance = instance.to_string();
        *name = rest.to_string();
        Some((instance, inner))
    }

    /// The entity, of the part placed as `instance`, as the part it is
    /// placed in names it.
    pub fn in_instance(&self, instance: &str) -> EntityRef {
        if let EntityRef::Frame { on, at } = self {
            return EntityRef::Frame {
                on: Box::new(on.in_instance(instance)),
                at: at.as_ref().map(|at| Box::new(at.in_instance(instance))),
            };
        }
        let mut outer = self.clone();
        if let Some(name) = outer.name_mut() {
            *name = instance_path(instance, name);
        }
        outer
    }

    /// Whether it is `scope` or part of it: a curve or a point of a sketch,
    /// or a face, an edge or a vertex of a solid — `solid`, the name of the
    /// solid it bounds, if any, which only the part knows.
    pub fn lies_in(&self, scope: &EntityRef, solid: Option<&str>) -> bool {
        match (self, scope) {
            (EntityRef::Frame { on, .. }, _) if !matches!(scope, EntityRef::Frame { .. }) => {
                on.lies_in(scope, solid)
            }
            (
                EntityRef::SketchCurve { sketch, .. } | EntityRef::SketchPoint { sketch, .. },
                EntityRef::Sketch { name },
            ) => sketch == name,
            (
                EntityRef::Face { .. } | EntityRef::Edge { .. } | EntityRef::Vertex { .. },
                EntityRef::Solid { name },
            ) => solid == Some(name.as_str()),
            _ => self == scope,
        }
    }
}

impl std::fmt::Display for EntityRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EntityRef::Vertex { name } => write!(f, "vertex {name:?}"),
            EntityRef::Edge { name } => write!(f, "edge {name:?}"),
            EntityRef::Face { name } => write!(f, "face {name:?}"),
            EntityRef::Datum {
                name,
                component: None,
            } => write!(f, "datum {name:?}"),
            EntityRef::Datum {
                name,
                component: Some(component),
            } => write!(f, "the {component} of datum {name:?}"),
            EntityRef::Solid { name } => write!(f, "solid {name:?}"),
            EntityRef::Sketch { name } => write!(f, "sketch {name:?}"),
            EntityRef::Sketch3d { name } => write!(f, "3-D sketch {name:?}"),
            EntityRef::Feature { name } => write!(f, "feature {name:?}"),
            EntityRef::SketchCurve { sketch, curve } => {
                write!(f, "curve {curve} of sketch {sketch:?}")
            }
            EntityRef::SketchPoint { sketch, point } => {
                write!(f, "point {point} of sketch {sketch:?}")
            }
            EntityRef::Frame { on, at: None } => write!(f, "the frame on {on}"),
            EntityRef::Frame { on, at: Some(at) } => write!(f, "the frame on {on} at {at}"),
        }
    }
}

impl EntityRef {
    /// The vertex `name`.
    pub fn vertex(name: impl Into<String>) -> Self {
        EntityRef::Vertex { name: name.into() }
    }

    /// The edge `name`.
    pub fn edge(name: impl Into<String>) -> Self {
        EntityRef::Edge { name: name.into() }
    }

    /// The face `name`.
    pub fn face(name: impl Into<String>) -> Self {
        EntityRef::Face { name: name.into() }
    }

    /// The solid `name`.
    pub fn solid(name: impl Into<String>) -> Self {
        EntityRef::Solid { name: name.into() }
    }

    /// The name of what it refers to, if that is one thing named by one.
    fn plain_name(&self) -> Option<&str> {
        match self {
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name }
            | EntityRef::Sketch3d { name }
            | EntityRef::Feature { name } => Some(name),
            _ => None,
        }
    }

    /// `names` as the references `kind` ([`EntityRef::edge`], say) makes of
    /// them, as a reference field holds them: an empty name is none.
    pub fn of_names(kind: fn(String) -> Self, names: &[String]) -> Vec<Self> {
        names
            .iter()
            .filter(|name| !name.is_empty())
            .cloned()
            .map(kind)
            .collect()
    }

    /// The names of those of `picked` that are of the kind `kind` makes.
    pub fn names_of(kind: fn(String) -> Self, picked: &[Self]) -> Vec<String> {
        let kind = std::mem::discriminant(&kind(String::new()));
        picked
            .iter()
            .filter(|e| std::mem::discriminant(*e) == kind)
            .filter_map(|e| e.plain_name().map(str::to_string))
            .collect()
    }

    /// `name` as the reference `kind` makes of it, as a field picking one
    /// thing holds it: an empty name is none.
    pub fn of_name(kind: fn(String) -> Self, name: &str) -> Vec<Self> {
        Self::of_names(kind, std::slice::from_ref(&name.to_string()))
    }

    /// The name of the last of `picked` that is of the kind `kind` makes:
    /// what a field picking one thing takes from a pick.
    pub fn last_name(kind: fn(String) -> Self, picked: &[Self]) -> Option<String> {
        Self::names_of(kind, picked).pop()
    }

    /// The sketch `name`.
    pub fn sketch(name: impl Into<String>) -> Self {
        EntityRef::Sketch { name: name.into() }
    }

    /// The 3-D sketch `name`.
    pub fn sketch3d(name: impl Into<String>) -> Self {
        EntityRef::Sketch3d { name: name.into() }
    }
}

/// What a step needs an entity it builds on to be: what a pick looks for,
/// and what a datum construction needs as an input.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    /// A vertex, a datum point, a frame's origin, a sketch point.
    Point,
    /// A straight edge, a datum axis, a frame's axis, a sketch line.
    Line,
    /// A planar face, a datum plane, a frame's plane.
    Plane,
    /// An edge of a face — of a solid or of a sheet.
    Edge,
    /// Any curve: an edge, a 3-D sketch's line, arc or spline, a planar
    /// sketch's curve.
    Curve,
    /// A circular edge.
    Circle,
    /// Something that turns around an axis: a circular edge, a cylindrical,
    /// conical or spherical face, a sketch circle.
    Round,
    /// Any face, of a solid or standing on its own.
    Face,
    /// A solid, as a whole.
    Solid,
    /// A face standing on its own, part of no solid.
    Sheet,
    /// A sketch, as a whole.
    Sketch,
    /// Curves to run along: a planar or a 3-D sketch, as a whole.
    Path,
    /// What a step did by combining tools with a solid, picked by a face
    /// it made.
    Feature,
    /// A reference frame — what a mate's joint turns about or slides along:
    /// where a face, an edge, a point or a datum is picked, a frame is put
    /// on it.
    Frame,
}

impl Role {
    pub const ALL: [Role; 14] = [
        Role::Point,
        Role::Line,
        Role::Plane,
        Role::Edge,
        Role::Curve,
        Role::Circle,
        Role::Round,
        Role::Face,
        Role::Solid,
        Role::Sheet,
        Role::Sketch,
        Role::Path,
        Role::Feature,
        Role::Frame,
    ];

    /// What it can be, each alternative on its own, as a requirement
    /// reads: `a point`; `a circular edge`, `a round face`.
    fn alternatives(self) -> &'static [&'static str] {
        match self {
            Role::Point => &["a point"],
            Role::Line => &["a line"],
            Role::Plane => &["a plane"],
            Role::Edge => &["an edge"],
            Role::Curve => &["an edge", "a sketch curve"],
            Role::Circle => &["a circular edge"],
            Role::Round => &["a circular edge", "a round face"],
            Role::Face => &["a face"],
            Role::Solid => &["a solid"],
            Role::Sheet => &["a face on its own"],
            Role::Sketch => &["a sketch"],
            Role::Path => &["a sketch", "a 3-D sketch", "an edge"],
            Role::Feature => &["a feature, by a face it made"],
            Role::Frame => &["a face", "an edge", "a point", "a datum"],
        }
    }

    /// As a requirement reads: `a point`, `a circular edge or a round face`.
    pub fn describe(self) -> String {
        describe_roles(&[self])
    }

    /// Its name, as a selection lists what an entity can be used as.
    pub fn name(self) -> &'static str {
        match self {
            Role::Point => "point",
            Role::Line => "line",
            Role::Plane => "plane",
            Role::Edge => "edge",
            Role::Curve => "curve",
            Role::Circle => "circle",
            Role::Round => "round",
            Role::Face => "face",
            Role::Solid => "solid",
            Role::Sheet => "sheet",
            Role::Sketch => "sketch",
            Role::Path => "path",
            Role::Feature => "feature",
            Role::Frame => "frame",
        }
    }
}

/// `roles` in words, as a requirement reads, each alternative once:
/// `a point, a line or a plane`.
pub fn describe_roles(roles: &[Role]) -> String {
    let mut words: Vec<&str> = Vec::new();
    for word in roles.iter().flat_map(|role| role.alternatives()) {
        if !words.contains(word) {
            words.push(word);
        }
    }
    match words.split_last() {
        None => String::new(),
        Some((last, [])) => last.to_string(),
        Some((last, rest)) => format!("{} or {last}", rest.join(", ")),
    }
}
