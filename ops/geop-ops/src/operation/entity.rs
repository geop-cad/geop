//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face, datum, solid or sketch of the part, by name — or of a part
//! placed in it, by its name there behind the instance's
//! ([`INSTANCE_SEPARATOR`]).

use super::{Chain, piece_names};
use crate::Part;
use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::{CurveId, PointId, Shape};
use geop_core_topology::Body;
use serde::{Deserialize, Serialize};

/// What separates an instance's name from the name of an entity of the part
/// it places: `bolt/extrude(head,end)` is the face `extrude(head,end)` of the
/// part placed as `bolt` — where it is placed — and `asm/bolt/...` reaches
/// into an instance of an instance. Instances are named by their step's id,
/// which cannot contain it (see [`crate::validate_operation_id`]), so the
/// first one always ends the instance's name.
pub const INSTANCE_SEPARATOR: char = '/';

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
    /// the frame [`ORIGIN`](crate::ORIGIN).
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
    /// hole — by the step's id (see [`crate::part::Feature`]): picked by a
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
    /// end of a straight edge (see [`crate::operation::Aspects::frame_on`]).
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
        let name = outer.name_mut().expect("every other entity has a name");
        *name = format!("{instance}{INSTANCE_SEPARATOR}{name}");
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

/// A curve picked to build on (see [`EntityRef::resolve_curve`]): where it
/// runs, and the names of it and of its ends, with where they are — what
/// is built from it is named after these.
#[derive(Clone, Debug)]
pub struct NamedCurve<S: Scalar> {
    pub name: String,
    pub curve: NurbCurve3D<S>,
    pub start: (String, Vector3<S>),
    pub end: (String, Vector3<S>),
}

/// A right-handed orthonormal frame at `origin` with `w` along `normal`,
/// and `u` the world axis most parallel to the plane normal to it,
/// projected into that plane — so frames with parallel normals line up.
pub fn frame_along<S: Scalar>(
    origin: Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<CoordinateSystem<S>> {
    let n = normal.normalize()?;
    let axis = (0..3)
        .min_by(|&a, &b| n[a].to_f64().abs().total_cmp(&n[b].to_f64().abs()))
        .expect("three axes");
    let mut a = Vector3::zero();
    a[axis] = S::ONE;
    let u = a.sub(&n.prod_scalar(n.prod_dot(&a))).normalize()?;
    let v = n.prod_cross(&u);
    CoordinateSystem::try_new(origin, u, v, n)
}

impl EntityRef {
    /// The edge `name`.
    pub fn edge(name: impl Into<String>) -> Self {
        EntityRef::Edge { name: name.into() }
    }

    /// The face `name`.
    pub fn face(name: impl Into<String>) -> Self {
        EntityRef::Face { name: name.into() }
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

    /// The sketch `name`.
    pub fn sketch(name: impl Into<String>) -> Self {
        EntityRef::Sketch { name: name.into() }
    }

    /// The 3-D sketch `name`.
    pub fn sketch3d(name: impl Into<String>) -> Self {
        EntityRef::Sketch3d { name: name.into() }
    }

    /// The datum it refers to in `part` — for a component of a frame, that
    /// component as a datum of its own. Fails if it is no datum of the
    /// part.
    pub fn resolve_datum<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Datum<S>> {
        let ctx = with_context!("resolving {self}");
        if let Some((name, inner)) = self.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            let datum = inner.resolve_datum(instance.part()).with_context(ctx)?;
            return Ok(Datum {
                kind: datum.kind,
                frame: instance.pose.motion().apply_frame(&datum.frame)?,
            });
        }
        let EntityRef::Datum { name, component } = self else {
            return Err(GeopError::new(format!("{self} is not a datum")));
        };
        let datum = part.datum(part.datum_id(name).with_context(ctx)?)?;
        match component {
            Some(component) => datum.component(*component).with_context(ctx),
            None => Ok(datum.clone()),
        }
    }

    /// The plane it lies in, as a frame with `u`/`v` a sketch's `x`/`y` on
    /// it and `w = u x v` its normal: a planar face's — normal pointing out
    /// of its solid, origin the world origin's projection onto it and `u`
    /// the world axis most parallel to it, so sketches on parallel faces
    /// line up — a datum plane's, a frame's plane's, or a sketch's. Fails
    /// for anything else.
    pub fn resolve_plane<S: Scalar>(&self, part: &Part<S>) -> GeopResult<CoordinateSystem<S>> {
        let ctx = with_context!("resolving the plane of {self}");
        if let Some((name, inner)) = self.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            let plane = inner.resolve_plane(instance.part()).with_context(ctx)?;
            return instance.pose.motion().apply_frame(&plane);
        }
        match self {
            EntityRef::Face { name } => {
                let id = part.face_id(name).with_context(ctx)?;
                let surface = &part.topology().get_face(id).with_context(ctx)?.surface;
                match surface.as_plane().with_context(ctx)? {
                    Some(plane) => frame_along(plane.project(&Vector3::zero()), &plane.normal),
                    None => Err(GeopError::new(format!("{self} is not planar"))),
                }
            }
            EntityRef::Datum { .. } => {
                let datum = self.resolve_datum(part)?;
                match datum.kind {
                    DatumKind::Plane => Ok(datum.frame),
                    _ => Err(GeopError::new(format!("{self} is not a plane"))),
                }
            }
            EntityRef::Sketch { name } => {
                let id = part.sketch_id(name).with_context(ctx)?;
                Ok(part.sketch(id).with_context(ctx)?.plane.clone())
            }
            _ => Err(GeopError::new(format!("{self} is not planar"))),
        }
    }

    /// The curves it refers to in `part`, as a path, a rail or a guide runs
    /// along them: an edge, of a face or of a wire, or a planar sketch's
    /// curve, a chain of its own; a sketch's or a 3-D sketch's one open
    /// chain of curves, or its one loop. In space, named after the entities
    /// they are: an edge's own and its vertices', a sketch curve `c3` of
    /// sketch `K` `K,c3`, its ends' `K,p1` after the points there, and a
    /// closed one's `K,c3,seam`; a sketch's pieces and joints the same, as
    /// a profile's are. Fails for anything else.
    pub fn resolve_chain<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Chain<S>> {
        let ctx = with_context!("resolving the curves of {self}");
        if let Some((name, inner)) = self.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            let chain = inner.resolve_chain(instance.part()).with_context(ctx)?;
            let motion = instance.pose.motion();
            let prefix = |n: &String| format!("{name}{INSTANCE_SEPARATOR}{n}");
            return Ok(Chain {
                name: prefix(&chain.name),
                curves: chain.curves.iter().map(|c| c.transform(&motion)).collect(),
                curve_names: chain.curve_names.iter().map(prefix).collect(),
                joint_names: chain.joint_names.iter().map(prefix).collect(),
            });
        }
        match self {
            EntityRef::Edge { name } => {
                let model = part.topology();
                let edge = model.get_edge(part.edge_id(name).with_context(ctx)?)?;
                let end = |v| {
                    part.name_of(v).map(str::to_string).ok_or_else(|| {
                        GeopError::new(format!("{v}, an end of edge {name:?}, has no name"))
                    })
                };
                Ok(Chain {
                    name: name.clone(),
                    curves: vec![edge.curve.clone()],
                    curve_names: vec![name.clone()],
                    joint_names: vec![
                        end(edge.start_vertex).with_context(ctx)?,
                        end(edge.end_vertex).with_context(ctx)?,
                    ],
                })
            }
            EntityRef::SketchCurve { sketch, curve } => {
                let placed = part.sketch(part.sketch_id(sketch).with_context(ctx)?)?;
                let geometry = placed.sketch.enclose::<S>().with_context(ctx)?;
                let class = placed.sketch.point_classes();
                let in_space = placed.curve_in_space(*curve, &geometry).with_context(ctx)?;
                let joint_names = match placed.sketch.curve(*curve)?.endpoints() {
                    Some((s, e)) => vec![
                        format!("{sketch},{}", class[&s]),
                        format!("{sketch},{}", class[&e]),
                    ],
                    None => vec![format!("{sketch},{curve},seam")],
                };
                Ok(Chain {
                    name: format!("{sketch},{curve}"),
                    curves: vec![in_space],
                    curve_names: vec![format!("{sketch},{curve}")],
                    joint_names,
                })
            }
            EntityRef::Sketch { name } => {
                let placed = part.sketch(part.sketch_id(name).with_context(ctx)?)?;
                let sketch = &placed.sketch;
                let geometry = sketch.enclose::<S>().with_context(ctx)?;
                let (lp, closed) = match sketch.shape().with_context(ctx)? {
                    Shape::Chain(chain) => (chain, false),
                    Shape::Region(region) if region.holes.is_empty() => (region.outer, true),
                    Shape::Region(_) => {
                        return Err(GeopError::new(format!(
                            "the sketch {name:?} has more than one loop: a path or a rail is one chain of curves, or one loop"
                        )))
                        .with_context(ctx);
                    }
                };
                let pieces = lp.to_nurbs(sketch, &geometry).with_context(ctx)?;
                let plane = &placed.plane;
                let (curve_names, joint_names) = piece_names(
                    name,
                    &pieces
                        .iter()
                        .map(|p| (p.name(), p.start.to_string(), p.end.to_string()))
                        .collect::<Vec<_>>(),
                    closed,
                );
                Ok(Chain {
                    name: name.clone(),
                    curves: pieces
                        .iter()
                        .map(|p| p.curve.embed(plane.origin(), plane.u(), plane.v()))
                        .collect::<GeopResult<_>>()
                        .with_context(ctx)?,
                    curve_names,
                    joint_names,
                })
            }
            EntityRef::Sketch3d { name } => {
                let sketch = part.sketch3d(part.sketch3d_id(name).with_context(ctx)?)?;
                let chains = sketch.chains().with_context(ctx)?;
                let [chain] = chains.as_slice() else {
                    return Err(GeopError::new(format!(
                        "the 3-D sketch {name:?} has {} chains of curves: a path is one chain of curves, or one loop",
                        chains.len()
                    )))
                    .with_context(ctx);
                };
                let pieces = chain
                    .to_nurbs(sketch, &sketch.enclose::<S>().with_context(ctx)?)
                    .with_context(ctx)?;
                let (curve_names, joint_names) = piece_names(
                    name,
                    &pieces
                        .iter()
                        .map(|p| (p.name(), p.start.to_string(), p.end.to_string()))
                        .collect::<Vec<_>>(),
                    chain.closed,
                );
                Ok(Chain {
                    name: name.clone(),
                    curves: pieces.into_iter().map(|p| p.curve).collect(),
                    curve_names,
                    joint_names,
                })
            }
            other => Err(GeopError::new(format!(
                "{other} has no curves: pick an edge, a sketch's curve, or a sketch"
            ))),
        }
    }

    /// The one curve it refers to in `part` — an edge, of a face or of a
    /// wire, or a planar sketch's curve, in space — with names for it and
    /// its ends (see [`EntityRef::resolve_chain`]). Fails for anything
    /// else, and for a sketch of more than one curve.
    pub fn resolve_curve<S: Scalar>(&self, part: &Part<S>) -> GeopResult<NamedCurve<S>> {
        let ctx = with_context!("resolving the curve of {self}");
        let chain = self.resolve_chain(part).with_context(ctx)?;
        let [curve] = chain.curves.as_slice() else {
            return Err(GeopError::new(format!(
                "{self} is {} curves, not one",
                chain.curves.len()
            )))
            .with_context(ctx);
        };
        let joints = chain.joints().with_context(ctx)?;
        let (start, end) = (joints[0], *joints.last().expect("a chain has joints"));
        let (start_name, end_name) = (
            chain.joint_names[0].clone(),
            chain
                .joint_names
                .last()
                .expect("a chain has joints")
                .clone(),
        );
        Ok(NamedCurve {
            name: chain.curve_names[0].clone(),
            curve: curve.clone(),
            start: (start_name, start),
            end: (end_name, end),
        })
    }

    /// The body it refers to in `part`: a solid by its name, a sheet —
    /// faces standing on their own, which have no name as a whole — by one
    /// of its faces. A face of a solid stands for the whole solid. Fails
    /// for anything else, and for a body of a placed part, which is that
    /// part's own program's to change.
    pub fn resolve_body<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Body> {
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!(
                "{self} belongs to a placed part; change it in the part's own program"
            )));
        }
        match self {
            EntityRef::Solid { name } => Ok(Body::Solid(part.solid_id(name)?)),
            EntityRef::Face { name } => part.topology().body_of_face(part.face_id(name)?),
            other => Err(GeopError::new(format!(
                "{other} is no body: pick a solid or a face standing on its own"
            ))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A reference field's names come and go as the kind of entity they
    /// name: others picked into it are left out, and an empty name is none.
    #[test]
    fn names_make_references_of_one_kind_and_back() {
        let names = vec!["a".to_string(), String::new(), "b".to_string()];
        let faces = EntityRef::of_names(EntityRef::face, &names);
        assert_eq!(faces, [EntityRef::face("a"), EntityRef::face("b")]);
        let mixed = [faces[0].clone(), EntityRef::edge("e"), faces[1].clone()];
        assert_eq!(EntityRef::names_of(EntityRef::face, &mixed), ["a", "b"]);
        assert_eq!(EntityRef::names_of(EntityRef::edge, &mixed), ["e"]);
    }
}
