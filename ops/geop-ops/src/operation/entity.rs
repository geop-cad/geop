//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face, datum, solid or sketch of the part, by name — or of a part
//! placed in it, by its name there behind the instance's
//! ([`INSTANCE_SEPARATOR`]).

use super::Lookup;
use super::{Chain, piece_names};
use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
pub use geop_core_program::editing::{EntityRef, INSTANCE_SEPARATOR, instance_path};
use geop_core_sketch::Shape;
use geop_core_topology::Body;

/// A curve picked to build on (see [`ResolveEntity::resolve_curve`]): where it
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
/// projected into that plane (see [`across`]) — so frames with parallel
/// normals line up, and a frame put on an axis has `u` where a joint
/// measures its turns from: two bodies mated on the same axis of each are
/// at a turn of zero.
pub fn frame_along<S: Scalar>(
    origin: Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<CoordinateSystem<S>> {
    let n = normal.normalize()?;
    let [u, v] = n.across()?;
    CoordinateSystem::try_new(origin, u, v, n)
}

/// The datum `name` as `lookup` has it: [`ORIGIN`], the world's origin and
/// axes, which every part has, or one a step added.
fn datum<S: Scalar>(name: &str, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Datum<S>> {
    if name == ORIGIN {
        return Ok(Datum {
            kind: DatumKind::Frame,
            frame: CoordinateSystem::world_at(Vector3::zero()),
        });
    }
    lookup.datum(name).cloned()
}

/// The name of the frame datum every part has: the world's origin and
/// axes, and the three planes between them (see
/// [`geop_core_math::primitives::DatumComponent`]).
pub const ORIGIN: &str = "origin";

/// The plane of [`ORIGIN`] square to `normal`.
pub fn origin_plane(normal: FrameAxis) -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(normal))
}

/// The axis `axis` of [`ORIGIN`].
pub fn origin_axis(axis: FrameAxis) -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Axis(axis))
}

/// What an [`EntityRef`] is in a part: its datum, plane, chain, curve or
/// body, as `lookup` has it.
pub trait ResolveEntity {
    fn resolve_datum<S: Scalar>(&self, _: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Datum<S>>;
    fn resolve_plane<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<CoordinateSystem<S>>;
    fn resolve_chain<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Chain<NurbCurve3D<S>>>;
    fn resolve_curve<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<NamedCurve<S>>;
    fn resolve_body<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Body>;
}

impl ResolveEntity for EntityRef {
    /// The datum it refers to in `part` — for a component of a frame, that
    /// component as a datum of its own. Fails if it is no datum of the
    /// part.
    fn resolve_datum<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Datum<S>> {
        let ctx = with_context!("resolving {self}");
        // A placed part's datum: its own, where that part is.
        if let Some(placed) = lookup.in_placed(self) {
            let (mut inside, pose, local) = placed.with_context(ctx)?;
            let datum = local.resolve_datum(&mut *inside).with_context(ctx)?;
            let frame = pose.motion().apply_frame(&datum.frame).with_context(ctx)?;
            return Ok(Datum { frame, ..datum });
        }
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!("{self} is of no part placed here")));
        }
        let EntityRef::Datum { name, component } = self else {
            return Err(GeopError::new(format!("{self} is not a datum")));
        };
        let datum = datum(name, lookup).with_context(ctx)?;
        match component {
            Some(component) => datum.component(*component).with_context(ctx),
            None => Ok(datum),
        }
    }

    /// The plane it lies in, as a frame with `u`/`v` a sketch's `x`/`y` on
    /// it and `w = u x v` its normal: a planar face's — normal pointing out
    /// of its solid, origin the world origin's projection onto it and `u`
    /// the world axis most parallel to it, so sketches on parallel faces
    /// line up — a datum plane's, a frame's plane's, or a sketch's. Fails
    /// for anything else.
    fn resolve_plane<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<CoordinateSystem<S>> {
        let ctx = with_context!("resolving the plane of {self}");
        // A placed part's plane: its own, where that part is.
        if let Some(placed) = lookup.in_placed(self) {
            let (mut inside, pose, local) = placed.with_context(ctx)?;
            let plane = local.resolve_plane(&mut *inside).with_context(ctx)?;
            return pose.motion().apply_frame(&plane).with_context(ctx);
        }
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!("{self} is of no part placed here")));
        }
        match self {
            EntityRef::Face { name } => {
                let model = lookup.model();
                let id = model.face_id(name).with_context(ctx)?;
                let surface = &model.topology().get_face(id).with_context(ctx)?.surface;
                match surface.as_plane().with_context(ctx)? {
                    Some(plane) => frame_along(plane.project(&Vector3::zero()), &plane.normal),
                    None => Err(GeopError::new(format!("{self} is not planar"))),
                }
            }
            EntityRef::Datum { .. } => {
                let datum = self.resolve_datum(lookup)?;
                match datum.kind {
                    DatumKind::Plane => Ok(datum.frame),
                    _ => Err(GeopError::new(format!("{self} is not a plane"))),
                }
            }
            EntityRef::Sketch { name } => Ok(lookup.sketch(name).with_context(ctx)?.plane.clone()),
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
    fn resolve_chain<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Chain<NurbCurve3D<S>>> {
        let ctx = with_context!("resolving the curves of {self}");
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!(
                "{self} is of a placed part: a path runs along the part's own curves, not a placed part's"
            )));
        }
        match self {
            EntityRef::Edge { name } => {
                let named = lookup.model();
                let model = named.topology();
                let edge = model.get_edge(named.edge_id(name).with_context(ctx)?)?;
                let end = |v| {
                    named.name_of(v).map(str::to_string).ok_or_else(|| {
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
                let placed = lookup.sketch(sketch).with_context(ctx)?;
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
                let placed = lookup.sketch(name).with_context(ctx)?;
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
                let (curve_names, joint_names) = piece_names(name, &pieces, closed);
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
                let sketch = lookup.sketch3d(name).with_context(ctx)?;
                let chains = sketch.chains().with_context(ctx)?;
                let [chain] = chains.as_slice() else {
                    return Err(GeopError::new(format!(
                        "the 3-D sketch {name:?} has {} chains of curves: a path is one chain of curves, or one loop",
                        chains.len()
                    )))
                    .with_context(ctx);
                };
                let geometry = sketch.enclose::<S>().with_context(ctx)?;
                let pieces = chain.to_nurbs(sketch, &geometry).with_context(ctx)?;
                Ok(Chain::of_pieces(name, pieces, chain.closed))
            }
            other => Err(GeopError::new(format!(
                "{other} has no curves: pick an edge, a sketch's curve, a sketch, or a 3-D sketch"
            ))),
        }
    }

    /// The one curve it refers to in `part` — an edge, of a face or of a
    /// wire, or a planar sketch's curve, in space — with names for it and
    /// its ends (see [`EntityRef::resolve_chain`]). Fails for anything
    /// else, and for a sketch of more than one curve.
    fn resolve_curve<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<NamedCurve<S>> {
        let ctx = with_context!("resolving the curve of {self}");
        let chain = self.resolve_chain(lookup).with_context(ctx)?;
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
    fn resolve_body<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Body> {
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!(
                "{self} belongs to a placed part; change it in the part's own program"
            )));
        }
        match self {
            EntityRef::Solid { name } => Ok(Body::Solid(lookup.model().solid_id(name)?)),
            EntityRef::Face { name } => {
                let model = lookup.model();
                model.topology().body_of_face(model.face_id(name)?)
            }
            other => Err(GeopError::new(format!(
                "{other} is no body: pick a solid or a face standing on its own"
            ))),
        }
    }
}

/// The bodies `refs` refer to (see [`ResolveEntity::resolve_body`]), each
/// once, in the order picked. None picked is refused.
pub fn resolve_bodies<S: Scalar>(
    refs: &[EntityRef],
    lookup: &mut (impl Lookup<S> + ?Sized),
) -> GeopResult<Vec<Body>> {
    if refs.is_empty() {
        return Err(GeopError::new("pick the bodies first"));
    }
    let mut bodies = Vec::new();
    for entity in refs {
        let body = entity.resolve_body(lookup)?;
        if !bodies.contains(&body) {
            bodies.push(body);
        }
    }
    Ok(bodies)
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
