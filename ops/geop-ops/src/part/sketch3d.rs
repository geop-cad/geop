//! A [`Part`]'s 3-D sketches: points and curves in space, named like any
//! other entity — the paths sweeps run along, and the rails that guide
//! them — and built as a wire (see [`geop_core_topology::Wire`]) whose
//! edges and vertices are named, picked and used as any other.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::nurb_curve::NurbCurve;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::{PointId, space::Sketch3d};
use geop_core_topology::{
    WireId,
    build::{BodySpec, EdgeSpec},
};

use super::ids::Sketch3dId;
use super::{BodyNames, Namer, Part};
use crate::Design;

/// A 3-D sketch as a part holds it: the sketch, and the wire it is built
/// as — none for a sketch with nothing in it.
#[derive(Clone, Debug)]
pub(crate) struct PartSketch3d {
    pub sketch: Sketch3d<Design>,
    pub wire: Option<WireId>,
}

impl<S: Scalar> Part<S> {
    /// Adds `sketch` to the part under `name` — solved, with the geometry
    /// of its references handed in (see [`Sketch3d::references`]) — and
    /// builds it as a wire: an edge for every curve drawn, construction
    /// curves aside, and a vertex for every point a curve of it ends at or
    /// that stands on its own. Points made one by a coincidence are one
    /// vertex, named after the lowest of their ids. In the sketch `K`, the
    /// edge of curve `c3` is named `sketch3d(K,c3)`, the vertex of point
    /// `p1` `sketch3d(K,p1)`.
    ///
    /// Fails, leaving the part unchanged, if `name` or one of those is
    /// already taken.
    pub fn add_sketch3d(
        &mut self,
        sketch: Sketch3d<Design>,
        name: impl Into<String>,
    ) -> GeopResult<Sketch3dId> {
        let name = name.into();
        let ctx = with_context!("Part::add_sketch3d({name})");
        if self.names.id_of(&name).is_some() {
            return Err(GeopError::new(format!("the name {name:?} is taken"))).with_context(ctx);
        }
        let (spec, names) = wire_of(&sketch, &Namer::new("sketch3d", &name)?).with_context(ctx)?;
        let wire = if spec.vertices.is_empty() {
            None
        } else {
            self.build_body(spec, names).with_context(ctx)?.wire
        };
        let id = Sketch3dId(self.fresh_id());
        self.names.insert(id, name)?;
        self.sketches3d.insert(id, PartSketch3d { sketch, wire });
        Ok(id)
    }

    pub fn sketch3d(&self, id: Sketch3dId) -> GeopResult<&Sketch3d<Design>> {
        self.sketches3d
            .get(&id)
            .map(|s| &s.sketch)
            .ok_or_else(|| GeopError::new(format!("Part has no 3-D sketch {id}")))
    }

    /// The wire the 3-D sketch `id` is built as — none for an empty one.
    pub fn sketch3d_wire(&self, id: Sketch3dId) -> GeopResult<Option<WireId>> {
        self.sketches3d
            .get(&id)
            .map(|s| s.wire)
            .ok_or_else(|| GeopError::new(format!("Part has no 3-D sketch {id}")))
    }

    /// Every 3-D sketch, in the order they were added.
    pub fn sketches3d(&self) -> impl Iterator<Item = (Sketch3dId, &Sketch3d<Design>)> {
        self.sketches3d.iter().map(|(&id, s)| (id, &s.sketch))
    }
}

/// The wire `sketch` is built as, and the names of its vertices and edges
/// (see [`Part::add_sketch3d`]).
fn wire_of<S: Scalar>(
    sketch: &Sketch3d<Design>,
    namer: &Namer,
) -> GeopResult<(BodySpec<S>, BodyNames)> {
    let class = sketch.point_classes();
    let geometry = sketch.enclose::<S>()?;
    let drawn: Vec<_> = sketch
        .curves
        .iter()
        .filter(|(_, c)| c.is_drawn() && !c.construction)
        .collect();
    let used = |p: &PointId| {
        sketch
            .curves
            .values()
            .any(|c| c.points().iter().any(|q| class[q] == class[p]))
    };
    let ends = drawn.iter().flat_map(|(_, c)| {
        let (start, end) = c.endpoints().expect("a drawn curve has ends");
        [start, end]
    });
    let alone = sketch.points.keys().copied().filter(|p| !used(p));
    let points: BTreeSet<PointId> = ends.chain(alone).map(|p| class[&p]).collect();
    // A vertex encloses every point of its class.
    let vertices: BTreeMap<PointId, Vector3<S>> = points
        .into_iter()
        .map(|rep| {
            let at = sketch
                .points
                .keys()
                .filter(|p| class[*p] == rep)
                .map(|p| geometry.points[p])
                .reduce(|a, b| a.union(&b))
                .expect("a class has its representative");
            (rep, at)
        })
        .collect();
    let index: BTreeMap<PointId, usize> =
        vertices.keys().enumerate().map(|(i, &p)| (p, i)).collect();
    let edges = drawn
        .iter()
        .map(|&(&id, curve)| {
            let (start, end) = curve.endpoints().expect("a drawn curve has ends");
            Ok(EdgeSpec {
                curve: NurbCurve::join(&sketch.curve_nurbs(id, &geometry)?)?,
                start: index[&class[&start]],
                end: index[&class[&end]],
            })
        })
        .collect::<GeopResult<_>>()?;
    let names = BodyNames {
        vertices: vertices
            .keys()
            .map(|p| namer.name(&[&p.to_string()]))
            .collect(),
        edges: drawn
            .iter()
            .map(|(id, _)| namer.name(&[&id.to_string()]))
            .collect(),
        faces: Vec::new(),
        solid: None,
    };
    let spec = BodySpec {
        vertices: vertices.into_values().collect(),
        edges,
        faces: Vec::new(),
        shells: Vec::new(),
        solid: false,
    };
    Ok((spec, names))
}
