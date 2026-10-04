//! [`PartDescription`]: a part's topology in terms of names only.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
};
use geop_core_topology::{CoedgeGeometry, Sense, boundary::BoundaryType};
use serde::{Deserialize, Serialize};

use super::{Part, ids::RefId};
use crate::Design;

/// A face's boundary loops, each as the coedges it runs through: `+E` / `-E`
/// for edge `E` traversed forwards / backwards, `@V` for a degenerate
/// coedge sitting at vertex `V` (a pole).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceDescription {
    pub outer: Vec<String>,
    pub holes: Vec<Vec<String>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeDescription {
    pub start: String,
    pub end: String,
}

/// A part placed in another: the file it is built from, where it is, and
/// whether it stays put.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InstanceDescription {
    pub file: String,
    pub pose: Pose<Design>,
    pub fixed: bool,
}

/// Everything about a part's topology that its names can express, and the
/// positions of its vertices — with no internal id anywhere, so two parts
/// built by the same program describe identically however their ids came
/// out, and a description diffs well as text. Loops start at their smallest
/// entry and holes are sorted, since neither has a first element of its own.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartDescription {
    /// Every solid's shells, each as its sorted face names.
    pub solids: BTreeMap<String, Vec<Vec<String>>>,
    pub faces: BTreeMap<String, FaceDescription>,
    pub edges: BTreeMap<String, EdgeDescription>,
    pub vertices: BTreeMap<String, [f64; 3]>,
    pub sketches: Vec<String>,
    pub datums: Vec<String>,
    pub instances: BTreeMap<String, InstanceDescription>,
    pub mates: Vec<String>,
}

impl PartDescription {
    pub fn of<S: Scalar>(part: &Part<S>) -> GeopResult<Self> {
        let model = part.topology();
        let name = |id: RefId| -> GeopResult<String> {
            part.name_of(id)
                .map(str::to_string)
                .ok_or_else(|| GeopError::new(format!("PartDescription: {id} has no name")))
        };
        let describe_loop = |boundary: BoundaryType| -> GeopResult<Vec<String>> {
            let mut entries = match boundary {
                BoundaryType::Vertex(v) => vec![format!("@{}", name(v.into())?)],
                BoundaryType::Loop(anchor) => model
                    .iterate_loop_coedges(anchor)
                    .map(|c| {
                        let coedge = model.get_coedge(c)?;
                        Ok(match coedge.geometry {
                            CoedgeGeometry::Edge(e) => match coedge.sense {
                                Sense::Forward => format!("+{}", name(e.into())?),
                                Sense::Reversed => format!("-{}", name(e.into())?),
                            },
                            CoedgeGeometry::Vertex(v) => format!("@{}", name(v.into())?),
                        })
                    })
                    .collect::<GeopResult<Vec<_>>>()?,
            };
            if let Some(first) = (0..entries.len()).min_by_key(|&i| &entries[i]) {
                entries.rotate_left(first);
            }
            Ok(entries)
        };

        let mut solids = BTreeMap::new();
        for (&id, solid) in &model.solids {
            let shells = solid
                .shells
                .iter()
                .map(|&shell| {
                    let mut faces = model
                        .get_shell(shell)?
                        .faces
                        .iter()
                        .map(|&f| name(f.into()))
                        .collect::<GeopResult<Vec<_>>>()?;
                    faces.sort();
                    Ok(faces)
                })
                .collect::<GeopResult<Vec<_>>>()?;
            solids.insert(name(id.into())?, shells);
        }
        let mut faces = BTreeMap::new();
        for (&id, face) in &model.faces {
            let mut holes = face
                .holes
                .iter()
                .map(|&h| describe_loop(h))
                .collect::<GeopResult<Vec<_>>>()?;
            holes.sort();
            faces.insert(
                name(id.into())?,
                FaceDescription {
                    outer: describe_loop(face.outer)?,
                    holes,
                },
            );
        }
        let mut edges = BTreeMap::new();
        for (&id, edge) in &model.edges {
            edges.insert(
                name(id.into())?,
                EdgeDescription {
                    start: name(edge.start_vertex.into())?,
                    end: name(edge.end_vertex.into())?,
                },
            );
        }
        let mut vertices = BTreeMap::new();
        for (&id, vertex) in &model.vertices {
            let p = vertex.point;
            vertices.insert(
                name(id.into())?,
                [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()],
            );
        }
        let mut sketches = part
            .sketches()
            .map(|(id, _)| name(id.into()))
            .chain(part.sketches3d().map(|(id, _)| name(id.into())))
            .collect::<GeopResult<Vec<_>>>()?;
        sketches.sort();
        let mut datums = part
            .datums()
            .map(|(id, _)| name(id.into()))
            .collect::<GeopResult<Vec<_>>>()?;
        datums.sort();
        let instances = part
            .instances()
            .map(|(id, instance)| {
                let description = InstanceDescription {
                    file: instance.component.file.clone(),
                    pose: instance.pose.cast(),
                    fixed: instance.fixed,
                };
                Ok((name(id.into())?, description))
            })
            .collect::<GeopResult<_>>()?;
        let mates = part.mates().map(|(name, _)| name.to_string()).collect();
        Ok(Self {
            solids,
            faces,
            edges,
            vertices,
            sketches,
            datums,
            instances,
            mates,
        })
    }
}
