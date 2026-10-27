//! How a [`NamedModel`] is kept between processes (see
//! [`geop_core_program::kept`]): written exactly, every entity as the
//! topology holds it, its id with it, so that the model read back is the
//! model written — equal to it, and building on it gives the same ids.
//!
//! Each revision of a part's model shares most of its entities with the
//! revision before, so the entities are put in the pool, which writes a
//! value the records of a file share once: the faces, edges and coedges —
//! which carry the surfaces and curves — each by itself, the small entities
//! and the names in runs of ids ([`RUN`]), so that an entity changed or
//! added changes only its own run.

use std::sync::Arc;

use geop_core_math::geop_error::GeopResult;
use geop_core_program::kept::Pool;
use geop_core_topology::{
    Coedge, CoedgeId, Edge, EdgeId, Face, FaceId, Model, Shell, ShellId, Solid, SolidId, Vertex,
    VertexId, Wire, WireId,
};
use imbl::OrdMap;
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::{Feature, NamedModel, RefId, names::NameRegistry};

/// How many ids of the small entities, and of the names, are put in the
/// pool as one value.
const RUN: u64 = 32;

/// A [`NamedModel`] as it is written: each kind of entity as the pool keys
/// of its values ([`Pool::put`]), its next id, the names in runs and the
/// aliases, and each feature by the step that made it.
#[derive(Serialize, Deserialize)]
struct Written {
    vertices: Vec<String>,
    edges: Vec<String>,
    coedges: Vec<String>,
    faces: Vec<String>,
    shells: Vec<String>,
    solids: Vec<String>,
    wires: Vec<String>,
    next_id: u64,
    names: Vec<String>,
    aliases: String,
    features: Vec<(String, String)>,
}

/// `model` written, its parts put in `pool`.
pub(crate) fn write(model: &NamedModel, pool: &mut Pool) -> GeopResult<String> {
    let topology = model.topology();
    let (names, aliases) = model.names().entries();
    let written = Written {
        vertices: runs(pool, &topology.vertices, |id| id.0)?,
        edges: each(pool, &topology.edges)?,
        coedges: each(pool, &topology.coedges)?,
        faces: each(pool, &topology.faces)?,
        shells: runs(pool, &topology.shells, |id| id.0)?,
        solids: runs(pool, &topology.solids, |id| id.0)?,
        wires: runs(pool, &topology.wires, |id| id.0)?,
        next_id: topology.next_id(),
        names: runs(pool, names.iter().map(|(id, name)| (id, name)), ref_id)?,
        aliases: pool.put(&aliases)?,
        features: (model.features.iter())
            .map(|(step, feature)| Ok((step.clone(), pool.put(&**feature)?)))
            .collect::<GeopResult<_>>()?,
    };
    serde_json::to_string(&written).map_err(|e| {
        geop_core_math::geop_error::GeopError::new(format!("writing a kept model: {e}"))
    })
}

/// The model `text` writes, its parts taken from `pool`.
pub(crate) fn read(text: &str, pool: &Pool) -> GeopResult<NamedModel> {
    let written: Written = serde_json::from_str(text).map_err(|e| {
        geop_core_math::geop_error::GeopError::new(format!("reading a kept model: {e}"))
    })?;
    let mut topology = Model::new().with_next_id(written.next_id);
    topology.vertices = read_all::<VertexId, Vertex, _>(pool, &written.vertices)?;
    topology.edges = read_all::<EdgeId, Edge, _>(pool, &written.edges)?;
    topology.coedges = read_all::<CoedgeId, Coedge, _>(pool, &written.coedges)?;
    topology.faces = read_all::<FaceId, Face, _>(pool, &written.faces)?;
    topology.shells = read_all::<ShellId, Shell, _>(pool, &written.shells)?;
    topology.solids = read_all::<SolidId, Solid, _>(pool, &written.solids)?;
    topology.wires = read_all::<WireId, Wire, _>(pool, &written.wires)?;
    let names: Vec<(RefId, String)> = read_all(pool, &written.names)?;
    let aliases: Vec<(String, String)> = pool.get(&written.aliases)?;
    let features = (written.features.iter())
        .map(|(step, key)| Ok((step.clone(), Arc::new(pool.get::<Feature>(key)?))))
        .collect::<GeopResult<_>>()?;
    Ok(NamedModel {
        topology,
        names: NameRegistry::from_entries(names, aliases),
        features,
    })
}

/// A name's id as a number: by the kind of entity, then its id — what the
/// names are sorted and run by.
pub(crate) fn ref_id(id: &RefId) -> u64 {
    let (kind, n) = match id {
        RefId::Vertex(id) => (0, id.0),
        RefId::Edge(id) => (1, id.0),
        RefId::Face(id) => (2, id.0),
        RefId::Solid(id) => (3, id.0),
    };
    // Runs of one kind only: ids are well below 2^60.
    (kind << 60) | n
}

/// Each of `entities` put by itself, with its id.
fn each<K: Ord + Clone + Serialize, V: Clone + Serialize>(
    pool: &mut Pool,
    entities: &OrdMap<K, V>,
) -> GeopResult<Vec<String>> {
    (entities.iter())
        .map(|(id, entity)| pool.put(&[(id, entity)]))
        .collect()
}

/// `entities`, in runs of [`RUN`] ids by `number`, each run put as one.
fn runs<'e, K: Serialize + 'e, V: Serialize + 'e>(
    pool: &mut Pool,
    entities: impl IntoIterator<Item = (&'e K, &'e V)>,
    number: impl Fn(&K) -> u64,
) -> GeopResult<Vec<String>> {
    let mut keys = Vec::new();
    let mut run: Vec<(&K, &V)> = Vec::new();
    let mut at = None;
    for (id, entity) in entities {
        let this = number(id) / RUN;
        if at.is_some_and(|at| at != this) {
            keys.push(pool.put(&run)?);
            run.clear();
        }
        at = Some(this);
        run.push((id, entity));
    }
    if !run.is_empty() {
        keys.push(pool.put(&run)?);
    }
    Ok(keys)
}

/// The entities the runs `keys` hold, each with its id.
fn read_all<K: DeserializeOwned, V: DeserializeOwned, M>(
    pool: &Pool,
    keys: &[String],
) -> GeopResult<M>
where
    M: FromIterator<(K, V)>,
{
    let mut all = Vec::new();
    for key in keys {
        all.extend(pool.get::<Vec<(K, V)>>(key)?);
    }
    Ok(all.into_iter().collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::unit_cube;

    /// A model read back is the model written, to the bit — and building on
    /// it gives the ids building on the original gives; a second revision
    /// that changed nothing adds nothing to the pool.
    #[test]
    fn a_kept_model_reads_back_as_it_was() {
        let cube = unit_cube();
        let mut pool = Pool::default();
        let text = write(&cube, &mut pool).unwrap();
        let read = read(&text, &pool).unwrap();
        assert!(read == cube);
        assert_eq!(read.topology().next_id(), cube.topology().next_id());
        let kept = pool.len();
        write(&cube.clone(), &mut pool).unwrap();
        assert_eq!(pool.len(), kept);
    }
}
