use core::fmt::{self, Display};
use std::collections::HashMap;
use std::hash::Hash;

use geop_core_math::scalars::Scalar;

use super::Model;

/// `map`'s entries sorted by id (`.0`, the stable numeric part every
/// `*Id` newtype wraps) — a `HashMap`'s own iteration order is arbitrary,
/// which would otherwise make `Model`'s `Display` output nondeterministic
/// from run to run.
fn sorted_by_id<Id: Copy + Eq + Hash, V>(map: &HashMap<Id, V>) -> Vec<(Id, &V)>
where
    Id: Into<u64>,
{
    let mut entries: Vec<(Id, &V)> = map.iter().map(|(&id, v)| (id, v)).collect();
    entries.sort_by_key(|(id, _)| (*id).into());
    entries
}

impl<S: Scalar> Display for Model<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Model {{")?;

        writeln!(f, "  vertices:")?;
        for (id, vertex) in sorted_by_id(&self.vertices) {
            writeln!(f, "    {id}: point={}", vertex.point)?;
        }

        writeln!(f, "  edges:")?;
        for (id, edge) in sorted_by_id(&self.edges) {
            writeln!(
                f,
                "    {id}: {} -> {}, curve={}",
                edge.start_vertex, edge.end_vertex, edge.curve
            )?;
        }

        writeln!(f, "  coedges:")?;
        for (id, coedge) in sorted_by_id(&self.coedges) {
            writeln!(
                f,
                "    {id}: geometry={:?}, sense={:?}, next={}, prev={}, face={}, pcurve={}",
                coedge.geometry, coedge.sense, coedge.next, coedge.prev, coedge.face, coedge.pcurve
            )?;
        }

        writeln!(f, "  faces:")?;
        for (id, face) in sorted_by_id(&self.faces) {
            writeln!(
                f,
                "    {id}: shell={}, outer={:?}, holes={:?}, surface={}",
                face.shell, face.outer, face.holes, face.surface
            )?;
        }

        writeln!(f, "  shells:")?;
        for (id, shell) in sorted_by_id(&self.shells) {
            writeln!(
                f,
                "    {id}: solid={}, faces={:?}",
                shell.solid, shell.faces
            )?;
        }

        writeln!(f, "  solids:")?;
        for (id, solid) in sorted_by_id(&self.solids) {
            writeln!(f, "    {id}: shells={:?}", solid.shells)?;
        }

        write!(f, "}}")
    }
}
