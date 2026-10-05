use core::fmt::{self, Display};

use geop_core_math::scalars::Scalar;

use super::Model;

impl<S: Scalar> Display for Model<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Model {{")?;

        writeln!(f, "  vertices:")?;
        for (id, vertex) in &self.vertices {
            writeln!(f, "    {id}: point={}", vertex.point)?;
        }

        writeln!(f, "  edges:")?;
        for (id, edge) in &self.edges {
            writeln!(
                f,
                "    {id}: {} -> {}, curve={}",
                edge.start_vertex, edge.end_vertex, edge.curve
            )?;
        }

        writeln!(f, "  coedges:")?;
        for (id, coedge) in &self.coedges {
            writeln!(
                f,
                "    {id}: geometry={:?}, sense={:?}, next={}, prev={}, face={}, pcurve={}",
                coedge.geometry, coedge.sense, coedge.next, coedge.prev, coedge.face, coedge.pcurve
            )?;
        }

        writeln!(f, "  faces:")?;
        for (id, face) in &self.faces {
            writeln!(
                f,
                "    {id}: shell={}, outer={:?}, holes={:?}, surface={}",
                face.shell, face.outer, face.holes, face.surface
            )?;
        }

        writeln!(f, "  shells:")?;
        for (id, shell) in &self.shells {
            writeln!(
                f,
                "    {id}: solid={:?}, faces={:?}",
                shell.solid, shell.faces
            )?;
        }

        writeln!(f, "  solids:")?;
        for (id, solid) in &self.solids {
            writeln!(f, "    {id}: shells={:?}", solid.shells)?;
        }

        writeln!(f, "  wires:")?;
        for (id, wire) in &self.wires {
            writeln!(
                f,
                "    {id}: vertices={:?}, edges={:?}",
                wire.vertices, wire.edges
            )?;
        }

        write!(f, "}}")
    }
}
