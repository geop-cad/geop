use imbl::OrdMap;

use geop_core_geometry::nurb_curve::NurbCurve;

use super::{
    Coedge, CoedgeId, Edge, EdgeId, Face, FaceId, Shell, ShellId, Solid, SolidId, Vertex, VertexId,
    Wire, WireId,
};

mod create;
mod display;
mod get;
mod iterate;

/// A 2-D parameter-space NURBS curve (pcurve).
///
/// Control points are in homogeneous parameter space: `(wu, wv, w)`.
pub type Curve2 = NurbCurve<3>;

/// A 3-D NURBS curve used as edge geometry.
///
/// Control points are in homogeneous 3-D space: `(wx, wy, wz, w)`.
pub type Curve3 = NurbCurve<4>;

/// The top-level boundary-representation model.
///
/// `Vertex`, `Edge`, `Coedge`, `Face`, `Shell` and `Solid` are the entities
/// genuinely shared by reference (an edge by its two coedges via `opposite`,
/// a vertex by every incident edge, and so on) — these live here in arenas
/// and are referenced by stable typed IDs.  Their geometry (a `Curve3` per
/// `Edge`, a `Curve2` pcurve per `Coedge`, a `NurbSurface` per `Face`) is
/// owned 1:1 by that entity directly rather than through another arena, since
/// nothing ever needs to reference it independently.  A loop has no entity of
/// its own: each entry of `Face::boundaries` just anchors a `CoedgeId` whose
/// `next`/`prev` cycle traces the whole loop.
///
/// Hierarchy (each arrow means "references one or more"):
/// ```text
/// Solid ──▶ Shell(s) ──▶ Face(s) [+Surface] ──▶ Coedge(s) [+Curve2] ──▶ Edge [+Curve3] ──▶ Vertex
/// Wire ──▶ Edge(s) [+Curve3], Vertex(es)
/// ```
/// A [`Wire`] holds edges and vertices that bound no face: curves in space.
/// V - E + F - L = 2 * (Scalar - G), E = 2C for a single-shell solid with genus G,
/// where V = #vertices, E = #edges, F = #faces, L = #hole loops (i.e.
/// `Σ (face.boundaries.len() - 1)`, not counting each face's mandatory outer loop),
/// Scalar = #shells, and G = genus.  Euler's formula generalizes to multiple
/// shells and/or genus > 0.
///
/// Creation (`insert_*`), lookup (`get_*`), and iteration (`iterate_*`)
/// methods each live in their own private submodule (`model::create`,
/// `model::get`, `model::iterate`) — all still just plain inherent `Model`
/// methods from the outside.
///
/// The arenas are ordered by id, which is the order of creation: every walk
/// over a model's entities — validation, a boolean's classification, the
/// remesh's pairing of faces — sees them in the same order on every run.
/// A `HashMap` iterates in an order seeded afresh in each process, and a
/// validation failure that depended on which face pair was checked first
/// appeared and disappeared between runs.
#[derive(Clone, Debug, PartialEq)]
pub struct Model {
    pub vertices: OrdMap<VertexId, Vertex>, // V
    pub edges: OrdMap<EdgeId, Edge>,        // E
    pub coedges: OrdMap<CoedgeId, Coedge>,  // C
    pub faces: OrdMap<FaceId, Face>,        // F
    pub shells: OrdMap<ShellId, Shell>,     // Scalar
    pub solids: OrdMap<SolidId, Solid>,
    /// Edges and vertices bounding no face (see [`Wire`]).
    pub wires: OrdMap<WireId, Wire>,

    next_id: u64,
}

impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}

impl Model {
    pub fn new() -> Self {
        Self {
            vertices: OrdMap::new(),
            edges: OrdMap::new(),
            coedges: OrdMap::new(),
            faces: OrdMap::new(),
            shells: OrdMap::new(),
            solids: OrdMap::new(),
            wires: OrdMap::new(),
            next_id: 1,
        }
    }

    /// The id the next entity it creates gets: what a model written to be
    /// read back as it was keeps with its entities ([`Model::with_next_id`]).
    pub fn next_id(&self) -> u64 {
        self.next_id
    }

    /// It, creating entities from the id `next_id` on: a model read back as
    /// it was written, its entities set as they were. `next_id` is past
    /// every id it holds, as [`Model::next_id`] gave it.
    pub fn with_next_id(mut self, next_id: u64) -> Self {
        self.next_id = next_id;
        self
    }

    fn fresh_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
}
