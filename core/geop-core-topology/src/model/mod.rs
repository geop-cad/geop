use std::collections::HashMap;

use geop_core_geometry::nurb_curve::NurbCurve;
use geop_core_math::scalars::Scalar;

use super::{
    Coedge, CoedgeId, Edge, EdgeId, Face, FaceId, Shell, ShellId, Solid, SolidId, Vertex, VertexId,
};

mod create;
mod display;
mod get;
mod iterate;

/// A 2-D parameter-space NURBS curve (pcurve).
///
/// Control points are in homogeneous parameter space: `(wu, wv, w)`.
pub type Curve2<S> = NurbCurve<S, 3>;

/// A 3-D NURBS curve used as edge geometry.
///
/// Control points are in homogeneous 3-D space: `(wx, wy, wz, w)`.
pub type Curve3<S> = NurbCurve<S, 4>;

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
/// ```
/// V - E + F - L = 2 * (S - G), E = 2C for a single-shell solid with genus G,
/// where V = #vertices, E = #edges, F = #faces, L = #hole loops (i.e.
/// `Σ (face.boundaries.len() - 1)`, not counting each face's mandatory outer loop),
/// S = #shells, and G = genus.  Euler's formula generalizes to multiple
/// shells and/or genus > 0.
///
/// Creation (`insert_*`), lookup (`get_*`), and iteration (`iterate_*`)
/// methods each live in their own private submodule (`model::create`,
/// `model::get`, `model::iterate`) — all still just plain inherent `Model`
/// methods from the outside.
#[derive(Clone)]
pub struct Model<S: Scalar> {
    pub vertices: HashMap<VertexId, Vertex<S>>, // V
    pub edges: HashMap<EdgeId, Edge<S>>,        // E
    pub coedges: HashMap<CoedgeId, Coedge<S>>,  // C
    pub faces: HashMap<FaceId, Face<S>>,        // F
    pub shells: HashMap<ShellId, Shell>,        // S
    pub solids: HashMap<SolidId, Solid>,

    next_id: u64,
}

impl<S: Scalar> Model<S> {
    pub fn new() -> Self {
        Self {
            vertices: HashMap::new(),
            edges: HashMap::new(),
            coedges: HashMap::new(),
            faces: HashMap::new(),
            shells: HashMap::new(),
            solids: HashMap::new(),
            next_id: 1,
        }
    }

    fn fresh_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
}
