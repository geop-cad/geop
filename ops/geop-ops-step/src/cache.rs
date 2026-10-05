//! An import's bodies kept between builds (see [`geop_ops::Library::cache`]):
//! reading a STEP file of hundreds of faces — parsing, converting, fitting
//! pcurves, healing — takes seconds; reading back what it came to takes
//! milliseconds.
//!
//! The bodies are kept as they are, every scalar by its two bounds, so what
//! is read back is the enclosure that was kept, not a rounding of it. The
//! key names everything they were worked out from: the file's text, the
//! scalar type, and [`VERSION`], which a change to how bodies are read must
//! raise.

use geop_core_geometry::{nurb_curve::NurbCurve, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};
use geop_core_topology::{
    Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use serde::{Deserialize, Serialize};

use crate::import::{Healing, ImportedBody};

/// Raised whenever reading a STEP file comes to different bodies than it
/// did, so bodies kept by an earlier reader are not taken for its own.
pub const VERSION: u32 = 2;

/// The key the bodies of the STEP file `text` are kept under, for scalars
/// `S`: the reader's version, the scalar type and a hash of the text (64-bit
/// FNV-1a, stable across processes and builds, unlike the standard hasher)
/// with the text's length.
pub fn key<S: Scalar>(text: &str) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in text.bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0100_0000_01b3);
    }
    let scalar = std::any::type_name::<S>()
        .rsplit("::")
        .next()
        .unwrap_or("scalar");
    format!("step-v{VERSION}-{scalar}-{hash:016x}-{}", text.len())
}

/// The bodies as bytes, to keep.
pub fn encode<S: Scalar>(bodies: &[ImportedBody<S>]) -> GeopResult<Vec<u8>> {
    let kept: Vec<KeptBody> = bodies.iter().map(KeptBody::of).collect();
    postcard::to_allocvec(&kept).map_err(|e| GeopError::new(format!("keeping the bodies: {e}")))
}

/// The bodies `bytes` keep (see [`encode`]); an error if they are not what
/// [`encode`] wrote.
pub fn decode<S: Scalar>(bytes: &[u8]) -> GeopResult<Vec<ImportedBody<S>>> {
    let kept: Vec<KeptBody> = postcard::from_bytes(bytes)
        .map_err(|e| GeopError::new(format!("reading kept bodies: {e}")))?;
    kept.into_iter().map(KeptBody::body).collect()
}

/// A scalar by its two bounds.
type Bounds = [f64; 2];

fn bounds<S: Scalar>(s: S) -> Bounds {
    [s.lower().to_f64(), s.upper().to_f64()]
}

fn scalar<S: Scalar>([lo, hi]: Bounds) -> S {
    S::from_f64(lo).union(S::from_f64(hi))
}

#[derive(Serialize, Deserialize)]
struct KeptCurve {
    degree: usize,
    control_points: Vec<Vec<Bounds>>,
    knots: Vec<Bounds>,
}

impl KeptCurve {
    fn of<S: Scalar, const D: usize>(curve: &NurbCurve<S, D>) -> Self {
        Self {
            degree: curve.degree,
            control_points: curve
                .control_points
                .iter()
                .map(|p| (0..D).map(|k| bounds(p[k])).collect())
                .collect(),
            knots: curve.knot_vector.iter().map(|&k| bounds(k)).collect(),
        }
    }

    fn curve<S: Scalar, const D: usize>(self) -> GeopResult<NurbCurve<S, D>> {
        NurbCurve::try_new(
            self.degree,
            points(self.control_points)?,
            self.knots.into_iter().map(scalar).collect(),
        )
    }
}

fn points<S: Scalar, const D: usize>(points: Vec<Vec<Bounds>>) -> GeopResult<Vec<Vector<S, D>>> {
    points
        .into_iter()
        .map(|p| {
            let p: [Bounds; D] = p
                .try_into()
                .map_err(|_| GeopError::new("reading kept bodies: a point of other dimension"))?;
            Ok(Vector::from_array(p.map(scalar)))
        })
        .collect()
}

#[derive(Serialize, Deserialize)]
struct KeptSurface {
    degree_u: usize,
    degree_v: usize,
    control_points: Vec<Vec<Bounds>>,
    knots_u: Vec<Bounds>,
    knots_v: Vec<Bounds>,
}

impl KeptSurface {
    fn of<S: Scalar>(surface: &NurbSurface3D<S>) -> Self {
        Self {
            degree_u: surface.degree_u,
            degree_v: surface.degree_v,
            control_points: surface
                .control_points
                .iter()
                .map(|p| (0..4).map(|k| bounds(p[k])).collect())
                .collect(),
            knots_u: surface.knot_vector_u.iter().map(|&k| bounds(k)).collect(),
            knots_v: surface.knot_vector_v.iter().map(|&k| bounds(k)).collect(),
        }
    }

    fn surface<S: Scalar>(self) -> GeopResult<NurbSurface3D<S>> {
        NurbSurface3D::try_new(
            self.degree_u,
            self.degree_v,
            points(self.control_points)?,
            self.knots_u.into_iter().map(scalar).collect(),
            self.knots_v.into_iter().map(scalar).collect(),
        )
    }
}

/// What a coedge runs along: an edge forward or reversed, or a vertex.
#[derive(Serialize, Deserialize)]
enum KeptOn {
    Forward(usize),
    Reversed(usize),
    Vertex(usize),
}

#[derive(Serialize, Deserialize)]
struct KeptCoedge {
    on: KeptOn,
    pcurve: KeptCurve,
}

impl KeptCoedge {
    fn of<S: Scalar>(coedge: &CoedgeSpec<S>) -> Self {
        Self {
            on: match coedge.on {
                CoedgeOn::Edge(e, Sense::Forward) => KeptOn::Forward(e),
                CoedgeOn::Edge(e, Sense::Reversed) => KeptOn::Reversed(e),
                CoedgeOn::Vertex(v) => KeptOn::Vertex(v),
            },
            pcurve: KeptCurve::of(&coedge.pcurve),
        }
    }

    fn coedge<S: Scalar>(self) -> GeopResult<CoedgeSpec<S>> {
        Ok(CoedgeSpec {
            on: match self.on {
                KeptOn::Forward(e) => CoedgeOn::Edge(e, Sense::Forward),
                KeptOn::Reversed(e) => CoedgeOn::Edge(e, Sense::Reversed),
                KeptOn::Vertex(v) => CoedgeOn::Vertex(v),
            },
            pcurve: self.pcurve.curve()?,
        })
    }
}

fn kept_loop<S: Scalar>(coedges: &[CoedgeSpec<S>]) -> Vec<KeptCoedge> {
    coedges.iter().map(KeptCoedge::of).collect()
}

fn read_loop<S: Scalar>(coedges: Vec<KeptCoedge>) -> GeopResult<Vec<CoedgeSpec<S>>> {
    coedges.into_iter().map(KeptCoedge::coedge).collect()
}

#[derive(Serialize, Deserialize)]
struct KeptFace {
    surface: KeptSurface,
    outer: Vec<KeptCoedge>,
    holes: Vec<Vec<KeptCoedge>>,
}

#[derive(Serialize, Deserialize)]
struct KeptEdge {
    curve: KeptCurve,
    start: usize,
    end: usize,
}

#[derive(Serialize, Deserialize)]
struct KeptBody {
    label: String,
    vertices: Vec<[Bounds; 3]>,
    edges: Vec<KeptEdge>,
    faces: Vec<KeptFace>,
    shells: Vec<Vec<usize>>,
    solid: bool,
    vertex_names: Vec<Vec<String>>,
    edge_names: Vec<Vec<String>>,
    face_names: Vec<Vec<String>>,
    healed_vertices: Vec<(String, f64)>,
    healed_edges: Vec<(String, f64)>,
    healed_faces: Vec<String>,
    uncertainty: f64,
}

impl KeptBody {
    fn of<S: Scalar>(body: &ImportedBody<S>) -> Self {
        let spec = &body.spec;
        Self {
            label: body.label.clone(),
            vertices: spec
                .vertices
                .iter()
                .map(|v| [0, 1, 2].map(|k| bounds(v[k])))
                .collect(),
            edges: spec
                .edges
                .iter()
                .map(|e| KeptEdge {
                    curve: KeptCurve::of(&e.curve),
                    start: e.start,
                    end: e.end,
                })
                .collect(),
            faces: spec
                .faces
                .iter()
                .map(|f| KeptFace {
                    surface: KeptSurface::of(&f.surface),
                    outer: kept_loop(&f.outer),
                    holes: f.holes.iter().map(|h| kept_loop(h)).collect(),
                })
                .collect(),
            shells: spec.shells.clone(),
            solid: spec.solid,
            vertex_names: body.vertex_names.clone(),
            edge_names: body.edge_names.clone(),
            face_names: body.face_names.clone(),
            healed_vertices: body.healed.vertices.clone(),
            healed_edges: body.healed.edges.clone(),
            healed_faces: body.healed.faces.clone(),
            uncertainty: body.healed.uncertainty,
        }
    }

    fn body<S: Scalar>(self) -> GeopResult<ImportedBody<S>> {
        let spec = BodySpec {
            vertices: self
                .vertices
                .into_iter()
                .map(|v| Vector::from_array(v.map(scalar)))
                .collect(),
            edges: self
                .edges
                .into_iter()
                .map(|e| {
                    Ok(EdgeSpec {
                        curve: e.curve.curve()?,
                        start: e.start,
                        end: e.end,
                    })
                })
                .collect::<GeopResult<_>>()?,
            faces: self
                .faces
                .into_iter()
                .map(|f| {
                    Ok(FaceSpec {
                        surface: f.surface.surface()?,
                        outer: read_loop(f.outer)?,
                        holes: f
                            .holes
                            .into_iter()
                            .map(read_loop)
                            .collect::<GeopResult<_>>()?,
                    })
                })
                .collect::<GeopResult<_>>()?,
            shells: self.shells,
            solid: self.solid,
        };
        Ok(ImportedBody {
            label: self.label,
            spec,
            vertex_names: self.vertex_names,
            edge_names: self.edge_names,
            face_names: self.face_names,
            healed: Healing {
                vertices: self.healed_vertices,
                edges: self.healed_edges,
                faces: self.healed_faces,
                uncertainty: self.uncertainty,
            },
        })
    }
}
