//! The limit surface of a cage as a B-rep: a solid if the cage is closed,
//! a sheet if it has a boundary, described whole as a [`BodySpec`].
//!
//! The B-rep has the cage's own topology. Every quad of the cage is one
//! face: the four patches its quads became (see [`crate::limit`]) make one
//! bicubic B-spline of 7 x 7 control points over `[0, 2]²`, its inner knots
//! triple. Every edge of the cage is one edge, a cubic of two spans over
//! `[0, 2]`, and every vertex one vertex, at its limit position. A face of
//! any other number of sides cannot be one tensor-product surface, so it
//! stays as the patches it became, one face per corner, joined at its face
//! point by edges to the middle of each side — where its sides are split.
//!
//! Names follow the cage: for the step `S`, the face of cage face `f7` is
//! `subd(S,f7)`, the edge of cage edge `e3-5` `subd(S,e3-5)`, the vertex of
//! cage vertex `v3` `subd(S,v3)`, and the solid `subd(S)`. Of an n-gon
//! `f7`, the face at its corner `v3` is `subd(S,f7,v3)`, its centre
//! `subd(S,f7,centre)` and the edge from there to the middle of side
//! `e3-5` `subd(S,f7,e3-5)`; that side's halves are `subd(S,e3-5,v3)` and
//! `subd(S,e3-5,v5)`, meeting at `subd(S,e3-5,mid)`.

use std::collections::HashMap;

use geop_core_geometry::{
    nurb_curve::NurbCurve,
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};
use geop_core_topology::{
    Curve2, Curve3, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{BodyNames, Namer};

use crate::{
    cage::Mesh,
    limit::{ControlNet, control_net},
    subdivide::subdivide,
};

fn homogeneous<S: Scalar>(p: &Vector3<S>) -> Vector4<S> {
    Vector4::from_array([p[0], p[1], p[2], S::ONE])
}

/// The clamped cubic knots of `spans` Bézier spans over `[0, spans]`.
fn cubic_knots<S: Scalar>(spans: usize) -> Vec<S> {
    let mut knots = vec![S::ZERO; 4];
    for s in 1..spans {
        knots.extend([S::from_i64(s as i64); 3]);
    }
    knots.extend([S::from_i64(spans as i64); 4]);
    knots
}

/// The cubic through Bézier control points `points`, one span per three.
fn cubic<S: Scalar>(points: &[Vector3<S>]) -> GeopResult<Curve3<S>> {
    NurbCurve::try_new(
        3,
        points.iter().map(homogeneous).collect(),
        cubic_knots((points.len() - 1) / 3),
    )
}

/// The straight pcurve from `a` to `b`, over `[0, length]`.
fn line<S: Scalar>(a: [i64; 2], b: [i64; 2], length: i64) -> GeopResult<Curve2<S>> {
    let point = |p: [i64; 2]| {
        geop_core_math::vector::Vector3::from_array([
            S::from_i64(p[0]),
            S::from_i64(p[1]),
            S::ONE,
        ])
    };
    NurbCurve::try_new(
        1,
        vec![point(a), point(b)],
        vec![S::ZERO, S::ZERO, S::from_i64(length), S::from_i64(length)],
    )
}

/// The B-rep edges a cage edge became.
#[derive(Clone, Copy, Debug)]
enum EdgeParts {
    /// One edge, running from the cage edge's first vertex to its second.
    Whole(usize),
    /// Split at its middle: the half from the first vertex, and the half to
    /// the second.
    Split(usize, usize),
}

/// What [`limit_body`] builds the B-rep from.
struct Builder<'a, S: Scalar> {
    net: &'a ControlNet<S>,
    namer: &'a Namer,
    spec: BodySpec<S>,
    names: BodyNames,
    /// Per vertex of the subdivided mesh that is one of the B-rep's: its
    /// index there.
    vertex: HashMap<usize, usize>,
}

impl<S: Scalar> Builder<'_, S> {
    fn add_vertex(&mut self, sub_vertex: usize, name: &[&str]) {
        self.vertex.insert(sub_vertex, self.spec.vertices.len());
        self.spec.vertices.push(self.net.corner[sub_vertex]);
        self.names.vertices.push(self.namer.name(name));
    }

    /// The curve along the subdivided mesh's vertices `path`, from one
    /// B-rep vertex to another.
    fn add_edge(&mut self, path: &[usize], name: &[&str]) -> GeopResult<usize> {
        let mut points = vec![self.net.corner[path[0]]];
        for w in path.windows(2) {
            points.push(self.net.edge[&(w[0], w[1])]);
            points.push(self.net.edge[&(w[1], w[0])]);
            points.push(self.net.corner[w[1]]);
        }
        self.spec.edges.push(EdgeSpec {
            curve: cubic(&points)?,
            start: self.vertex[&path[0]],
            end: self.vertex[&path[path.len() - 1]],
        });
        self.names.edges.push(self.namer.name(name));
        Ok(self.spec.edges.len() - 1)
    }
}

/// The coedges of a face's side from cage vertex `from` to `to`, `parts`
/// the cage edge's B-rep edges, in `(u, v)` from `start` through `middle`
/// to `end`.
fn side<S: Scalar>(
    parts: EdgeParts,
    from: usize,
    to: usize,
    [start, middle, end]: [[i64; 2]; 3],
) -> GeopResult<Vec<CoedgeSpec<S>>> {
    let forward = from < to;
    let sense = if forward {
        Sense::Forward
    } else {
        Sense::Reversed
    };
    Ok(match parts {
        EdgeParts::Whole(e) => vec![CoedgeSpec {
            on: CoedgeOn::Edge(e, sense),
            pcurve: line(start, end, 2)?,
        }],
        EdgeParts::Split(first, second) => {
            let (near, far) = if forward {
                (first, second)
            } else {
                (second, first)
            };
            vec![
                CoedgeSpec {
                    on: CoedgeOn::Edge(near, sense),
                    pcurve: line(start, middle, 1)?,
                },
                CoedgeSpec {
                    on: CoedgeOn::Edge(far, sense),
                    pcurve: line(middle, end, 1)?,
                },
            ]
        }
    })
}

/// Checks that a closed mesh's faces run counter-clockwise seen from
/// outside: that it encloses a volume that is definitely positive.
fn check_outward<S: Scalar>(mesh: &Mesh) -> GeopResult<()> {
    let p = |v: usize| Vector3::from_array(mesh.positions[v].map(S::from_f64));
    let mut volume = S::ZERO;
    for face in &mesh.faces {
        for k in 1..face.len() - 1 {
            let (a, b, c) = (p(face[0]), p(face[k]), p(face[k + 1]));
            volume = volume.add(a.prod_dot(&b.prod_cross(&c)));
        }
    }
    if volume.definitely_greater(S::ZERO) {
        Ok(())
    } else {
        Err(GeopError::new(format!(
            "the cage encloses no volume, or is turned inside out — its faces must run counter-clockwise seen from outside (six times its volume: {volume:?})"
        )))
    }
}

/// The limit surface of `mesh` as a B-rep (see the module docs), named
/// with `namer`: a solid if the mesh is closed, else a sheet.
pub fn limit_body<S: Scalar>(mesh: &Mesh, namer: &Namer) -> GeopResult<(BodySpec<S>, BodyNames)> {
    let topology = mesh.topology()?;
    let edges = topology.edges();
    let closed = edges.values().all(|faces| faces.len() == 2);
    if closed {
        check_outward::<S>(mesh)?;
    }
    let sub = subdivide::<S>(mesh, &topology)?;
    let net = control_net(&sub)?;
    let mut b = Builder {
        net: &net,
        namer,
        spec: BodySpec {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            shells: Vec::new(),
            solid: closed,
        },
        names: BodyNames {
            solid: closed.then(|| namer.root()),
            ..BodyNames::default()
        },
        vertex: HashMap::new(),
    };

    for v in 0..mesh.positions.len() {
        b.add_vertex(v, &[&mesh.vertex_names[v]]);
    }
    let is_quad = |f: usize| mesh.faces[f].len() == 4;
    for (f, face) in mesh.faces.iter().enumerate() {
        if face.len() != 4 {
            b.add_vertex(sub.face_point[f], &[&mesh.face_names[f], "centre"]);
        }
    }
    let mut parts: HashMap<(usize, usize), EdgeParts> = HashMap::new();
    for (&(a, c), faces) in &edges {
        let name = mesh.edge_name(a, c);
        let m = sub.edge_point[&(a, c)];
        let part = if faces.iter().all(|&f| is_quad(f)) {
            EdgeParts::Whole(b.add_edge(&[a, m, c], &[&name])?)
        } else {
            b.add_vertex(m, &[&name, "mid"]);
            EdgeParts::Split(
                b.add_edge(&[a, m], &[&name, &mesh.vertex_names[a]])?,
                b.add_edge(&[m, c], &[&name, &mesh.vertex_names[c]])?,
            )
        };
        parts.insert((a, c), part);
    }
    let part = |a: usize, c: usize| parts[&(a.min(c), a.max(c))];

    for (f, face) in mesh.faces.iter().enumerate() {
        let n = face.len();
        if n == 4 {
            let mut grid = vec![Vector3::zero(); 49];
            for k in 0..4 {
                let patch = net.patch(&sub, sub.face_start[f] + k, 0);
                for (i, row) in patch.iter().enumerate() {
                    for (j, point) in row.iter().enumerate() {
                        let (u, v) = match k {
                            0 => (i, j),
                            1 => (6 - j, i),
                            2 => (6 - i, 6 - j),
                            _ => (j, 6 - i),
                        };
                        grid[u * 7 + v] = *point;
                    }
                }
            }
            let surface = NurbSurface::try_new(
                3,
                3,
                grid.iter().map(homogeneous).collect(),
                cubic_knots(2),
                cubic_knots(2),
            )?;
            let corners = [[0, 0], [2, 0], [2, 2], [0, 2]];
            let mut outer = Vec::new();
            for k in 0..4 {
                let (from, to) = (face[k], face[(k + 1) % 4]);
                let (s, e) = (corners[k], corners[(k + 1) % 4]);
                let middle = [(s[0] + e[0]) / 2, (s[1] + e[1]) / 2];
                outer.extend(side(part(from, to), from, to, [s, middle, e])?);
            }
            b.spec.faces.push(FaceSpec {
                surface,
                outer,
                holes: Vec::new(),
            });
            b.names.faces.push(namer.name(&[&mesh.face_names[f]]));
        } else {
            let centre = sub.face_point[f];
            let spokes = (0..n)
                .map(|k| {
                    let (a, c) = (face[k], face[(k + 1) % n]);
                    let m = sub.edge_point[&(a.min(c), a.max(c))];
                    b.add_edge(&[centre, m], &[&mesh.face_names[f], &mesh.edge_name(a, c)])
                })
                .collect::<GeopResult<Vec<_>>>()?;
            for k in 0..n {
                let (v, after, before) = (face[k], face[(k + 1) % n], face[(k + n - 1) % n]);
                let patch = net.patch(&sub, sub.face_start[f] + k, 0);
                let surface: NurbSurface3D<S> = NurbSurface::try_new(
                    3,
                    3,
                    patch.iter().flatten().map(homogeneous).collect(),
                    cubic_knots(1),
                    cubic_knots(1),
                )?;
                let mut outer = Vec::new();
                // From the corner to the middle of the side after it: the
                // first half of that side.
                let ahead = side(part(v, after), v, after, [[0, 0], [1, 0], [2, 0]])?;
                outer.push(ahead[0].clone());
                outer.push(CoedgeSpec {
                    on: CoedgeOn::Edge(spokes[k], Sense::Reversed),
                    pcurve: line([1, 0], [1, 1], 1)?,
                });
                outer.push(CoedgeSpec {
                    on: CoedgeOn::Edge(spokes[(k + n - 1) % n], Sense::Forward),
                    pcurve: line([1, 1], [0, 1], 1)?,
                });
                // From the middle of the side before it back to the corner:
                // the second half of that side.
                let behind = side(part(before, v), before, v, [[0, 2], [0, 1], [0, 0]])?;
                outer.push(behind[1].clone());
                b.spec.faces.push(FaceSpec {
                    surface,
                    outer,
                    holes: Vec::new(),
                });
                b.names
                    .faces
                    .push(namer.name(&[&mesh.face_names[f], &mesh.vertex_names[v]]));
            }
        }
    }
    b.spec.shells = vec![(0..b.spec.faces.len()).collect()];
    Ok((b.spec, b.names))
}
