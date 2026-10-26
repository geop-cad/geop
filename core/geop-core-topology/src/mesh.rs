//! A body of triangles ([`BodySpec::of_triangles`]): what a mesh read from
//! a file (STL) is as a B-rep.
//!
//! The topology comes from the mesh's own indexing, not from stitching
//! geometry: two triangles share an edge where they share its two corners.
//! Each triangle is a face on its plane. The plane is a patch over a domain
//! the triangle does not decide — the box of every corner of the mesh seen
//! along the axis the plane faces most — so triangles in one plane lie on
//! the very same patch, and [`BodySpec::merged`] makes them one face at
//! once.

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use crate::{
    Curve2, Curve3, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};

impl BodySpec {
    /// The body the triangles `triangles` make, each three indices into
    /// `points`, counter-clockwise seen from the side its face points to:
    /// a solid where every edge is shared by two triangles, a sheet where
    /// some are by one. A triangle whose corners repeat an index has no
    /// area and no edge of its own, and is left out. Refused, naming the
    /// triangle: corners in one line, and an edge shared by more than two
    /// triangles or by two running it the same way.
    pub fn of_triangles(points: &[Vector3], triangles: &[[usize; 3]]) -> GeopResult<BodySpec> {
        let triangles: Vec<(usize, [usize; 3])> = (triangles.iter().copied().enumerate())
            .filter(|(_, [a, b, c])| a != b && b != c && c != a)
            .collect();
        if triangles.is_empty() {
            return Err(GeopError::new("the mesh has no triangles"));
        }
        if let Some((t, _)) =
            (triangles.iter()).find(|(_, corners)| corners.iter().any(|&p| p >= points.len()))
        {
            return Err(GeopError::new(format!(
                "triangle {t} names a corner the mesh does not have"
            )));
        }
        // The box of every corner, which each plane's patch spans.
        let lo = Vector3::from_array(std::array::from_fn(|k| {
            points
                .iter()
                .map(|p| p[k].lower())
                .fold(points[0][k].lower(), |a, b| {
                    if b.definitely_less(a) { b } else { a }
                })
        }));
        let hi = Vector3::from_array(std::array::from_fn(|k| {
            points
                .iter()
                .map(|p| p[k].upper())
                .fold(points[0][k].upper(), |a, b| {
                    if b.definitely_greater(a) { b } else { a }
                })
        }));
        let mut spec = BodySpec::new(false);
        spec.vertices = points.to_vec();
        let mut edge_of: std::collections::HashMap<(usize, usize), usize> = Default::default();
        let mut senses: Vec<Vec<Sense>> = Vec::new();
        for &(t, [a, b, c]) in &triangles {
            let corners = [points[a], points[b], points[c]];
            let normal = corners[1]
                .sub(&corners[0])
                .prod_cross(&corners[2].sub(&corners[0]));
            if normal.norm_sq().could_be_equal(Scalar::ZERO) {
                return Err(GeopError::new(format!(
                    "triangle {t} has its corners in one line: it has no plane"
                )));
            }
            // Seen along the axis the plane faces most — a free choice of
            // parametrization; any it faces at all would do.
            let k = (0..3)
                .max_by(|&i, &j| {
                    normal[i]
                        .abs()
                        .to_f64()
                        .total_cmp(&normal[j].abs().to_f64())
                })
                .expect("three axes");
            let (mut i, mut j) = ((k + 1) % 3, (k + 2) % 3);
            if normal[k].definitely_less(Scalar::ZERO) {
                std::mem::swap(&mut i, &mut j);
            }
            // The plane: `x_k = (n . p0 - n_i u - n_j v) / n_k`.
            let d = normal.prod_dot(&corners[0]);
            let point = |u: Scalar, v: Scalar| -> GeopResult<Vector3> {
                let mut p = Vector3::zero();
                p[i] = u;
                p[j] = v;
                p[k] = d
                    .sub(normal[i].mul(u))
                    .sub(normal[j].mul(v))
                    .div(normal[k])?;
                Ok(p)
            };
            let surface = NurbSurface3D::bilinear(
                point(lo[i], lo[j])?,
                point(hi[i], lo[j])?,
                point(hi[i], hi[j])?,
                point(lo[i], hi[j])?,
            )?;
            // Its `(u, v)` run over `[0, 1]`, the box's.
            let uv = |p: &Vector3| -> GeopResult<Vector2> {
                Ok(Vector2::from_array([
                    p[i].sub(lo[i]).div(hi[i].sub(lo[i]))?,
                    p[j].sub(lo[j]).div(hi[j].sub(lo[j]))?,
                ]))
            };
            let mut outer = Vec::with_capacity(3);
            for (from, to) in [(a, b), (b, c), (c, a)] {
                let key = (from.min(to), from.max(to));
                let e = *edge_of.entry(key).or_insert_with(|| {
                    spec.edges.push(EdgeSpec {
                        curve: Curve3::line(points[key.0], points[key.1])
                            .expect("two corners apart"),
                        start: key.0,
                        end: key.1,
                    });
                    senses.push(Vec::new());
                    spec.edges.len() - 1
                });
                let sense = if from == key.0 {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                if senses[e].len() >= 2 || senses[e].contains(&sense) {
                    return Err(GeopError::new(format!(
                        "triangle {t} shares its edge from corner {from} to {to} with {} other triangle(s), {}: the mesh is no surface there",
                        senses[e].len(),
                        if senses[e].contains(&sense) {
                            "running it the same way"
                        } else {
                            "more than one"
                        }
                    )));
                }
                senses[e].push(sense);
                outer.push(CoedgeSpec {
                    on: CoedgeOn::Edge(e, sense),
                    pcurve: Curve2::line(uv(&points[from])?, uv(&points[to])?)?,
                });
            }
            spec.faces.push(FaceSpec {
                surface,
                outer,
                holes: Vec::new(),
            });
        }
        spec.solid = senses.iter().all(|s| s.len() == 2);
        spec.shells = vec![(0..spec.faces.len()).collect()];
        // Corners no triangle uses are no vertices.
        let mut used = vec![false; points.len()];
        for edge in &spec.edges {
            used[edge.start] = true;
            used[edge.end] = true;
        }
        let mut index = vec![usize::MAX; points.len()];
        let mut vertices = Vec::new();
        for (p, &u) in used.iter().enumerate() {
            if u {
                index[p] = vertices.len();
                vertices.push(points[p]);
            }
        }
        for edge in &mut spec.edges {
            edge.start = index[edge.start];
            edge.end = index[edge.end];
        }
        spec.vertices = vertices;
        spec.check()?;
        Ok(spec)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use crate::{
        Model,
        build::BodySpec,
        validation::{ValidationParameters, validate},
    };

    /// The unit cube as twelve triangles, two per side, outward.
    fn cube() -> (Vec<Vector3>, Vec<[usize; 3]>) {
        let points = (0..8)
            .map(|k| {
                Vector3::from_array(
                    [k & 1, (k >> 1) & 1, (k >> 2) & 1].map(|c| Scalar::from_f64(c as f64)),
                )
            })
            .collect();
        let quads = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let triangles = quads
            .iter()
            .flat_map(|[a, b, c, d]| [[*a, *b, *c], [*a, *c, *d]])
            .collect();
        (points, triangles)
    }

    /// Twelve triangles of a cube: a valid solid, and merged, six faces.
    #[test]
    fn a_cube_of_triangles_is_a_box() {
        let (points, triangles) = cube();
        let spec = BodySpec::of_triangles(&points, &triangles).unwrap();
        assert!(spec.solid);
        assert_eq!(spec.faces.len(), 12);
        let mut model = Model::new();
        model.build_body(spec.clone()).unwrap();
        validate(&ValidationParameters::default(), &model).unwrap();
        let (merged, _) = spec.merged(|_, _| true).unwrap();
        assert_eq!(merged.faces.len(), 6);
        assert_eq!(merged.edges.len(), 12);
        let mut model = Model::new();
        model.build_body(merged).unwrap();
        validate(&ValidationParameters::default(), &model).unwrap();
    }

    /// A triangle left out: an open box, a sheet.
    #[test]
    fn a_cube_missing_a_triangle_is_a_sheet() {
        let (points, mut triangles) = cube();
        triangles.pop();
        let spec = BodySpec::of_triangles(&points, &triangles).unwrap();
        assert!(!spec.solid);
    }

    /// A triangle turned the other way shares edges running them the same
    /// way as its neighbours: refused, naming it.
    #[test]
    fn a_triangle_turned_round_is_refused() {
        let (points, mut triangles) = cube();
        triangles[11].swap(1, 2);
        let error = BodySpec::of_triangles(&points, &triangles).unwrap_err();
        assert!(format!("{error}").contains("same way"), "{error}");
    }
}
