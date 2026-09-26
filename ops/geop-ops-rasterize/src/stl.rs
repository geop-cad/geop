//! STL export: a [`RasterizedModel`]'s faces as a triangle mesh file.
//!
//! STL is a bag of triangles, each with a normal and its three corners
//! listed counter-clockwise seen from outside. The triangles here are the
//! ones the viewer draws (see [`crate::rasterize_model_tagged`]), so an
//! exported mesh is exactly what is on screen. Their winding comes from the
//! `(u, v)` triangulation, which says nothing about outside; the surface
//! normals do — every face's surface normal points out of its solid (see
//! `geop_core_topology::validation::face_orientation`) — so each triangle
//! is wound to agree with them.
//!
//! Faces are sampled one by one, so two faces sharing an edge each sample
//! it on their own: the mesh is as watertight as those samplings agree.

use std::io::{self, Write};

use geop_core_math::{primitives::TriangleFace, scalars::Scalar, vector::Vector3};
use geop_core_topology::FaceId;

use crate::RasterizedModel;

/// One STL facet: its outward normal, and its corners counter-clockwise
/// seen from outside.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StlTriangle {
    pub normal: [f32; 3],
    pub corners: [[f32; 3]; 3],
}

/// Which of STL's two encodings to write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StlFormat {
    /// Compact, and what most tools expect.
    Binary,
    /// Plain text, one line per normal and corner — readable and diffable.
    Ascii,
}

fn f32s<S: Scalar>(v: &Vector3<S>) -> [f32; 3] {
    [
        v[0].to_f64() as f32,
        v[1].to_f64() as f32,
        v[2].to_f64() as f32,
    ]
}

/// `t` as an STL facet, wound to face the way its surface does: outward.
/// A triangle without surface normals (at a pole) keeps its winding.
fn outward<S: Scalar>(t: &TriangleFace<S>) -> StlTriangle {
    let flat = f32s(&t.normal);
    let surface = t.vertex_normals.map(|ns| {
        ns.iter().map(f32s).fold([0.0; 3], |acc, n| {
            [acc[0] + n[0], acc[1] + n[1], acc[2] + n[2]]
        })
    });
    let agrees = surface.is_none_or(|s| flat[0] * s[0] + flat[1] * s[1] + flat[2] * s[2] >= 0.0);
    let [a, b, c] = [f32s(&t.a), f32s(&t.b), f32s(&t.c)];
    if agrees {
        StlTriangle {
            normal: flat,
            corners: [a, b, c],
        }
    } else {
        StlTriangle {
            normal: flat.map(|x| -x),
            corners: [a, c, b],
        }
    }
}

/// The triangles of `faces` of `raster`, in the order given, wound outward.
pub fn stl_triangles<S: Scalar>(raster: &RasterizedModel<S>, faces: &[FaceId]) -> Vec<StlTriangle> {
    faces
        .iter()
        .filter_map(|face| raster.faces.get(face))
        .flatten()
        .map(outward)
        .collect()
}

/// Write `triangles` as an STL file named `name` (the solid name an ASCII
/// file carries, and the header of a binary one).
pub fn write_stl(
    triangles: &[StlTriangle],
    name: &str,
    format: StlFormat,
    out: &mut impl Write,
) -> io::Result<()> {
    match format {
        StlFormat::Binary => {
            // An 80-byte header, which must not start with "solid" — some
            // readers take that for an ASCII file.
            let mut header = [b' '; 80];
            let text = format!("geop {name}");
            let len = text.len().min(80);
            header[..len].copy_from_slice(&text.as_bytes()[..len]);
            out.write_all(&header)?;
            let count = u32::try_from(triangles.len())
                .map_err(|_| io::Error::other("too many triangles for a binary STL file"))?;
            out.write_all(&count.to_le_bytes())?;
            for t in triangles {
                for v in std::iter::once(&t.normal).chain(&t.corners) {
                    for x in v {
                        out.write_all(&x.to_le_bytes())?;
                    }
                }
                // Attribute byte count, unused.
                out.write_all(&0u16.to_le_bytes())?;
            }
        }
        StlFormat::Ascii => {
            // A name is a single word in ASCII STL.
            let name: String = name
                .chars()
                .map(|c| if c.is_whitespace() { '_' } else { c })
                .collect();
            writeln!(out, "solid {name}")?;
            for t in triangles {
                let [nx, ny, nz] = t.normal;
                writeln!(out, "  facet normal {nx:e} {ny:e} {nz:e}")?;
                writeln!(out, "    outer loop")?;
                for [x, y, z] in t.corners {
                    writeln!(out, "      vertex {x:e} {y:e} {z:e}")?;
                }
                writeln!(out, "    endloop")?;
                writeln!(out, "  endfacet")?;
            }
            writeln!(out, "endsolid {name}")?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::scal_in_f64::ScalInF64, vector::Vector3};
    use geop_core_part::Part;
    use geop_ops_extrude_revolve::{cube_solid, sphere::sphere_solid};

    use super::*;
    use crate::rasterize_model_tagged;

    type S = ScalInF64;

    fn triangles_of(part: &Part<S>) -> Vec<StlTriangle> {
        let raster = rasterize_model_tagged(part.topology(), 8).unwrap();
        let mut faces: Vec<FaceId> = raster.faces.keys().copied().collect();
        faces.sort_by_key(|f| f.0);
        stl_triangles(&raster, &faces)
    }

    fn unit_cube() -> Vec<StlTriangle> {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "c",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        triangles_of(&part)
    }

    /// Every facet of a closed convex solid around `centre` faces away from
    /// it, by its winding and by its normal.
    fn assert_outward(triangles: &[StlTriangle], centre: [f32; 3]) {
        assert!(!triangles.is_empty());
        let sub = |p: [f32; 3], q: [f32; 3]| [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
        let dot = |p: [f32; 3], q: [f32; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
        for t in triangles {
            let [a, b, c] = t.corners;
            let (u, v) = (sub(b, a), sub(c, a));
            let cross = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let out = sub(a, centre);
            assert!(dot(cross, out) > 0.0, "{t:?} winds inward");
            assert!(dot(t.normal, out) > 0.0, "{t:?} has an inward normal");
        }
    }

    #[test]
    fn cube_facets_face_outward() {
        assert_outward(&unit_cube(), [0.5; 3]);
    }

    /// Curved faces: the winding has to come from the surface normals.
    #[test]
    fn sphere_facets_face_outward() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "s", Vector3::from_array([S::ZERO; 3]), S::ONE).unwrap();
        assert_outward(&triangles_of(&part), [0.0; 3]);
    }

    #[test]
    fn binary_layout() {
        let triangles = unit_cube();
        let mut bytes = Vec::new();
        write_stl(&triangles, "cube", StlFormat::Binary, &mut bytes).unwrap();
        assert_eq!(bytes.len(), 84 + 50 * triangles.len());
        assert!(!bytes.starts_with(b"solid"));
        assert_eq!(
            u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize,
            triangles.len()
        );
    }

    #[test]
    fn ascii_layout() {
        let triangles = unit_cube();
        let mut bytes = Vec::new();
        write_stl(&triangles, "my cube", StlFormat::Ascii, &mut bytes).unwrap();
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.starts_with("solid my_cube\n"));
        assert!(text.trim_end().ends_with("endsolid my_cube"));
        assert_eq!(text.matches("facet normal").count(), triangles.len());
        assert_eq!(text.matches("vertex").count(), 3 * triangles.len());
    }
}
