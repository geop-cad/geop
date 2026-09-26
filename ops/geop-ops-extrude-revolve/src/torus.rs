//! A torus built from a 4x4 grid of NURBS patches with seams in both
//! parametric directions.

use crate::common::{
            EdgeRegistry, finish_solid, line2, make_coedge, new_face, sqrt2_over_2, vertex,
            wire_loop,
        };
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::{Vector2, Vector3, Vector4}};
use geop_core_geometry::nurb_surface::{NurbSurface, NurbSurface3D};
use geop_core_topology::{CoedgeId, Model, Sense, Shell, SolidId, VertexId};

const N: usize = 4; // segments around the major circle (theta)
const M: usize = 4; // segments around the minor circle (phi)

fn add<S: Scalar>(x: Vector3<S>, y: Vector3<S>) -> Vector3<S> {
    Vector3::from_array([x[0].add(y[0]), x[1].add(y[1]), x[2].add(y[2])])
}

fn weighted_pt3<S: Scalar>(p: Vector3<S>, w: S) -> Vector4<S> {
    Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), w])
}

fn pt3<S: Scalar>(p: Vector3<S>) -> Vector4<S> {
    Vector4::from_array([p[0], p[1], p[2], S::ONE])
}

/// Arc-midpoint control point for a 90-degree circular arc from `p0` to `p2`
/// around `center`.
fn arc_mid<S: Scalar>(p0: Vector3<S>, p2: Vector3<S>, center: Vector3<S>) -> Vector3<S> {
    add(p0, p2).sub(&center)
}

/// A degree-(2,2) patch whose 4 edge_loop curves are exact 90-degree circular
/// arcs: `v=0`: `p00 -> p10` around `center_v0`; `u=1`: `p10 -> p11` around
/// `center_u1`; `v=1` (reversed): `p11 -> p01` around `center_v1`; `u=0`
/// (reversed): `p01 -> p00` around `center_u0`.
#[allow(clippy::too_many_arguments)]
fn grid_patch_surface<S: Scalar>(
    p00: Vector3<S>,
    p10: Vector3<S>,
    p01: Vector3<S>,
    p11: Vector3<S>,
    center_v0: Vector3<S>,
    center_u1: Vector3<S>,
    center_v1: Vector3<S>,
    center_u0: Vector3<S>,
) -> GeopResult<NurbSurface3D<S>> {
    let w: S = sqrt2_over_2();
    let mid_v0 = arc_mid(p00, p10, center_v0);
    let mid_u1 = arc_mid(p10, p11, center_u1);
    let mid_v1 = arc_mid(p01, p11, center_v1);
    let mid_u0 = arc_mid(p00, p01, center_u0);
    // The patch is the tensor product of two exact quarter-circle arcs (the
    // `u` profile arc around the tube's cross-section, the `v` arc around
    // the major circle), so the center control point/weight is the
    // corresponding combination of the two arcs' midpoints/weights: the
    // weight grid is the outer product of `[1, w, 1]` with itself
    // (center weight `w*w`), and the center position is the `v`-arc
    // midpoints' "arc_mid" sum around the `u`-arc centers (equivalently the
    // `u`-arc midpoints' sum around the `v`-arc centers).
    let m = w.mul(w);
    let interior = add(mid_v0, mid_v1).sub(&add(center_u0, center_u1));

    let knots = vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE];
    // control_points[r*3 + s], r,s in {0,1,2}.
    let cps = vec![
        pt3(p00),
        weighted_pt3(mid_u0, w),
        pt3(p01),
        weighted_pt3(mid_v0, w),
        weighted_pt3(interior, m),
        weighted_pt3(mid_v1, w),
        pt3(p10),
        weighted_pt3(mid_u1, w),
        pt3(p11),
    ];
    NurbSurface::try_new(2, 2, cps, knots.clone(), knots)
}

/// Build a closed, manifold torus with major radius `major_r` (distance from
/// the center of the tube to the center of the torus) and minor radius
/// `minor_r` (radius of the tube), as a 4x4 grid of NURBS patches. Both grid
/// directions wrap around (seams), so opposite coedges may belong to the
/// same face's loop.
pub fn torus_solid<S: Scalar>(model: &mut Model<S>, major_r: S, minor_r: S) -> GeopResult<SolidId> {
    let z = S::ZERO;
    let one = S::ONE;
    // cos/sin at multiples of 90 degrees.
    let cos_t = [one, z, z.sub(one), z];
    let sin_t = [z, one, z, z.sub(one)];
    let cos_p = cos_t;
    let sin_p = sin_t;

    // V[i][j] for i in 0..N (theta), j in 0..M (phi).
    let mut positions = [[Vector3::<S>::zero(); M]; N];
    let mut verts = [[VertexId(0); M]; N];
    for i in 0..N {
        for j in 0..M {
            let ring_r = major_r.add(minor_r.mul(cos_p[j]));
            let x = ring_r.mul(cos_t[i]);
            let y = ring_r.mul(sin_t[i]);
            let zc = minor_r.mul(sin_p[j]);
            let p = Vector3::from_array([x, y, zc]);
            positions[i][j] = p;
            verts[i][j] = vertex(model, p);
        }
    }

    // Circle centers.
    let horiz_center = |j: usize| Vector3::from_array([z, z, minor_r.mul(sin_p[j])]);
    let vert_center =
        |i: usize| Vector3::from_array([major_r.mul(cos_t[i]), major_r.mul(sin_t[i]), z]);

    let shell_id = model.insert_shell(Shell {
        faces: vec![],
        solid: None,
    });
    let mut registry = EdgeRegistry::new();

    for i in 0..N {
        for j in 0..M {
            let i1 = (i + 1) % N;
            let j1 = (j + 1) % M;

            let p00 = positions[i][j];
            let p10 = positions[i1][j];
            let p01 = positions[i][j1];
            let p11 = positions[i1][j1];

            let surface = grid_patch_surface(
                p00,
                p10,
                p01,
                p11,
                horiz_center(j),
                vert_center(i1),
                horiz_center(j1),
                vert_center(i),
            )?;
            let (_face_id, loop_id) = new_face(model, shell_id, surface, Sense::Forward);

            let (v00, v10, v01, v11) = (verts[i][j], verts[i1][j], verts[i][j1], verts[i1][j1]);
            let (c_v0, c_u1, c_v1, c_u0) = (
                horiz_center(j),
                vert_center(i1),
                horiz_center(j1),
                vert_center(i),
            );

            // EdgeLoop: (P00,P10) [v=0], (P10,P11) [u=1], (P11,P01) [v=1 rev], (P01,P00) [u=0 rev].
            let segs: [(
                VertexId,
                Vector3<S>,
                VertexId,
                Vector3<S>,
                Vector3<S>,
                (f64, f64),
                (f64, f64),
            ); 4] = [
                (v00, p00, v10, p10, c_v0, (0.0, 0.0), (1.0, 0.0)),
                (v10, p10, v11, p11, c_u1, (1.0, 0.0), (1.0, 1.0)),
                (v11, p11, v01, p01, c_v1, (1.0, 1.0), (0.0, 1.0)),
                (v01, p01, v00, p00, c_u0, (0.0, 1.0), (0.0, 0.0)),
            ];

            let mut coedge_ids = [CoedgeId(0); 4];
            for (k, &(va, vap, vb, vbp, center, uv0, uv1)) in segs.iter().enumerate() {
                let (edge_id, sense) =
                    registry.get_or_create_edge(model, va, vb, || {
                        crate::common::arc3(
                            vap,
                            arc_mid(vap, vbp, center),
                            vbp,
                            sqrt2_over_2(),
                        )
                    })?;

                let pcurve = line2(
                    Vector2::from_array([S::from_f64(uv0.0), S::from_f64(uv0.1)]),
                    Vector2::from_array([S::from_f64(uv1.0), S::from_f64(uv1.1)]),
                )?;
                let coedge_id = make_coedge(model, edge_id, sense, loop_id, pcurve);
                coedge_ids[k] = coedge_id;
                registry.register_coedge(va, vb, coedge_id);
            }

            wire_loop(model, loop_id, &coedge_ids);
        }
    }

    registry.wire_opposites(model)?;

    Ok(finish_solid(model, shell_id))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;
    use geop_core_topology::validation::{validate, validate_manifold};

    fn check_torus_is_valid<S: Scalar>() {
        let mut model = Model::<S>::new();
        torus_solid(&mut model, S::from_f64(2.0), S::ONE).unwrap();

        if let Err(e) = validate(&model, 5) {
            panic!("{e}");
        }
        if let Err(e) = validate_manifold(&model) {
            panic!("{e}");
        }
    }
    #[test]
    fn torus_is_valid() {
        for_all_scalars!(check_torus_is_valid);
    }

    /// Every grid patch is a tensor product of two exact quarter-circle
    /// arcs, so every interior `(u, v)` point (not just the patch edge_loop)
    /// must satisfy the torus's implicit equation
    /// `(sqrt(x^2 + y^2) - major_r)^2 + z^2 == minor_r^2`.
    fn check_torus_patch_points_on_torus<S: Scalar>() {
        let mut model = Model::<S>::new();
        let major_r = S::from_f64(2.0);
        let minor_r = S::ONE;
        torus_solid(&mut model, major_r, minor_r).unwrap();

        let minor_r_sq = minor_r.mul(minor_r);
        for face in model.faces.values() {
            let surface = &face.surface;
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=4 {
                for j in 0..=4 {
                    let u = u0.add(u1.sub(u0).mul(S::from_ratio(i as i64, 4).unwrap()));
                    let v = v0.add(v1.sub(v0).mul(S::from_ratio(j as i64, 4).unwrap()));
                    let p = surface.evaluate(u, v).unwrap();
                    let radial = p[0].mul(p[0]).add(p[1].mul(p[1])).sqrt().unwrap();
                    let lhs = radial
                        .sub(major_r)
                        .mul(radial.sub(major_r))
                        .add(p[2].mul(p[2]));
                    let err = lhs.sub(minor_r_sq).abs();
                    assert!(
                        !err.definitely_greater(S::from_f64(1e-9)),
                        "u={u}, v={v}, p={p:?}, lhs={lhs}, expected={minor_r_sq}"
                    );
                }
            }
        }
    }
    #[test]
    fn torus_patch_points_on_torus() {
        for_all_scalars!(check_torus_patch_points_on_torus);
    }
}
