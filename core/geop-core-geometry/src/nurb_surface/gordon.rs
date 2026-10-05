//! The Gordon surface through a network of curves
//! ([`NurbSurface3D::gordon`]): curves in two directions, each crossing
//! every curve of the other once, interpolated all at once.
//!
//! It is the Coons construction (see [`NurbSurface3D::coons`]) carried from
//! four sides to any number of curves: the surface skinned through the `u`
//! curves, plus the one skinned through the `v` curves, less the
//! tensor-product surface through the points where they cross. Each of the
//! three runs through the crossings; along a `u` curve the second and the
//! third agree, so the sum is the `u` curve there, and the same holds for
//! the `v` curves. Everything is done on homogeneous control points, so it
//! is exact for rational curves too.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface3D;
use crate::nurb_curve::{NurbCurve, NurbCurve3D};

/// The degree a network is skinned with across its curves: cubic, or lower
/// where there are too few curves for it.
const SKIN_DEGREE: usize = 3;

fn exactly_one<S: Scalar>(w: S) -> bool {
    w.is_subset_of(S::ONE) && S::ONE.is_subset_of(w)
}

/// Where a family of curves is to cross the curves of the other family, on
/// the domain `[0, 1]` they are brought onto: per crossed curve, where each
/// of the family crosses it, as a fraction of its stretch between its first
/// and last crossing, averaged over the family. Which parameters is a free
/// choice — any increasing ones make the same surface pass through the
/// same curves — and the average keeps each curve's own speed nearly as it
/// was, so the parameters are sharpened, and the ends are exactly 0 and 1.
fn common_parameters<S: Scalar>(crossings: &[Vec<S>]) -> GeopResult<Vec<S>> {
    let n = crossings[0].len();
    let count = S::from_i64(crossings.len() as i64);
    let mut params = vec![S::ZERO; n];
    params[n - 1] = S::ONE;
    for (i, param) in params.iter_mut().enumerate().take(n - 1).skip(1) {
        let mut sum = S::ZERO;
        for c in crossings {
            sum = sum.add(c[i].sub(c[0]).div(c[n - 1].sub(c[0]))?);
        }
        *param = sum.div(count)?.sharpen();
    }
    if !params.windows(2).all(|w| w[0].definitely_less(w[1])) {
        return Err(GeopError::new(format!(
            "NurbSurface::gordon: the crossings {crossings:?} do not each run in one order"
        )));
    }
    Ok(params)
}

/// The surface skinned through `rows` — compatible curves along `u`, on
/// one knot vector — the `j`-th at `v = params[j]`: each column of their
/// control points interpolated across them (see
/// [`NurbCurve::interpolate_homogeneous`]). Its first and last rows are the
/// first and last curves' control points: that is what the interpolation
/// gives there exactly, and taking them as they are leaves out its
/// rounding.
fn skin<S: Scalar>(rows: &[NurbCurve3D<S>], params: &[S]) -> GeopResult<NurbSurface3D<S>> {
    let first = &rows[0];
    let nu = first.control_points.len();
    let mut columns = Vec::with_capacity(nu);
    for k in 0..nu {
        let values: Vec<Vector4<S>> = rows.iter().map(|r| r.control_points[k]).collect();
        let mut column = NurbCurve::interpolate_homogeneous(&values, params, SKIN_DEGREE)?;
        let last = column.control_points.len() - 1;
        column.control_points[0] = values[0];
        column.control_points[last] = values[values.len() - 1];
        columns.push(column);
    }
    let nv = columns[0].control_points.len();
    let control_points = (0..nu)
        .flat_map(|i| (0..nv).map(move |j| (i, j)))
        .map(|(i, j)| columns[i].control_points[j])
        .collect();
    NurbSurface3D::try_new(
        first.degree,
        columns[0].degree,
        control_points,
        first.knot_vector.clone(),
        columns[0].knot_vector.clone(),
    )
}

impl<S: Scalar> NurbSurface3D<S> {
    /// The same surface with `u` and `v` swapped.
    fn transposed(&self) -> Self {
        let (nu, nv) = (self.num_u, self.num_v);
        let control_points = (0..nv)
            .flat_map(|j| (0..nu).map(move |i| (i, j)))
            .map(|(i, j)| self.control_points[i * nv + j])
            .collect();
        Self {
            degree_u: self.degree_v,
            degree_v: self.degree_u,
            num_u: nv,
            num_v: nu,
            control_points,
            knot_vector_u: self.knot_vector_v.clone(),
            knot_vector_v: self.knot_vector_u.clone(),
            aabb: self.aabb,
        }
    }

    /// Its rows of control points along `u`, as curves, one per `v` index.
    fn rows_along_u(&self) -> GeopResult<Vec<NurbCurve3D<S>>> {
        (0..self.num_v)
            .map(|j| {
                let points = (0..self.num_u)
                    .map(|i| self.control_points[i * self.num_v + j])
                    .collect();
                NurbCurve::try_new(self.degree_u, points, self.knot_vector_u.clone())
            })
            .collect()
    }

    /// `surfaces`, all on one domain, made compatible along `u` (see
    /// [`NurbCurve::compatible`]): one degree and one knot vector, every
    /// surface still the surface it was.
    fn compatible_along_u(surfaces: &[Self]) -> GeopResult<Vec<Self>> {
        let mut rows = Vec::new();
        for s in surfaces {
            rows.extend(s.rows_along_u()?);
        }
        let mut rows = NurbCurve::compatible(&rows)?.into_iter();
        surfaces
            .iter()
            .map(|s| {
                let curves: Vec<_> = rows.by_ref().take(s.num_v).collect();
                let num_u = curves[0].control_points.len();
                let control_points = (0..num_u)
                    .flat_map(|i| (0..s.num_v).map(move |j| (i, j)))
                    .map(|(i, j)| curves[j].control_points[i])
                    .collect();
                NurbSurface3D::try_new(
                    curves[0].degree,
                    s.degree_v,
                    control_points,
                    curves[0].knot_vector.clone(),
                    s.knot_vector_v.clone(),
                )
            })
            .collect()
    }

    /// The Gordon surface through a network of curves on `[0, 1]²`: the
    /// curves `u_curves` along `u`, in order of increasing `v`, and
    /// `v_curves` along `v`, in order of increasing `u`, each `u` curve
    /// crossing each `v` curve once. `u_crossings[j][i]` is the parameter of
    /// `u_curves[j]` where it crosses `v_curves[i]`, `v_crossings[i][j]`
    /// that of `v_curves[i]` there; both increase along each curve. Two
    /// curves at least in each direction.
    ///
    /// Each curve is reparametrized (see [`NurbCurve::reparametrized`]) so
    /// that the curves cross at common parameters, chosen as
    /// `common_parameters` says, and cut to the stretch between its first
    /// and last crossing: the surface's sides are the first and last curves
    /// of each family. The crossing points are each the union of where the
    /// two curves have them — an error naming both if they could not be
    /// one point. The surface is then the sum of the two skinned surfaces
    /// less the tensor-product one, each of degree three across the curves,
    /// or lower for fewer than four. Its border rows are the outer curves'
    /// control points, as [`NurbSurface3D::coons`]' are, so it interpolates
    /// them exactly, and with two curves each way it is their Coons patch.
    ///
    /// Across a crossed curve the surface is as smooth as the curves'
    /// reparametrizations: where they cross every curve at the same
    /// fraction of its length, smooth; otherwise each curve's speed changes
    /// there, and the surface may bend across it.
    pub fn gordon(
        u_curves: &[NurbCurve3D<S>],
        v_curves: &[NurbCurve3D<S>],
        u_crossings: &[Vec<S>],
        v_crossings: &[Vec<S>],
    ) -> GeopResult<Self> {
        let (m, n) = (u_curves.len(), v_curves.len());
        let shaped = |crossings: &[Vec<S>], curves: usize, each: usize| {
            crossings.len() == curves && crossings.iter().all(|c| c.len() == each)
        };
        if m < 2 || n < 2 || !shaped(u_crossings, m, n) || !shaped(v_crossings, n, m) {
            return Err(GeopError::new(format!(
                "NurbSurface::gordon: {m} u curves and {n} v curves, two at least each way, need the crossings of each with every other, not {u_crossings:?} and {v_crossings:?}"
            )));
        }
        let u_params = common_parameters(u_crossings)?;
        let v_params = common_parameters(v_crossings)?;
        let reparametrized = |curves: &[NurbCurve3D<S>], crossings: &[Vec<S>], params: &[S]| {
            curves
                .iter()
                .zip(crossings)
                .map(|(c, x)| c.reparametrized(x, params))
                .collect::<GeopResult<Vec<_>>>()
        };
        let u = reparametrized(u_curves, u_crossings, &u_params)?;
        let v = reparametrized(v_curves, v_crossings, &v_params)?;

        // The crossings, `points[j][i]` where `u[j]` crosses `v[i]`, of
        // weight one: every curve's weight is one at its crossings.
        let mut points = Vec::with_capacity(m);
        for (j, uj) in u.iter().enumerate() {
            let mut row = Vec::with_capacity(n);
            for (i, vi) in v.iter().enumerate() {
                let a = uj.evaluate(u_params[i])?;
                let b = vi.evaluate(v_params[j])?;
                if !a.could_be_equal(&b) {
                    return Err(GeopError::new(format!(
                        "NurbSurface::gordon: u curve {j} is at {a:?} where it crosses v curve {i}, which is at {b:?}"
                    )));
                }
                let p: Vector3<S> = a.union(&b);
                row.push(Vector4::from_array([p[0], p[1], p[2], S::ONE]));
            }
            points.push(row);
        }
        let rational = u
            .iter()
            .chain(&v)
            .any(|c| c.control_points.iter().any(|p| !exactly_one(p[3])));

        let along_u = skin(&NurbCurve::compatible(&u)?, &v_params)?;
        let along_v = skin(&NurbCurve::compatible(&v)?, &u_params)?.transposed();
        let through_points = {
            let rows = points
                .iter()
                .map(|row| NurbCurve::interpolate_homogeneous(row, &u_params, SKIN_DEGREE))
                .collect::<GeopResult<Vec<_>>>()?;
            skin(&rows, &v_params)?
        };
        let surfaces = Self::compatible_along_u(&[along_u, along_v, through_points])?;
        let transposed: Vec<Self> = surfaces.iter().map(Self::transposed).collect();
        let surfaces: Vec<Self> = Self::compatible_along_u(&transposed)?
            .iter()
            .map(Self::transposed)
            .collect();
        let [along_u, along_v, through_points] = &surfaces[..] else {
            unreachable!("three surfaces");
        };

        let (nu, nv) = (along_u.num_u, along_u.num_v);
        let mut control_points = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in 0..nv {
                let k = i * nv + j;
                let (on_u_side, on_v_side) = (j == 0 || j == nv - 1, i == 0 || i == nu - 1);
                let mut p = match (on_u_side, on_v_side) {
                    (true, true) => along_u.control_points[k].union(&along_v.control_points[k]),
                    (true, false) => along_u.control_points[k],
                    (false, true) => along_v.control_points[k],
                    (false, false) => along_u.control_points[k]
                        .add(&along_v.control_points[k])
                        .sub(&through_points.control_points[k]),
                };
                if !rational {
                    // Every weight is one, and so is every blend of them:
                    // the arithmetic above would only round it.
                    p[3] = S::ONE;
                }
                control_points.push(p);
            }
        }
        NurbSurface3D::try_new(
            along_u.degree_u,
            along_u.degree_v,
            control_points,
            along_u.knot_vector_u.clone(),
            along_u.knot_vector_v.clone(),
        )
    }
}

#[cfg(test)]
mod tests;
