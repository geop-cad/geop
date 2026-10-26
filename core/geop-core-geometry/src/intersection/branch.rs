//! The branch where two surfaces meet, followed from point to point: a
//! corrector that brings a predicted point onto both surfaces at once
//! ([`predictor_corrector_step`]), and a curve fitted through points of a
//! branch that encloses the branch between them too ([`fit_branch`]).
//!
//! Where the branch runs and where it ends is the caller's to decide: the
//! booleans march from a piercing point to the vertex they reach
//! (`geop-lib-booleans`' `trace_one_side`); a shell knows both ends, the
//! moved vertices, and the edge whose inner copy the branch is.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::Matrix,
    scalars::Scalar,
    solvers::{
        iterate::{Repeat, Step, iterate},
        linear::solve_linear_system,
    },
    vector::{Vector, Vector3},
    with_context,
};

use crate::{
    nurb_curve::{NurbCurve, WidestBetween},
    nurb_surface::NurbSurface3D,
};

/// Where a point of an intersection branch is on both surfaces:
/// `(u_a, v_a, u_b, v_b)`.
pub type OnBoth = (Scalar, Scalar, Scalar, Scalar);

/// How many damped-Newton iterations the corrector may take to bring a
/// predicted point onto *both* surfaces at once. Like [`NEWTON_ITERATIONS`]
/// this bounds effort, not correctness: the loop exits early the moment the
/// two surfaces' enclosures of the point overlap, and failing to converge
/// within the budget is reported as an error rather than accepted.
const CORRECTOR_ITERATIONS: usize = 20;

/// Levenberg-Marquardt damping for the corrector's linear solve. Purely a
/// conditioning term — it keeps the step finite where the 4x4 system is
/// rank-deficient (the two surfaces locally parallel, or a parametric pole
/// where one surface's `Su`/`Sv` collapse), at the cost of a shorter step
/// that the next iteration simply continues. It cannot affect what a
/// converged answer *means*: the `could_be_equal` test below is what
/// decides whether the corrector actually landed on the curve.
const CORRECTOR_DAMPING: f64 = 1e-12;

/// One predictor-corrector step from `point` (with known `(u_a, v_a)` /
/// `(u_b, v_b)` on `surf_a`/`surf_b`) along `dir`: predicts `point + dir *
/// step_size`, then solves for the point that lies on *both* surfaces at
/// once by damped Newton, returning the union of the two surfaces'
/// enclosures of it plus each surface's own resulting `(u, v)`.
///
/// The corrector solves one system in the four unknowns `(u_a, v_a, u_b,
/// v_b)` rather than projecting onto each surface separately. Separate
/// projections each enforce their own surface's constraint while ignoring
/// the other's, so they land a full prediction-error apart and stay there
/// no matter how good the predictor is — and that error is not small: the
/// marching point inherits the start vertex's own interval width (a vertex
/// found by `curve_surface_intersect` is located only to within
/// `min_subdivision_size`), which `sharpen` turns into a concrete offset.
///
/// Three of the four residuals are `S_a(u_a, v_a) - S_b(u_b, v_b) = 0`.
/// Those alone are underdetermined by exactly one degree of freedom — the
/// intersection curve is a curve, so sliding along it satisfies them
/// equally well — and that free direction is what the damper has to pin
/// down. It pins it *biased to the last direction*: the fourth residual
/// holds the solution on the plane through the prediction with normal
/// `dir`, so the corrector may move freely onto the curve but not drift
/// along it, and the step stays the one the predictor asked for.
///
/// A step that would leave a patch is *not* rejected: `project` clamps into
/// the domain, so such a step lands exactly on the patch boundary — which
/// is precisely where an intersection curve ends, and therefore where the
/// vertex it must connect to lives. Refusing it instead stops the march a
/// full `step_size` short of that vertex, which is how a trace ends up
/// never reaching anything. Deciding whether a direction is viable at all
/// is `face_contains`' job on the first step (see a trace's), not
/// something for every step to re-litigate via raw domain bounds.
#[allow(clippy::too_many_arguments)]
pub fn predictor_corrector_step(
    surf_a: &NurbSurface3D,
    surf_b: &NurbSurface3D,
    point: Vector3,
    dir: Vector3,
    u_a: Scalar,
    v_a: Scalar,
    u_b: Scalar,
    v_b: Scalar,
    step_size: Scalar,
) -> GeopResult<(Vector3, Scalar, Scalar, Scalar, Scalar)> {
    let ctx = with_context!(
        "predictor_corrector_step(point={point:?}, dir={dir:?}, u_a={u_a:?}, v_a={v_a:?}, u_b={u_b:?}, v_b={v_b:?}, step_size={step_size:?})"
    );
    // Sharpened exactly once, here. The prediction is a seed and a bias for
    // the damped Newton below, free to be any point along the step, so
    // collapsing it costs no accuracy — and it fixes the plane the corrector
    // solves against. Nothing downstream of this line sharpens.
    //
    // The plane's normal `dir` is as free as its point: any plane crossing
    // the curve pins the corrector's free direction equally well. Left wide,
    // it is the one wide row of an otherwise sharp Jacobian — and where the
    // two surfaces meet at a shallow angle, `dir` (a normalized cross product
    // of nearly parallel normals, taken at the previous step's honest
    // `(u, v)`) is wide by percents, enough to put zero inside a pivot of the
    // nearly singular system and fail the solve.
    let dir = dir.sharpen();
    let target = point.add(&dir.prod_scalar(step_size)).sharpen();
    let (u_a_lo, u_a_hi) = surf_a.domain_u();
    let (v_a_lo, v_a_hi) = surf_a.domain_v();
    let (u_b_lo, u_b_hi) = surf_b.domain_u();
    let (v_b_lo, v_b_hi) = surf_b.domain_v();
    let lambda = Scalar::from_f64(CORRECTOR_DAMPING);

    // Two copies of the iterate ([`iterate`]): the sharp *seed* the next
    // Newton step starts from, and the *answer*, that same value with the
    // last update's width kept, which is what gets returned.
    //
    // Sharpening the seed is legitimate (any point inside it is an equally
    // good place to start) and necessary (left unsharpened, 20 steps compound
    // width until `evaluate` fails outright — measured at 36/175 scenes).
    // Sharpening the *answer* is not: callers ask `face_contains` where this
    // point is, and a sharp `(u, v)` sitting 6e-17 off a trim boundary reports
    // `Inside` a face it is actually on the edge of, because `could_be_equal`
    // has no width to work with. That produced traced curves running along a
    // face's own boundary, spliced in as zero-area slivers.
    //
    // Seeding sharp while returning honest is what the wart needed: the
    // returned width is that of a *single* step from a sharp seed — bounded
    // per step rather than merely not discarded — so it states how well this
    // iterate is pinned down without compounding.
    //
    // That holds for the first seed too. The `(u, v)` handed in are the
    // previous step's honest answer, as wide as that step left them; used
    // as the seed unsharpened, each step starts from the last one's width,
    // carries it through the residual into its own, and the march compounds
    // it step after step. On a tool whose corners are only known to 1e-12 —
    // computed from a solid's own vertices — that reached a whole patch's
    // width within a dozen steps (`cube_minus_box_with_wide_corners`).
    let clamp = |[u_a, v_a, u_b, v_b]: [Scalar; 4]| {
        [
            u_a.clamped(u_a_lo, u_a_hi),
            v_a.clamped(v_a_lo, v_a_hi),
            u_b.clamped(u_b_lo, u_b_hi),
            v_b.clamped(v_b_lo, v_b_hi),
        ]
    };
    let sharp = |x: [Scalar; 4]| x.map(|s| s.sharpen());
    let given = [u_a, v_a, u_b, v_b];
    // Where the two surfaces were found to meet.
    let mut met = None;
    let corrected = iterate(
        clamp(sharp(given)),
        given,
        CORRECTOR_ITERATIONS,
        Repeat::Never,
        // Each update is kept twice: honestly, and sharpened as the next
        // seed — see above for why the two differ. Sharpen *before*
        // clamping, as `NurbSurface::project` does: `clamped` cannot pull
        // back an interval that merely straddles a domain bound, and
        // collapsing such an interval to its midpoint can land just past
        // the bound, where the next `evaluate` rejects it. On a sharp value
        // `clamped`'s comparisons are exact, so the seed is always in domain.
        |&next| {
            let honest = clamp(next);
            (clamp(sharp(honest)), honest)
        },
        |&[u_a1, v_a1, u_b1, v_b1]| -> GeopResult<Step<[Scalar; 4]>> {
            let p_a = surf_a
                .evaluate(u_a1, v_a1)
                .with_context(&|e: GeopError| e.with_context("evaluate surf_a"))
                .with_context(&ctx)?;
            let p_b = surf_b
                .evaluate(u_b1, v_b1)
                .with_context(&|e: GeopError| e.with_context("evaluate surf_b"))
                .with_context(&ctx)?;
            // All *four* residuals have to be satisfied, not just the three
            // that put the point on both surfaces. The corrector is handed
            // the previous step's converged `(u, v)`, which already lies on
            // both surfaces — testing only those three would report success
            // without ever moving, leaving the march pinned at its first
            // step forever. The fourth residual is what says the point has
            // actually advanced to the plane the predictor stepped to.
            let on_plane = dir.prod_dot(&p_a.sub(&target));
            if p_a.could_be_equal(&p_b) && on_plane.could_be_equal(Scalar::ZERO) {
                // The union of the two surfaces' enclosures, not their
                // average: both enclose the *same* point on the
                // intersection curve, so the smallest interval containing
                // both is the honest answer. Averaging instead invents a
                // sharp midpoint claiming more precision than either had —
                // and hides how far apart they were, exactly the signal
                // that the two surfaces have started to disagree about
                // where the curve is.
                met = Some(p_a.union(&p_b));
                return Ok(Step::Done);
            }

            let (sau, sav) = surf_a
                .derivatives(u_a1, v_a1)
                .with_context(&|e: GeopError| e.with_context("derivatives of surf_a"))
                .with_context(&ctx)?;
            let (sbu, sbv) = surf_b
                .derivatives(u_b1, v_b1)
                .with_context(&|e: GeopError| e.with_context("derivatives of surf_b"))
                .with_context(&ctx)?;

            // Residuals: the two surfaces must meet (rows 0..3), and the
            // meeting point must sit on the plane the predictor stepped to
            // (row 3).
            let delta = p_a.sub(&p_b);
            let mut f = [Scalar::ZERO; 4];
            let mut j = [[Scalar::ZERO; 4]; 4];
            for k in 0..3 {
                f[k] = delta[k];
                j[k] = [sau[k], sav[k], sbu[k].neg(), sbv[k].neg()];
            }
            f[3] = on_plane;
            j[3] = [
                dir.prod_dot(&sau),
                dir.prod_dot(&sav),
                Scalar::ZERO,
                Scalar::ZERO,
            ];

            // Damped normal equations `(J^T J + lambda I) d = -J^T f`. Going
            // through `J^T J` rather than solving `J d = -f` directly is
            // what lets the damping act at all, and keeps the step finite
            // where `J` loses rank instead of erroring out of an otherwise
            // fine trace.
            //
            // The matrix is sharpened: it only steers the step, and any
            // matrix near the true Jacobian leads Newton to the same fixed
            // point — the residual `f` alone decides where that is, so the
            // step's honest width is the residual's, carried through a sharp
            // preconditioner (as in Krawczyk's `Y = mid(J)^-1`). Left as
            // intervals, the elimination amplifies the surfaces' own width by
            // the system's condition, which `J^T J` squares: where the
            // surfaces meet at a shallow angle, that put zero inside the last
            // pivot of a solvable system.
            let mut ata = [[Scalar::ZERO; 4]; 4];
            let mut atf = [Scalar::ZERO; 4];
            for r in 0..4 {
                for c in 0..4 {
                    let mut sum = Scalar::ZERO;
                    for row in &j {
                        sum = sum.add(row[r].mul(row[c]));
                    }
                    ata[r][c] = if r == c { sum.add(lambda) } else { sum }.sharpen();
                }
                let mut sum = Scalar::ZERO;
                for k in 0..4 {
                    sum = sum.add(j[k][r].mul(f[k]));
                }
                atf[r] = sum.neg();
            }

            let step = solve_linear_system(&Matrix::from_rows(ata), &Vector::from_array(atf))
                .with_context(&ctx)?;
            Ok(Step::Next([
                u_a1.add(step[0]),
                v_a1.add(step[1]),
                u_b1.add(step[2]),
                v_b1.add(step[3]),
            ]))
        },
    )?;
    if let Some(point) = met {
        let [u_a, v_a, u_b, v_b] = corrected.answer;
        return Ok((point, u_a, v_a, u_b, v_b));
    }
    let [u_a1, v_a1, u_b1, v_b1] = corrected.seed;

    Err(ctx(GeopError::new(format!(
        "the corrector did not bring the two surfaces onto a common point within {CORRECTOR_ITERATIONS} iterations: face_a lands at (u={u_a1:?}, v={v_a1:?}) and face_b at (u={u_b1:?}, v={v_b1:?}), which cannot be the same point — at least one of them has left its surface"
    ))))
}

/// The curve of a traced branch whose shape nothing names: a cubic through
/// the marched `points` (each on both surfaces at its `uvs`), widened to
/// enclose the branch between them.
///
/// `widest_between` is told the widest point of the branch located between
/// the marched ones — leg, fraction, point, width — of the last attempt to
/// fit it: the curve is widened to hold it, so one that is wide makes the
/// whole curve wide, and this says which.
pub fn fit_branch(
    surf_a: &NurbSurface3D,
    surf_b: &NurbSurface3D,
    points: &[Vector3],
    uvs: &[OnBoth],
    widest_between: &mut WidestBetween,
    ctx: &dyn Fn(GeopError) -> GeopError,
) -> GeopResult<NurbCurve<4>> {
    // The edge's curve interpolates the marched points, and must
    // enclose the intersection branch *between* them too — a later
    // trace of the same branch from its other end tests its own
    // points against this curve (the duplicate check in
    // a trace's), and they lie on the branch, not on an
    // interpolant.
    //
    // Located with the same corrector the march uses, at a fraction
    // `frac` of the way along the leg from `from` (whose surface
    // parameters are `(u_a, v_a, u_b, v_b)`) to `to`.
    let on_branch = |from: Vector3,
                     (u_a, v_a, u_b, v_b): OnBoth,
                     to: Vector3,
                     frac: Scalar|
     -> GeopResult<(Vector3, OnBoth)> {
        let leg_ctx = with_context!("on_branch(from={from:?}, to={to:?}, frac={frac:?})");
        let chord = to.sub(&from);
        let step = chord.norm().mul(frac);
        let (p, na, va, nb, vb) = predictor_corrector_step(
            surf_a,
            surf_b,
            from,
            chord.normalize().with_context(&leg_ctx)?,
            u_a,
            v_a,
            u_b,
            v_b,
            step,
        )
        .with_context(&leg_ctx)?;
        Ok((p, (na, va, nb, vb)))
    };
    // The march's stride leaves a cubic through its points drifting ~1e-5
    // from the branch; a branch crossed in only a stride or two would be
    // left with too few points for a cubic at all — a single stride,
    // halved, is three, and a quadratic through three points of an ellipse
    // drifts ~4e-6 from it. That drift is enclosed as width, honestly, but
    // only in the directions it points in, and a pcurve fitted by
    // projecting the curve onto a surface tilted against them inherits it
    // as an offset its own width does not cover. And a branch whose
    // curvature runs from gentle to tight along it — the involute flank of
    // a gear tooth crossing a plane, its curvature radius growing tenfold
    // from root to tip — leaves the cubic drifting a hundred times the
    // usual. So the legs are split finer (see `fit_enclosing`).
    let stations: Vec<(Vector3, OnBoth)> =
        points.iter().copied().zip(uvs.iter().copied()).collect();
    NurbCurve::<4>::fit_enclosing(
        &stations,
        |(from, on_both), (to, _), frac| on_branch(*from, *on_both, *to, frac),
        widest_between,
        ctx,
    )
}
