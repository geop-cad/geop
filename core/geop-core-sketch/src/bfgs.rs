//! A dense BFGS minimizer with a backtracking (Armijo) line search.
//!
//! Sketches have at most a few hundred variables, so the inverse Hessian
//! approximation is kept as a full `n x n` matrix.

/// When [`minimize`] stops.
#[derive(Clone, Copy, Debug)]
pub struct BfgsOptions {
    pub max_iterations: usize,
    /// Stop once the objective is at or below this value.
    pub f_tolerance: f64,
    /// Stop once every gradient component is at or below this magnitude.
    pub g_tolerance: f64,
}

#[derive(Clone, Debug)]
pub struct BfgsResult {
    pub x: Vec<f64>,
    pub f: f64,
    pub iterations: usize,
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Minimize `f` from `x0`. `f` returns the objective and its gradient.
///
/// Always returns the best point found; whether that point is good enough is
/// the caller's question (it knows what the objective means), which is why
/// this reports no "converged" flag of its own.
pub fn minimize(
    f: impl Fn(&[f64]) -> (f64, Vec<f64>),
    x0: Vec<f64>,
    options: BfgsOptions,
) -> BfgsResult {
    let n = x0.len();
    let mut x = x0;
    let (mut fx, mut g) = f(&x);
    // Inverse Hessian approximation, row-major. Starts as the identity and is
    // rescaled after the first accepted step (Nocedal & Wright, eq. 6.20).
    let mut h = vec![0.0; n * n];
    for i in 0..n {
        h[i * n + i] = 1.0;
    }
    let mut first_update = true;

    for iteration in 0..options.max_iterations {
        if fx <= options.f_tolerance || g.iter().all(|gi| gi.abs() <= options.g_tolerance) {
            return BfgsResult {
                x,
                f: fx,
                iterations: iteration,
            };
        }

        // Search direction p = -H g; fall back to steepest descent if the
        // approximation has stopped producing a descent direction.
        let mut p: Vec<f64> = (0..n).map(|i| -dot(&h[i * n..(i + 1) * n], &g)).collect();
        let mut slope = dot(&p, &g);
        if slope >= 0.0 {
            p = g.iter().map(|gi| -gi).collect();
            slope = dot(&p, &g);
            h.iter_mut().for_each(|v| *v = 0.0);
            for i in 0..n {
                h[i * n + i] = 1.0;
            }
            first_update = true;
        }

        // Backtracking line search on the Armijo condition.
        let mut alpha = 1.0;
        let mut accepted = None;
        for _ in 0..60 {
            let x_new: Vec<f64> = x.iter().zip(&p).map(|(xi, pi)| xi + alpha * pi).collect();
            let (f_new, g_new) = f(&x_new);
            if f_new.is_finite() && f_new <= fx + 1e-4 * alpha * slope {
                accepted = Some((x_new, f_new, g_new));
                break;
            }
            alpha *= 0.5;
        }
        let Some((x_new, f_new, g_new)) = accepted else {
            // No decrease along a descent direction: we are at the limit of
            // what floating point can resolve.
            return BfgsResult {
                x,
                f: fx,
                iterations: iteration,
            };
        };

        let s: Vec<f64> = x_new.iter().zip(&x).map(|(a, b)| a - b).collect();
        let y: Vec<f64> = g_new.iter().zip(&g).map(|(a, b)| a - b).collect();
        let sy = dot(&s, &y);
        // Skip the update unless it keeps H positive definite.
        if sy > 1e-12 * dot(&s, &s).sqrt() * dot(&y, &y).sqrt() && sy > 0.0 {
            if first_update {
                let scale = sy / dot(&y, &y);
                h.iter_mut().for_each(|v| *v = 0.0);
                for i in 0..n {
                    h[i * n + i] = scale;
                }
                first_update = false;
            }
            // H <- (I - rho s y^T) H (I - rho y s^T) + rho s s^T
            let rho = 1.0 / sy;
            let hy: Vec<f64> = (0..n).map(|i| dot(&h[i * n..(i + 1) * n], &y)).collect();
            let yhy = dot(&y, &hy);
            for i in 0..n {
                for j in 0..n {
                    h[i * n + j] += -rho * (hy[i] * s[j] + s[i] * hy[j])
                        + (rho * rho * yhy + rho) * s[i] * s[j];
                }
            }
        }

        x = x_new;
        fx = f_new;
        g = g_new;
    }

    BfgsResult {
        x,
        f: fx,
        iterations: options.max_iterations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minimizes_rosenbrock() {
        let rosenbrock = |x: &[f64]| {
            let (a, b) = (x[0], x[1]);
            let f = (1.0 - a).powi(2) + 100.0 * (b - a * a).powi(2);
            let g = vec![
                -2.0 * (1.0 - a) - 400.0 * a * (b - a * a),
                200.0 * (b - a * a),
            ];
            (f, g)
        };
        let r = minimize(
            rosenbrock,
            vec![-1.2, 1.0],
            BfgsOptions {
                max_iterations: 500,
                f_tolerance: 1e-20,
                g_tolerance: 1e-12,
            },
        );
        assert!(
            (r.x[0] - 1.0).abs() < 1e-6 && (r.x[1] - 1.0).abs() < 1e-6,
            "{r:?}"
        );
    }
}
