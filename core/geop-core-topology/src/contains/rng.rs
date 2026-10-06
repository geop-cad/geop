//! A tiny, seedable, deterministic PRNG for picking random ray directions in
//! containment queries — no external `rand` dependency.

use geop_core_math::{
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

/// xorshift64* generator.
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        // xorshift64* requires a nonzero state.
        Self {
            state: if seed == 0 { 0x9E3779B97F4A7C15 } else { seed },
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform float in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform float in `[lo, hi)`.
    pub fn next_range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }

    /// A random direction, uniformly distributed on the unit circle.
    pub fn next_direction2<S: Scalar>(&mut self) -> Vector2<S> {
        let theta = self.next_range(0.0, std::f64::consts::TAU);
        Vector2::from_array([S::from_f64(theta.cos()), S::from_f64(theta.sin())])
    }

    /// A random direction, uniformly distributed on the unit sphere
    /// (Archimedes' cylindrical projection: uniform `z`, uniform angle).
    pub fn next_direction3<S: Scalar>(&mut self) -> Vector3<S> {
        let z = self.next_range(-1.0, 1.0);
        let theta = self.next_range(0.0, std::f64::consts::TAU);
        let r = (1.0 - z * z).max(0.0).sqrt();
        Vector3::from_array([
            S::from_f64(r * theta.cos()),
            S::from_f64(r * theta.sin()),
            S::from_f64(z),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::Rng;

    #[test]
    fn same_seed_is_deterministic() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn f64_stays_in_unit_range() {
        let mut r = Rng::new(7);
        for _ in 0..1000 {
            let x = r.next_f64();
            assert!((0.0..1.0).contains(&x));
        }
    }
}
