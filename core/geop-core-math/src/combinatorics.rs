//! Counting, exactly in integers.

/// `n` choose `k`, exactly, for the small `n` of a polynomial's degree:
/// each partial product is itself a binomial coefficient, so every division
/// is exact.
pub fn binomial(n: usize, k: usize) -> i64 {
    (0..k).fold(1i64, |acc, i| acc * (n - i) as i64 / (i as i64 + 1))
}
