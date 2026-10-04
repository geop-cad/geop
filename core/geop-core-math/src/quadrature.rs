//! [`integrate`]: adaptive Gauss–Kronrod quadrature of a vector-valued
//! function over an interval, in [`Scalar`]s.
//!
//! What it returns is an enclosure in two parts, and the two have different
//! standing:
//!
//! - **The rule's own value is enclosed.** Every node and weight of the
//!   7-point Gauss / 15-point Kronrod pair is taken as the interval between
//!   the two `f64`s around its tabulated value, so it contains the exact
//!   node and weight, and the integrand is evaluated in interval arithmetic
//!   at those intervals. The sum therefore encloses what the exact rule
//!   gives for the integrand — rounding, and any width the integrand's own
//!   data carries, included.
//! - **The rule's truncation error is estimated, not proven.** It is the
//!   difference between the Kronrod and the Gauss value of each panel: the
//!   standard estimate, which for a smooth integrand vastly overestimates
//!   the Kronrod value's error (that is what makes it a usable bound in
//!   practice), but which a rigorous enclosure would replace by a bound on a
//!   high derivative of the integrand — something a NURBS integrand over a
//!   trimmed domain does not offer cheaply. The result is widened by it, so
//!   that width states how well the integral is known; it is honest about
//!   being an estimate in [`Integral::converged`] and in the docs of every
//!   caller, rather than claiming to be proven.
//!
//! [`Quadrature`]'s tolerance and panel budget only decide how hard the
//! search tries: a tighter tolerance gives a narrower result, and a budget
//! run out gives a wider one, never a different answer.

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// The Kronrod nodes in `[0, 1]` of the 15-point rule, largest first; the
/// odd ones (`[1]`, `[3]`, `[5]`, `[7] = 0`) are the 7-point Gauss rule's.
/// QUADPACK's `qk15` tables.
const XGK: [f64; 8] = [
    0.991_455_371_120_812_639_206_854_697_526_329,
    0.949_107_912_342_758_524_526_189_684_047_851,
    0.864_864_423_359_769_072_789_712_788_640_926,
    0.741_531_185_599_394_439_863_864_773_280_788,
    0.586_087_235_467_691_130_294_144_845_693_013,
    0.405_845_151_377_397_166_906_606_412_076_961,
    0.207_784_955_007_898_467_600_689_403_773_245,
    0.0,
];

/// The 15-point Kronrod weights, for the nodes of [`XGK`].
const WGK: [f64; 8] = [
    0.022_935_322_010_529_224_963_732_008_058_970,
    0.063_092_092_629_978_553_290_700_663_189_204,
    0.104_790_010_322_250_183_839_876_322_541_518,
    0.140_653_259_715_525_918_745_189_590_510_238,
    0.169_004_726_639_267_902_826_583_426_598_550,
    0.190_350_578_064_785_409_913_256_402_421_014,
    0.204_432_940_075_298_892_414_161_999_234_649,
    0.209_482_141_084_727_828_012_999_174_891_714,
];

/// The 7-point Gauss weights, for the nodes `XGK[1]`, `XGK[3]`, `XGK[5]`
/// and `XGK[7]`.
const WG: [f64; 4] = [
    0.129_484_966_168_869_693_270_611_432_679_082,
    0.279_705_391_489_276_667_901_467_771_423_780,
    0.381_830_050_505_118_944_950_369_775_488_975,
    0.417_959_183_673_469_387_755_102_040_816_327,
];

/// The interval between the two `f64`s around `x` — `x` itself, if it is
/// zero, which is exact: an enclosure of the real number `x` was rounded
/// from.
fn around<S: Scalar>(x: f64) -> S {
    if x == 0.0 {
        return S::ZERO;
    }
    let step = |up: bool| {
        let bits = x.to_bits();
        f64::from_bits(if (x > 0.0) == up { bits + 1 } else { bits - 1 })
    };
    S::from_f64(step(false)).union(S::from_f64(step(true)))
}

/// How hard [`integrate`] tries: until every component's estimated error
/// is below `relative_tolerance` times the integral of its absolute value,
/// or until it has split the interval into `max_panels` panels.
#[derive(Clone, Copy, Debug)]
pub struct Quadrature {
    pub relative_tolerance: f64,
    pub max_panels: usize,
}

impl Default for Quadrature {
    fn default() -> Self {
        Self {
            relative_tolerance: 1e-10,
            max_panels: 200,
        }
    }
}

/// An integral, as [`integrate`] finds it.
#[derive(Clone, Debug)]
pub struct Integral<S: Scalar> {
    /// Each component's enclosure: the rule's value, enclosed, widened by
    /// its estimated truncation error (see the module docs).
    pub value: Vec<S>,
    /// Whether every component's estimated error came below the tolerance
    /// within the panel budget. When not, `value` is still widened by the
    /// estimate, only wider than asked.
    pub converged: bool,
}

/// One panel of the adaptive search: its ends, the Kronrod value, its
/// estimated error and the integral of the integrand's absolute value, per
/// component.
struct Panel<S: Scalar> {
    a: S,
    b: S,
    kronrod: Vec<S>,
    error: Vec<f64>,
    absolute: Vec<f64>,
    /// How wide the two rules' enclosures are together.
    width: Vec<f64>,
}

/// The 15-point Kronrod and 7-point Gauss rules applied to `f` on `[a, b]`.
fn panel<S: Scalar>(
    f: &mut impl FnMut(S) -> GeopResult<Vec<S>>,
    a: S,
    b: S,
    components: usize,
) -> GeopResult<Panel<S>> {
    let half = b.sub(a).div(S::TWO)?;
    let center = a.add(half);
    let mut kronrod = vec![S::ZERO; components];
    let mut gauss = vec![S::ZERO; components];
    let mut absolute = vec![0.0; components];
    let mut add = |values: &[S], k: usize| -> GeopResult<()> {
        if values.len() != components {
            return Err(GeopError::new(format!(
                "integrate: the integrand gave {} components, {components} expected",
                values.len()
            )));
        }
        let wk = around::<S>(WGK[k]).mul(half);
        // The odd Kronrod nodes are the Gauss rule's.
        let wg = (k % 2 == 1).then(|| around::<S>(WG[k / 2]).mul(half));
        for (c, &value) in values.iter().enumerate() {
            kronrod[c] = kronrod[c].add(wk.mul(value));
            absolute[c] += WGK[k] * half.to_f64().abs() * value.to_f64().abs();
            if let Some(wg) = wg {
                gauss[c] = gauss[c].add(wg.mul(value));
            }
        }
        Ok(())
    };
    for k in 0..XGK.len() {
        if XGK[k] == 0.0 {
            add(&f(center)?, k)?;
            continue;
        }
        let offset = half.mul(around(XGK[k]));
        add(&f(center.sub(offset))?, k)?;
        add(&f(center.add(offset))?, k)?;
    }
    let error = kronrod
        .iter()
        .zip(&gauss)
        .map(|(k, g)| (k.midpoint().to_f64() - g.midpoint().to_f64()).abs())
        .collect();
    let width = kronrod
        .iter()
        .zip(&gauss)
        .map(|(k, g)| k.width().to_f64() + g.width().to_f64())
        .collect();
    Ok(Panel {
        a,
        b,
        kronrod,
        error,
        absolute,
        width,
    })
}

/// The integral of the `components`-valued `f` from `breaks[0]` to the last
/// of `breaks`, each consecutive pair a panel to start with — where the
/// integrand is not smooth, say, at the knots of a spline.
///
/// Global adaptive: the panel whose error is largest, relative to its
/// component's scale, is halved until every component has converged or
/// the budget is spent (see [`Quadrature`]). Where to halve a panel is a
/// free choice, so its midpoint is sharpened.
pub fn integrate<S: Scalar>(
    mut f: impl FnMut(S) -> GeopResult<Vec<S>>,
    breaks: &[S],
    components: usize,
    quadrature: &Quadrature,
) -> GeopResult<Integral<S>> {
    if breaks.len() < 2 {
        return Ok(Integral {
            value: vec![S::ZERO; components],
            converged: true,
        });
    }
    let mut panels = breaks
        .windows(2)
        .map(|w| panel(&mut f, w[0], w[1], components))
        .collect::<GeopResult<Vec<_>>>()?;
    let total = |panels: &[Panel<S>], pick: fn(&Panel<S>) -> &Vec<f64>| -> Vec<f64> {
        (0..components)
            .map(|c| panels.iter().map(|p| pick(p)[c]).sum())
            .collect()
    };
    let converged = loop {
        let scale = total(&panels, |p| &p.absolute);
        let error = total(&panels, |p| &p.error);
        // Converged where the estimate is below the tolerance — or below
        // what the arithmetic resolves at all: a difference between two
        // rules smaller than their own enclosures' widths is rounding, not
        // truncation, and no halving reduces it.
        let width = total(&panels, |p| &p.width);
        let done = (0..components)
            .all(|c| error[c] <= quadrature.relative_tolerance * scale[c] || error[c] <= width[c]);
        if done {
            break true;
        }
        if panels.len() >= quadrature.max_panels {
            break false;
        }
        // The panel contributing most to the component furthest from
        // converging.
        let worst = |p: &Panel<S>| {
            (0..components)
                .filter(|&c| scale[c] > 0.0)
                .map(|c| p.error[c] / scale[c])
                .fold(0.0, f64::max)
        };
        let (index, _) = panels
            .iter()
            .enumerate()
            .map(|(i, p)| (i, worst(p)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .expect("there is a panel");
        let Panel { a, b, .. } = panels.swap_remove(index);
        let mid = a.add(b).div(S::TWO)?.sharpen();
        panels.push(panel(&mut f, a, mid, components)?);
        panels.push(panel(&mut f, mid, b, components)?);
    };
    let error = total(&panels, |p| &p.error);
    let value = (0..components)
        .map(|c| {
            let sum = panels.iter().fold(S::ZERO, |acc, p| acc.add(p.kronrod[c]));
            sum.add(S::from_f64(-error[c]).union(S::from_f64(error[c])))
        })
        .collect();
    Ok(Integral { value, converged })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        for_all_scalars,
        scalars::{Field, Ring, Scalar},
    };

    fn check_polynomials_are_exact<S: Scalar>() {
        // x^5 - 2x^2 + 1 on [0, 2]: 64/6 - 16/3 + 2 = 22/3.
        let f = |x: S| {
            Ok(vec![
                x.mul(x)
                    .mul(x)
                    .mul(x)
                    .mul(x)
                    .sub(S::TWO.mul(x).mul(x))
                    .add(S::ONE),
            ])
        };
        let integral = integrate(f, &[S::ZERO, S::TWO], 1, &Quadrature::default()).unwrap();
        assert!(integral.converged);
        let exact = S::from_ratio(22, 3).unwrap();
        assert!(
            integral.value[0].could_be_equal(exact),
            "{:?}",
            integral.value[0]
        );
        // `ScalInFPA64` is fixed point, at 2^-32, rounding every product.
        assert!(
            integral.value[0].width().to_f64() < 1e-6,
            "{:?}",
            integral.value[0]
        );
    }
    #[test]
    fn polynomials_are_exact() {
        for_all_scalars!(check_polynomials_are_exact);
    }

    /// A function the rule cannot integrate exactly converges by halving
    /// panels, and the result encloses the exact value.
    #[test]
    fn smooth_functions_converge_and_are_enclosed() {
        use crate::scalars::ScalInF64 as S;
        // 1 / (1 + x^2) on [0, 1]: pi / 4.
        let f = |x: S| Ok(vec![S::ONE.div(S::ONE.add(x.mul(x)))?]);
        let integral = integrate(f, &[S::ZERO, S::ONE], 1, &Quadrature::default()).unwrap();
        assert!(integral.converged);
        assert!(integral.value[0].could_be_equal(S::PI.div(S::from_f64(4.0)).unwrap()));
        // sqrt(x) on [0, 1] has an unbounded derivative at 0: it takes
        // halving towards it, and still encloses 2/3.
        let f = |x: S| Ok(vec![x.sqrt()?]);
        let integral = integrate(f, &[S::ZERO, S::ONE], 1, &Quadrature::default()).unwrap();
        assert!(integral.value[0].could_be_equal(S::from_ratio(2, 3).unwrap()));
        assert!(
            integral.value[0].width().to_f64() < 1e-6,
            "{:?}",
            integral.value[0]
        );
    }

    /// A budget too small leaves the result wider, and says so.
    #[test]
    fn a_spent_budget_is_reported() {
        use crate::scalars::ScalInF64 as S;
        let f = |x: S| Ok(vec![x.sqrt()?]);
        let tight = Quadrature {
            relative_tolerance: 1e-14,
            max_panels: 2,
        };
        let integral = integrate(f, &[S::ZERO, S::ONE], 1, &tight).unwrap();
        assert!(!integral.converged);
        assert!(integral.value[0].could_be_equal(S::from_ratio(2, 3).unwrap()));
    }
}
