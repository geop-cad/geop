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
//!
//! [`integrate_polynomial`] is for an integrand known to be a polynomial
//! of bounded degree on every panel. There the Gauss–Legendre rule of
//! enough points is exact, so there is no truncation error to estimate,
//! no halving, and the enclosure is the rule's value alone — proven, and
//! from a fifth of the evaluations a 15-point panel takes.

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// The Kronrod nodes in `[0, 1]` of the 15-point rule, largest first; the
/// odd ones (`[1]`, `[3]`, `[5]`, `[7] = 0`) are the 7-point Gauss rule's.
/// QUADPACK's `qk15` tables.
// The tables as published: `around` encloses the real numbers they name.
#[allow(clippy::excessive_precision)]
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
// The tables as published: `around` encloses the real numbers they name.
#[allow(clippy::excessive_precision)]
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
// The tables as published: `around` encloses the real numbers they name.
#[allow(clippy::excessive_precision)]
const WG: [f64; 4] = [
    0.129_484_966_168_869_693_270_611_432_679_082,
    0.279_705_391_489_276_667_901_467_771_423_780,
    0.381_830_050_505_118_944_950_369_775_488_975,
    0.417_959_183_673_469_387_755_102_040_816_327,
];

/// The Gauss–Legendre rules of 1 to 10 points: each node in `[0, 1]`,
/// largest first, with its weight on `[-1, 1]` — the nodes of the `n`-point
/// rule are these and their negatives, `0` once if `n` is odd. The `n`-point
/// rule integrates polynomials of degree `2n - 1` exactly.
// Computed to 50 digits (mpmath), printed to 36: `around` encloses the real
// numbers they name.
#[allow(clippy::excessive_precision)]
const GAUSS_LEGENDRE: [&[(f64, f64)]; 10] = [
    &[(0.0, 2.0)],
    &[(0.577_350_269_189_625_764_509_148_780_501_957_456, 1.0)],
    &[
        (
            0.774_596_669_241_483_377_035_853_079_956_479_922,
            0.555_555_555_555_555_555_555_555_555_555_555_556,
        ),
        (0.0, 0.888_888_888_888_888_888_888_888_888_888_888_889),
    ],
    &[
        (
            0.861_136_311_594_052_575_223_946_488_892_809_505,
            0.347_854_845_137_453_857_373_063_949_221_999_407,
        ),
        (
            0.339_981_043_584_856_264_802_665_759_103_244_687,
            0.652_145_154_862_546_142_626_936_050_778_000_593,
        ),
    ],
    &[
        (
            0.906_179_845_938_663_992_797_626_878_299_392_965,
            0.236_926_885_056_189_087_514_264_040_719_917_363,
        ),
        (
            0.538_469_310_105_683_091_036_314_420_700_208_805,
            0.478_628_670_499_366_468_041_291_514_835_638_193,
        ),
        (0.0, 0.568_888_888_888_888_888_888_888_888_888_888_889),
    ],
    &[
        (
            0.932_469_514_203_152_027_812_301_554_493_994_609,
            0.171_324_492_379_170_345_040_296_142_172_732_894,
        ),
        (
            0.661_209_386_466_264_513_661_399_595_019_905_347,
            0.360_761_573_048_138_607_569_833_513_837_716_112,
        ),
        (
            0.238_619_186_083_196_908_630_501_721_680_711_935,
            0.467_913_934_572_691_047_389_870_343_989_550_995,
        ),
    ],
    &[
        (
            0.949_107_912_342_758_524_526_189_684_047_851_262,
            0.129_484_966_168_869_693_270_611_432_679_082_018,
        ),
        (
            0.741_531_185_599_394_439_863_864_773_280_788_407,
            0.279_705_391_489_276_667_901_467_771_423_779_582,
        ),
        (
            0.405_845_151_377_397_166_906_606_412_076_961_463,
            0.381_830_050_505_118_944_950_369_775_488_975_134,
        ),
        (0.0, 0.417_959_183_673_469_387_755_102_040_816_326_531),
    ],
    &[
        (
            0.960_289_856_497_536_231_683_560_868_569_472_99,
            0.101_228_536_290_376_259_152_531_354_309_962_19,
        ),
        (
            0.796_666_477_413_626_739_591_553_936_475_830_437,
            0.222_381_034_453_374_470_544_355_994_426_240_884,
        ),
        (
            0.525_532_409_916_328_985_817_739_049_189_246_349,
            0.313_706_645_877_887_287_337_962_201_986_601_313,
        ),
        (
            0.183_434_642_495_649_804_939_476_142_360_183_981,
            0.362_683_783_378_361_982_965_150_449_277_195_612,
        ),
    ],
    &[
        (
            0.968_160_239_507_626_089_835_576_202_903_672_87,
            0.081_274_388_361_574_411_971_892_158_110_523_650_7,
        ),
        (
            0.836_031_107_326_635_794_299_429_788_069_734_877,
            0.180_648_160_694_857_404_058_472_031_242_912_81,
        ),
        (
            0.613_371_432_700_590_397_308_702_039_341_474_185,
            0.260_610_696_402_935_462_318_742_869_418_632_85,
        ),
        (
            0.324_253_423_403_808_929_038_538_014_643_336_609,
            0.312_347_077_040_002_840_068_630_406_584_443_666,
        ),
        (0.0, 0.330_239_355_001_259_763_164_525_069_286_974_049),
    ],
    &[
        (
            0.973_906_528_517_171_720_077_964_012_084_452_053,
            0.066_671_344_308_688_137_593_568_809_893_331_792_9,
        ),
        (
            0.865_063_366_688_984_510_732_096_688_423_493_049,
            0.149_451_349_150_580_593_145_776_339_657_697_332,
        ),
        (
            0.679_409_568_299_024_406_234_327_365_114_873_576,
            0.219_086_362_515_982_043_995_534_934_228_163_192,
        ),
        (
            0.433_395_394_129_247_190_799_265_943_165_784_162,
            0.269_266_719_309_996_355_091_226_921_569_469_353,
        ),
        (
            0.148_874_338_981_631_210_884_826_001_129_719_985,
            0.295_524_224_714_752_870_173_892_994_651_338_329,
        ),
    ],
];

/// The highest degree of a polynomial [`integrate_polynomial`] integrates.
pub const MAX_POLYNOMIAL_DEGREE: usize = 2 * GAUSS_LEGENDRE.len() - 1;

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
    /// How many times the integrand was evaluated: the work it took.
    pub evaluations: usize,
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
    for (k, &node) in XGK.iter().enumerate() {
        if node == 0.0 {
            add(&f(center)?, k)?;
            continue;
        }
        let offset = half.mul(around(node));
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
    let mut evaluations = 0;
    let mut f = |x: S| {
        evaluations += 1;
        f(x)
    };
    if breaks.len() < 2 {
        return Ok(Integral {
            value: vec![S::ZERO; components],
            converged: true,
            evaluations: 0,
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
        let pending: Vec<usize> = (0..components)
            .filter(|&c| error[c] > quadrature.relative_tolerance * scale[c] && error[c] > width[c])
            .collect();
        if pending.is_empty() {
            break true;
        }
        if panels.len() >= quadrature.max_panels {
            break false;
        }
        // The panel contributing most to the component furthest from
        // converging — of those not converged yet. A converged component
        // that is all rounding, its scale nearly zero (the `z` moments of a
        // vertical wall), has relative errors far above any other's, and
        // ranked with the rest it drew every halving to its noise while the
        // panel holding a pending component's error was never halved.
        let worst = |p: &Panel<S>| {
            pending
                .iter()
                .filter(|&&c| scale[c] > 0.0)
                .map(|&c| p.error[c] / scale[c])
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
    Ok(Integral {
        value,
        converged,
        evaluations,
    })
}

/// The integral of the `components`-valued `f` from `breaks[0]` to the last
/// of `breaks`, where `f` is a polynomial of degree at most `degree` between
/// every two consecutive breaks — a spline, say, split at its knots.
///
/// The Gauss–Legendre rule of `degree / 2 + 1` points is exact for such an
/// `f`, so the result is the rule's value, enclosed in interval arithmetic,
/// and nothing else: no truncation error, always converged. Whether `f` is
/// such a polynomial is the caller's claim to make, from what `f` is; a
/// degree above [`MAX_POLYNOMIAL_DEGREE`] is an error.
pub fn integrate_polynomial<S: Scalar>(
    mut f: impl FnMut(S) -> GeopResult<Vec<S>>,
    breaks: &[S],
    components: usize,
    degree: usize,
) -> GeopResult<Integral<S>> {
    let Some(rule) = GAUSS_LEGENDRE.get(degree / 2) else {
        return Err(GeopError::new(format!(
            "integrate_polynomial: no rule for degree {degree}, at most \
             {MAX_POLYNOMIAL_DEGREE}"
        )));
    };
    let mut value = vec![S::ZERO; components];
    let mut evaluations = 0;
    for w in breaks.windows(2) {
        let half = w[1].sub(w[0]).div(S::TWO)?;
        let center = w[0].add(half);
        let mut add = |x: S, weight: f64| -> GeopResult<()> {
            let values = f(x)?;
            evaluations += 1;
            if values.len() != components {
                return Err(GeopError::new(format!(
                    "integrate_polynomial: the integrand gave {} components, {components} \
                     expected",
                    values.len()
                )));
            }
            let weight = around::<S>(weight).mul(half);
            for (sum, v) in value.iter_mut().zip(values) {
                *sum = sum.add(weight.mul(v));
            }
            Ok(())
        };
        for &(node, weight) in rule.iter() {
            if node == 0.0 {
                add(center, weight)?;
                continue;
            }
            let offset = half.mul(around(node));
            add(center.sub(offset), weight)?;
            add(center.add(offset), weight)?;
        }
    }
    Ok(Integral {
        value,
        converged: true,
        evaluations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        for_all_scalars,
        scalars::{Field, Ring, Scalar},
    };

    /// A peak that needs halving, beside a component that is all rounding
    /// — nearly zero, noisy, converged by its width — on the other half of
    /// the range. The noise's relative errors are far above the peak's, and
    /// ranked with them they drew every halving away from the peak: the
    /// moments of a hole's wall did not converge.
    #[test]
    fn halving_follows_the_components_not_converged() {
        type S = crate::scalars::ScalInF64;
        let f = |x: S| {
            let t = x.to_f64();
            let peak = 1.0 / (1.0 + 1e4 * (t + 0.5) * (t + 0.5));
            let noise = if t > 0.0 {
                1e-18 * (1e7 * t).sin()
            } else {
                0.0
            };
            Ok(vec![
                S::from_f64(peak),
                S::from_f64(noise - 1e-15).union(S::from_f64(noise + 1e-15)),
            ])
        };
        let quadrature = Quadrature {
            relative_tolerance: 1e-10,
            max_panels: 64,
        };
        let integral = integrate(f, &[S::from_f64(-1.0), S::ONE], 2, &quadrature).unwrap();
        assert!(integral.converged, "{integral:?}");
        // ∫ 1 / (1 + 10⁴ (x + ½)²) = (atan(50) + atan(150)) / 100.
        let exact = (50.0_f64.atan() + 150.0_f64.atan()) / 100.0;
        assert!(
            integral.value[0].could_be_equal(S::from_f64(exact)),
            "{integral:?}"
        );
    }

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

    /// Every rule integrates every power up to its degree exactly — each
    /// table entry checked — over panels of a split range, with `n` points
    /// a panel.
    fn check_gauss_legendre_is_exact<S: Scalar>() {
        for degree in 0..=MAX_POLYNOMIAL_DEGREE {
            let f = |x: S| {
                Ok((0..=degree)
                    .map(|k| (0..k).fold(S::ONE, |p, _| p.mul(x)))
                    .collect())
            };
            let breaks = [S::from_f64(-1.0), S::from_f64(0.25), S::ONE];
            let integral = integrate_polynomial(f, &breaks, degree + 1, degree).unwrap();
            assert!(integral.converged);
            assert_eq!(integral.evaluations, 2 * (degree / 2 + 1));
            for (k, value) in integral.value.iter().enumerate() {
                // ∫ x^k from -1 to 1: 2 / (k + 1) for even k, 0 for odd.
                let exact = if k % 2 == 0 {
                    S::from_ratio(2, k as i64 + 1).unwrap()
                } else {
                    S::ZERO
                };
                assert!(
                    value.could_be_equal(exact),
                    "x^{k}, degree {degree}: {value:?}"
                );
                assert!(value.width().to_f64() < 1e-6, "x^{k}: {value:?}");
            }
        }
        let too_high = integrate_polynomial(
            |_: S| Ok(vec![S::ONE]),
            &[S::ZERO, S::ONE],
            1,
            MAX_POLYNOMIAL_DEGREE + 1,
        );
        assert!(too_high.is_err());
    }
    #[test]
    fn gauss_legendre_is_exact() {
        for_all_scalars!(check_gauss_legendre_is_exact);
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
