//! Maps of space applied to geometry: a rigid motion, a mirror or a
//! scaling ([`Motion`]), and the deformations of a body about an axis —
//! tapering (or pinching, one way across the axis), shearing, twisting and
//! bending ([`AxialMap`]).
//!
//! # One mechanism: composition on the control points
//!
//! In a frame whose third axis is the deformation's axis, an axial map
//! takes the point `(x, y, t)` to `x a(t) + y b(t) + c(t)`: affine in `x`
//! and `y`, with coefficients that are functions of `t` alone. A taper
//! scales `x` and `y` by a factor linear in `t`; a twist turns them by an
//! angle linear in `t`; a shear moves them across by an offset linear in
//! `t`; a bend carries the plane `t = const` round a
//! circle, `x` measured towards its centre. Each coefficient is
//! `constant + slope s + cos·cos(rate s) + sin·sin(rate s)` with `s` the
//! distance along the axis from the zone's start ([`AxialFunction`]).
//!
//! On a Bézier piece of a NURBS, `x`, `y` and `t` are polynomials over the
//! weight `W`. A coefficient that is a polynomial of degree `d` in `t`,
//! written about the piece's middle `tc` as `Σ f_n (t - tc)^n`, composes
//! into `Σ f_n U^n W^(d-n) / W^d` with `U = T - tc W`: so the image of the
//! piece is the rational Bézier piece of numerator `Σ_i e_i (Â_i x̂ + B̂_i
//! ŷ + Ĉ_i W)` and weight `W^(d+1)`, of `d + 1` times the degree — computed
//! from the control points by products of Bernstein polynomials
//! ([`crate::bernstein`]), no evaluation, no fitting. The parametrization
//! is kept, so every pcurve on a deformed face still traces it.
//!
//! A taper's coefficients are linear: the composition is exact. A twist's
//! and a bend's are trigonometric: each is replaced by its cubic
//! interpolant at Chebyshev points over the piece's reach along the axis,
//! whose error, `(|cos| + |sin|) |rate|^4 / 4! · max |Π (τ - τ_j)|`, is a
//! proven bound; times what `x` and `y` reach over the piece, it bounds how
//! far the composed piece can be from the true image, and every control
//! point is widened by it (a polynomial piece stays polynomial, its weights
//! exactly one). A piece whose bound is above the target is halved (along
//! the direction it runs most along the axis) until it is below, and a
//! curve or surface that would need more than `MAX_PIECES` pieces is
//! refused by name. The pieces are joined back at knots of full
//! multiplicity, the shared control points the union of the two
//! enclosures.
//!
//! # Zones
//!
//! The law holds over the zone `[start, end]` along the axis. Before and
//! after it the map continues as the affine map it is tangent to at the
//! zone's end ([`Motion::affine`]): rigidly for a twist or a bend, as a
//! constant scaling across the axis for a taper. So a body is first divided
//! where it crosses the zone's ends (the operation imprints those planes),
//! and each piece of it is moved by the piece of the map it lies in. The
//! two agree on the plane they share.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use crate::{bernstein::Bernstein, nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};

/// A map of space as it moves geometry: points, curves and surfaces, each
/// keeping its parametrization, so a pcurve on a moved face still traces
/// it. `at` gives a point of the curve or of the face the surface bounds,
/// asked for only by a map made of pieces where the geometry's control
/// points straddle two of them: the piece that holds `at` moves it.
pub trait Deformation {
    /// The point `p` moved.
    fn point(&self, p: &Vector3) -> GeopResult<Vector3>;
    /// The curve `curve` moved.
    fn curve(
        &self,
        curve: &NurbCurve3D,
        at: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbCurve3D>;
    /// The surface `surface` moved.
    fn surface(
        &self,
        surface: &NurbSurface3D,
        at: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbSurface3D>;
    /// Whether it turns the side a surface's normal points to: a mirror.
    fn mirrors(&self) -> bool;
    /// Whether it is affine, mapping control points to control points.
    /// One that is not costs by the size of what it maps, so a face's
    /// surface is first cut down to what the face uses of it.
    fn affine(&self) -> bool;
}

impl Deformation for Motion {
    fn point(&self, p: &Vector3) -> GeopResult<Vector3> {
        Ok(self.apply(p))
    }

    fn curve(
        &self,
        curve: &NurbCurve3D,
        _: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbCurve3D> {
        Ok(curve.transform(self))
    }

    fn surface(
        &self,
        surface: &NurbSurface3D,
        _: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbSurface3D> {
        Ok(surface.transform(self))
    }

    fn mirrors(&self) -> bool {
        Motion::mirrors(self)
    }

    fn affine(&self) -> bool {
        true
    }
}

/// `constant + slope s + cos·cos(rate s) + sin·sin(rate s)`, with `s = t -
/// start`: a coefficient of an [`AxialMap`] as a function of the
/// coordinate `t` along its axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxialFunction {
    constant: Scalar,
    slope: Scalar,
    cos: Scalar,
    sin: Scalar,
    rate: Scalar,
    start: Scalar,
}

impl AxialFunction {
    const ZERO: Self = AxialFunction {
        constant: Scalar::ZERO,
        slope: Scalar::ZERO,
        cos: Scalar::ZERO,
        sin: Scalar::ZERO,
        rate: Scalar::ZERO,
        start: Scalar::ZERO,
    };

    /// `constant + slope (t - start)`.
    fn linear(constant: Scalar, slope: Scalar, start: Scalar) -> Self {
        AxialFunction {
            constant,
            slope,
            start,
            ..Self::ZERO
        }
    }

    /// `constant + cos·cos(rate s) + sin·sin(rate s)`, `s = t - start`.
    fn trig(constant: Scalar, cos: Scalar, sin: Scalar, rate: Scalar, start: Scalar) -> Self {
        AxialFunction {
            constant,
            cos,
            sin,
            rate,
            start,
            ..Self::ZERO
        }
    }

    fn is_zero(&self) -> bool {
        [self.constant, self.slope, self.cos, self.sin]
            .iter()
            .all(|c| c.is_exactly(Scalar::ZERO))
    }

    /// Whether it has a trigonometric part: a polynomial of no degree.
    fn is_trig(&self) -> bool {
        !(self.cos.is_exactly(Scalar::ZERO) && self.sin.is_exactly(Scalar::ZERO))
    }

    /// The value at `t`, an enclosure for an interval `t`.
    fn evaluate(&self, t: Scalar) -> Scalar {
        let s = t.sub(self.start);
        let mut value = self.constant;
        if !self.slope.is_exactly(Scalar::ZERO) {
            value = value.add(self.slope.mul(s));
        }
        if self.is_trig() {
            let phase = self.rate.mul(s);
            value = value
                .add(self.cos.mul(phase.cos()))
                .add(self.sin.mul(phase.sin()));
        }
        value
    }

    /// The derivative along the axis.
    fn derivative(&self) -> Self {
        AxialFunction {
            constant: self.slope,
            slope: Scalar::ZERO,
            cos: self.rate.mul(self.sin),
            sin: self.rate.mul(self.cos).neg(),
            rate: self.rate,
            start: self.start,
        }
    }

    /// A polynomial of degree `degree` (at least one) standing in for the
    /// function within `reach` of `middle`, as its coefficients of powers
    /// of `s = (t - middle) / reach` (of nothing but the constant where
    /// `reach` is zero), and an enclosure of how far the function can be
    /// from it there. The linear part is itself: exact. The trigonometric
    /// one is interpolated at Chebyshev points `s_j`, whose error is
    /// bounded by `max |d^(d+1) f / ds^(d+1)| / (d+1)! · max |Π (s -
    /// s_j)|` (the remainder of interpolation, for any nodes): the
    /// derivative by `(|cos| + |sin|) (|rate| reach)^(d+1)`, the node
    /// polynomial by the largest of its Bernstein coefficients over pieces
    /// of `[-1, 1]`. The nodes are a free choice, so they are sharp. In `s`
    /// nothing is divided by the reach, which may be as small as rounding.
    fn interpolant(
        &self,
        middle: Scalar,
        reach: Scalar,
        degree: usize,
    ) -> GeopResult<(Vec<Scalar>, Scalar)> {
        let mut coefficients = vec![self.evaluate(middle)];
        if reach.is_exactly(Scalar::ZERO) {
            coefficients.resize(degree + 1, Scalar::ZERO);
            return Ok((coefficients, Scalar::ZERO));
        }
        if !self.is_trig() {
            coefficients.push(self.slope.mul(reach));
            coefficients.resize(degree + 1, Scalar::ZERO);
            return Ok((coefficients, Scalar::ZERO));
        }
        let nodes: Vec<Scalar> = (0..=degree)
            .map(|j| {
                let angle = std::f64::consts::PI * (2 * j + 1) as f64 / (2 * (degree + 1)) as f64;
                Scalar::from_f64(angle.cos())
            })
            .collect();
        // Newton's divided differences, then its form multiplied out.
        let mut differences: Vec<Scalar> = (nodes.iter())
            .map(|&n| self.evaluate(middle.add(reach.mul(n))))
            .collect();
        for level in 1..=degree {
            for j in (level..=degree).rev() {
                let step = nodes[j].sub(nodes[j - level]);
                differences[j] = differences[j].sub(differences[j - 1]).div(step)?;
            }
        }
        let mut coefficients = vec![differences[degree]];
        for j in (0..degree).rev() {
            // coefficients · (s - s_j) + differences[j].
            let mut next = vec![Scalar::ZERO; coefficients.len() + 1];
            for (n, &c) in coefficients.iter().enumerate() {
                next[n + 1] = next[n + 1].add(c);
                next[n] = next[n].sub(c.mul(nodes[j]));
            }
            next[0] = next[0].add(differences[j]);
            coefficients = next;
        }
        // The node polynomial's largest value over [-1, 1]: the largest
        // Bernstein coefficient over each of a few pieces of it (over the
        // whole, they stand far above its oscillation).
        const PIECES: i64 = 16;
        let mut nodal = Scalar::ZERO;
        for k in 0..PIECES {
            let lo = Scalar::from_ratio(2 * k - PIECES, PIECES)?;
            let hi = Scalar::from_ratio(2 * k + 2 - PIECES, PIECES)?;
            let mut piece = Bernstein::constant(Scalar::ONE);
            for &n in &nodes {
                piece = piece.product(&Bernstein::new([1, 0], vec![lo.sub(n), hi.sub(n)]))?;
            }
            nodal = (piece.coefficients().iter()).fold(nodal, |m, c| m.max(c.abs()));
        }
        let step = self.rate.abs().mul(reach.abs());
        let mut bound = self.cos.abs().add(self.sin.abs()).mul(nodal);
        for n in 1..=degree + 1 {
            bound = bound.mul(step).div(Scalar::from_i64(n as i64))?;
        }
        let bound = bound.upper().to_f64();
        Ok((coefficients, Scalar::new(-bound, bound)))
    }
}

/// Which piece of an [`AxialMap`] moves a point: the affine map before the
/// zone, the law over it, the affine map after it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Piece {
    Before,
    Law,
    After,
}

/// The most pieces one curve or surface is cut into to bring its image
/// within the target: past it, the map is refused for that geometry.
const MAX_PIECES: usize = 4096;

/// How close to the true image a composed piece has to be, relative to
/// its size and its distance from the axis: a target for how finely to
/// cut, not a tolerance any answer relies on — whatever the bound comes to
/// widens the piece.
const RELATIVE_TARGET: f64 = 1e-9;

/// A taper, a twist or a bend about an axis (see the module's doc): in the
/// frame `frame` at `origin` — `x`, `y` and the axis, orthonormal and
/// right-handed — the point `(x, y, t)` goes to `x a(t) + y b(t) + c(t)`
/// over the zone `start ≤ t ≤ end`, `columns` being `[a, b, c]` by their
/// components in the frame. Before and after the zone, the affine map the
/// law is tangent to at its end.
#[derive(Clone, Debug, PartialEq)]
pub struct AxialMap {
    origin: Vector3,
    frame: [Vector3; 3],
    start: Scalar,
    end: Scalar,
    columns: [[AxialFunction; 3]; 3],
}

impl AxialMap {
    /// The frame `x`, `y`, `axis` , `x` along `toward` (square
    /// to the axis) or, without it, chosen by [`Vector3::across`]. Fails
    /// for an axis of no length, a `toward` along it, or a zone that does
    /// not run definitely forward.
    fn frame(
        axis: &Vector3,
        toward: Option<&Vector3>,
        (start, end): (Scalar, Scalar),
    ) -> GeopResult<([Vector3; 3], Scalar, Scalar)> {
        if !end.definitely_greater(start) {
            return Err(GeopError::new(format!(
                "AxialMap: the zone from {start:?} to {end:?} along the axis does not run forward"
            )));
        }
        // A unit axis is kept as it is: dividing by a norm of exactly one
        // would only widen it.
        let unit = |v: &Vector3| {
            if v.norm_sq().is_exactly(Scalar::ONE) {
                Ok(*v)
            } else {
                v.normalize()
            }
        };
        let t = unit(axis).map_err(|e| e.with_context(format!("AxialMap: the axis {axis:?}")))?;
        let x = match toward {
            None => t.across()?[0],
            Some(toward) => {
                let square = toward.sub(&t.prod_scalar(toward.prod_dot(&t)));
                unit(&square).map_err(|e| {
                    e.with_context(format!(
                        "AxialMap: {toward:?} runs along the axis {axis:?}, so it gives no direction to bend towards"
                    ))
                })?
            }
        };
        let y = t.prod_cross(&x);
        Ok(([x, y, t], start, end))
    }

    /// Tapering about the axis through `origin` along `axis`: across the
    /// axis, scaled by one at `start` and by `factor` at `end`, linearly in
    /// between, and by `factor` past the end — or, pinching, only along
    /// `toward` (square to the axis), the other way across left as it is.
    /// Exact. Fails for a factor that is not definitely positive.
    pub fn taper(
        origin: Vector3,
        axis: &Vector3,
        toward: Option<&Vector3>,
        zone: (Scalar, Scalar),
        factor: Scalar,
    ) -> GeopResult<Self> {
        if !factor.definitely_greater(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "AxialMap::taper: a factor of {factor:?} turns the body inside out or flattens it; it has to be positive"
            )));
        }
        let (frame, start, end) = Self::frame(axis, toward, zone)?;
        let slope = factor.sub(Scalar::ONE).div(end.sub(start))?;
        let scale = AxialFunction::linear(Scalar::ONE, slope, start);
        let zero = AxialFunction::ZERO;
        let across = match toward {
            None => scale,
            Some(_) => AxialFunction::linear(Scalar::ONE, Scalar::ZERO, start),
        };
        let along = AxialFunction::linear(start, Scalar::ONE, start);
        Ok(AxialMap {
            origin,
            frame,
            start,
            end,
            columns: [
                [scale, zero, zero],
                [zero, across, zero],
                [zero, zero, along],
            ],
        })
    }

    /// Shearing along the axis through `origin` along `axis`: each plane
    /// across the axis moved towards `toward` (square to the axis) by
    /// `offset` times its distance from `start` over the zone's length.
    /// Affine, so the same map before, over and past the zone: the zone
    /// only sets the rate ([`AxialMap::as_motion`]).
    pub fn shear(
        origin: Vector3,
        axis: &Vector3,
        toward: &Vector3,
        zone: (Scalar, Scalar),
        offset: Scalar,
    ) -> GeopResult<Self> {
        let (frame, start, end) = Self::frame(axis, Some(toward), zone)?;
        let (zero, one, s0) = (Scalar::ZERO, Scalar::ONE, AxialFunction::ZERO);
        let unit = AxialFunction::linear(one, zero, start);
        let across = AxialFunction::linear(zero, offset.div(end.sub(start))?, start);
        let along = AxialFunction::linear(start, one, start);
        Ok(AxialMap {
            origin,
            frame,
            start,
            end,
            columns: [[unit, s0, s0], [s0, unit, s0], [across, s0, along]],
        })
    }

    /// The affine map it is, where it is one everywhere — a shear: its
    /// columns across the axis constant and its offset linear along it, so
    /// the law is the map it is tangent to at either end. None for a taper,
    /// a twist or a bend.
    pub fn as_motion(&self) -> GeopResult<Option<Motion>> {
        let constant = |f: &AxialFunction| !f.is_trig() && f.slope.is_exactly(Scalar::ZERO);
        let linear = |f: &AxialFunction| !f.is_trig();
        let affine = self.columns[0].iter().all(constant)
            && self.columns[1].iter().all(constant)
            && self.columns[2].iter().all(linear);
        if !affine {
            return Ok(None);
        }
        self.tangent_motion(self.start).map(Some)
    }

    /// Twisting about the axis through `origin` along `axis`: turned by
    /// nothing at `start` and by `angle` (radians, right-handed about the
    /// axis) at `end`, uniformly in between, rigidly by `angle` past the
    /// end. Either hand.
    pub fn twist(
        origin: Vector3,
        axis: &Vector3,
        zone: (Scalar, Scalar),
        angle: Scalar,
    ) -> GeopResult<Self> {
        let (frame, start, end) = Self::frame(axis, None, zone)?;
        let rate = angle.div(end.sub(start))?;
        let one = Scalar::ONE;
        let (zero, s0) = (Scalar::ZERO, AxialFunction::ZERO);
        let cos = AxialFunction::trig(zero, one, zero, rate, start);
        let sin = AxialFunction::trig(zero, zero, one, rate, start);
        let minus_sin = AxialFunction::trig(zero, zero, one.neg(), rate, start);
        let along = AxialFunction::linear(start, one, start);
        Ok(AxialMap {
            origin,
            frame,
            start,
            end,
            columns: [[cos, sin, s0], [minus_sin, cos, s0], [s0, s0, along]],
        })
    }

    /// Bending about an axis square to the line through `origin` along
    /// `axis`: the stretch of that line from `start` to `end` is wound onto
    /// a circular arc of `angle` (radians), curving towards `toward`, its
    /// length kept — the radius is the zone's length over the angle. The
    /// plane square to the line at each point stays a plane, turned with
    /// it; past the end everything is carried on rigidly. A negative angle
    /// curves away from `toward`, an angle of nothing leaves everything
    /// where it is. Fails for a turn of a whole circle or more, which would
    /// lay the zone over itself.
    pub fn bend(
        origin: Vector3,
        axis: &Vector3,
        toward: &Vector3,
        zone: (Scalar, Scalar),
        angle: Scalar,
    ) -> GeopResult<Self> {
        let (frame, start, end) = Self::frame(axis, Some(toward), zone)?;
        let full = Scalar::PI.mul(Scalar::TWO);
        if angle.abs().could_be_greater(full) || angle.abs().could_be_equal(full) {
            return Err(GeopError::new(format!(
                "AxialMap::bend: a bend of {angle:?} radians turns a whole circle or more, laying the zone over itself"
            )));
        }
        let (zero, one, s0) = (Scalar::ZERO, Scalar::ONE, AxialFunction::ZERO);
        let along = AxialFunction::linear(start, one, start);
        let unit = AxialFunction::linear(one, zero, start);
        let columns = if angle.is_exactly(zero) {
            [[unit, s0, s0], [s0, unit, s0], [s0, s0, along]]
        } else {
            let rate = angle.div(end.sub(start))?;
            let radius = end.sub(start).div(angle)?;
            let cos = AxialFunction::trig(zero, one, zero, rate, start);
            let minus_sin = AxialFunction::trig(zero, zero, one.neg(), rate, start);
            // c = (R (1 - cos), 0, start + R sin).
            let cx = AxialFunction::trig(radius, radius.neg(), zero, rate, start);
            let ct = AxialFunction::trig(start, zero, radius, rate, start);
            [[cos, s0, minus_sin], [s0, unit, s0], [cx, s0, ct]]
        };
        Ok(AxialMap {
            origin,
            frame,
            start,
            end,
            columns,
        })
    }

    /// Where the zone starts and ends along the axis.
    pub fn zone(&self) -> (Scalar, Scalar) {
        (self.start, self.end)
    }

    /// How far along the axis `p` lies: its coordinate `t`.
    pub fn along(&self, p: &Vector3) -> Scalar {
        self.local(p)[2]
    }

    /// The square of the plane across the axis at `t` along it, reaching
    /// `reach` from the axis each way: what divides a body where it crosses
    /// from one piece of the map into the next.
    pub fn plane_across(&self, t: Scalar, reach: Scalar) -> GeopResult<NurbSurface3D> {
        let [x, y, axis] = &self.frame;
        let center = self.origin.add(&axis.prod_scalar(t));
        let corner = |sx: Scalar, sy: Scalar| {
            center
                .add(&x.prod_scalar(sx.mul(reach)))
                .add(&y.prod_scalar(sy.mul(reach)))
        };
        let (minus, plus) = (Scalar::ONE.neg(), Scalar::ONE);
        NurbSurface3D::bilinear(
            corner(minus, minus),
            corner(plus, minus),
            corner(plus, plus),
            corner(minus, plus),
        )
    }

    /// The coordinates of `p` in the frame: `(x, y, t)`.
    fn local(&self, p: &Vector3) -> [Scalar; 3] {
        let q = p.sub(&self.origin);
        self.frame.map(|e| e.prod_dot(&q))
    }

    /// The point with coordinates `local` in the frame, relative to the
    /// origin: `Σ local_i e_i`.
    fn world(&self, local: [Scalar; 3]) -> Vector3 {
        (self.frame.iter().zip(local))
            .map(|(e, c)| e.prod_scalar(c))
            .reduce(|a, b| a.add(&b))
            .expect("three axes")
    }

    /// The pieces of the map a coordinate `t` along the axis could fall in.
    fn pieces(&self, t: Scalar) -> Vec<Piece> {
        let mut pieces = Vec::new();
        if t.could_be_less(self.start) {
            pieces.push(Piece::Before);
        }
        if !(t.definitely_less(self.start) || t.definitely_greater(self.end)) {
            pieces.push(Piece::Law);
        }
        if t.could_be_greater(self.end) {
            pieces.push(Piece::After);
        }
        pieces
    }

    /// The affine map the law is tangent to at `t`, a zone's end:
    /// `x a(t) + y b(t) + c(t) + (s - t) c'(t)` for the point at `s` along
    /// the axis.
    fn tangent_motion(&self, t: Scalar) -> GeopResult<Motion> {
        let at = |column: &[AxialFunction; 3]| column.map(|f| f.evaluate(t));
        let [a, b, c] = [
            at(&self.columns[0]),
            at(&self.columns[1]),
            at(&self.columns[2]),
        ];
        let dc = self.columns[2].map(|f| f.derivative().evaluate(t));
        let [ex, ey, et] = &self.frame;
        // Column k of the linear part: the image of the world's axis k,
        // whose coordinates in the frame are (ex[k], ey[k], et[k]).
        let columns = std::array::from_fn(|k| {
            self.world(std::array::from_fn(|i| {
                a[i].mul(ex[k]).add(b[i].mul(ey[k])).add(dc[i].mul(et[k]))
            }))
        });
        let linear = |p: &Vector3| -> Vector3 {
            (columns.iter().zip(p.to_array()))
                .map(|(col, pk): (&Vector3, Scalar)| col.prod_scalar(pk))
                .reduce(|x, y| x.add(&y))
                .expect("three columns")
        };
        let offset = self.world(std::array::from_fn(|i| c[i].sub(t.mul(dc[i]))));
        let position = self.origin.add(&offset).sub(&linear(&self.origin));
        Motion::affine(columns, position)
    }

    /// The law at the point with coordinates `(x, y, t)`, `t` in the zone.
    fn law_point(&self, [x, y, t]: [Scalar; 3]) -> Vector3 {
        let local = std::array::from_fn(|i| {
            let [a, b, c] = [0, 1, 2].map(|col| self.columns[col][i]);
            let mut v = c.evaluate(t);
            if !a.is_zero() {
                v = v.add(x.mul(a.evaluate(t)));
            }
            if !b.is_zero() {
                v = v.add(y.mul(b.evaluate(t)));
            }
            v
        });
        self.origin.add(&self.world(local))
    }

    /// The degree of the polynomials the law's coefficients are replaced
    /// by: one where all are linear (then exactly); three where one is
    /// trigonometric, so a plane's image is of degree four, whose moments
    /// are still integrated exactly (`Model::mass_properties`).
    fn interpolant_degree(&self) -> usize {
        if self.columns.iter().flatten().any(AxialFunction::is_trig) {
            3
        } else {
            1
        }
    }

    /// Which piece moves geometry whose control points are `points`: the
    /// one their coordinates along the axis fall in, or, where they
    /// straddle a zone's end, the one `at` falls in — the law where that is
    /// on the end itself, where the law and the affine map agree.
    fn piece_of(
        &self,
        points: &[Vector4],
        at: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<Piece> {
        let t = (points.iter())
            .map(|p| Ok(self.local(&p.dehomogenized()?)[2]))
            .collect::<GeopResult<Vec<_>>>()?;
        if let Some(piece) = self.piece_holding(&t) {
            return Ok(piece);
        }
        let t = self.local(&at()?)[2];
        Ok(self.piece_holding(&[t]).unwrap_or(Piece::Law))
    }

    /// The piece that holds every one of `t`, coordinates along the axis,
    /// up to its ends: the law where each could be within the zone, its
    /// ends included, where the law and the affine maps agree; an affine
    /// map where each could be on that one's side. None where they reach
    /// definitely into two. Each coordinate on its own, so a point on an
    /// end that rounding has widened past it still counts as on it.
    fn piece_holding(&self, t: &[Scalar]) -> Option<Piece> {
        let all = |f: &dyn Fn(Scalar) -> bool| t.iter().all(|&t| f(t));
        if all(&|t| !t.definitely_less(self.start) && !t.definitely_greater(self.end)) {
            Some(Piece::Law)
        } else if all(&|t| !t.definitely_greater(self.start)) {
            Some(Piece::Before)
        } else if all(&|t| !t.definitely_less(self.end)) {
            Some(Piece::After)
        } else {
            None
        }
    }

    /// The law applied to one Bézier piece, given by the Bernstein
    /// polynomials of its homogeneous coordinates `(X, Y, Z, W)`: the
    /// image's, its control points widened by the bound on how far it can
    /// be from the true image. `None` where that bound is above the target:
    /// the piece has to be cut smaller. Fails where the piece reaches the
    /// axis a bend winds round.
    fn law_piece(&self, h: &[Bernstein; 4]) -> GeopResult<Option<[Bernstein; 4]>> {
        // A polynomial piece, its weights all exactly one, has a polynomial
        // image, its weights exactly one too, rather than products of ones
        // rounded outward into a rational piece.
        let one = Bernstein::constant(Scalar::ONE);
        let polynomial = (h[3].coefficients().iter()).all(|w| w.is_exactly(Scalar::ONE));
        let w = if polynomial { &one } else { &h[3] };
        // Coordinates relative to the origin, then in the frame.
        let relative: Vec<Bernstein> = (0..3)
            .map(|k| {
                let o = self.origin[k];
                if o.is_exactly(Scalar::ZERO) {
                    Ok(h[k].clone())
                } else {
                    Ok(Bernstein::combination([(None, &h[k]), (Some(o.neg()), w)])?
                        .expect("two terms"))
                }
            })
            .collect::<GeopResult<_>>()?;
        let local: Vec<Bernstein> = (self.frame.iter())
            .map(|e| {
                let terms = (0..3).filter(|&k| !e[k].is_exactly(Scalar::ZERO)).map(|k| {
                    (
                        (!e[k].is_exactly(Scalar::ONE)).then_some(e[k]),
                        &relative[k],
                    )
                });
                Ok(Bernstein::combination(terms)?
                    .unwrap_or_else(|| Bernstein::constant(Scalar::ZERO)))
            })
            .collect::<GeopResult<_>>()?;
        // What x, y and t reach over the piece (the convex hull property:
        // the weights are positive).
        let reach = |b: &Bernstein| -> GeopResult<Scalar> {
            (b.coefficients().iter().zip(h[3].coefficients()))
                .map(|(c, w)| if polynomial { Ok(*c) } else { c.div(*w) })
                .reduce(|a, b| Ok(a?.union(b?)))
                .expect("at least one coefficient")
        };
        let [x_range, y_range, t_range] = [reach(&local[0])?, reach(&local[1])?, reach(&local[2])?];
        self.check_axis(x_range)?;
        let middle = t_range.sharpen();
        let half = (t_range.upper().sub(middle))
            .max(middle.sub(t_range.lower()))
            .upper();
        let factor = [x_range.abs().upper(), y_range.abs().upper(), Scalar::ONE];
        let size = [x_range, y_range, t_range]
            .iter()
            .fold(Scalar::ONE, |m, r| m.max(r.abs().upper()));
        let target = size.mul(Scalar::from_f64(RELATIVE_TARGET));
        let functions = || {
            (0..3)
                .flat_map(move |col| (0..3).map(move |i| (col, i, self.columns[col][i])))
                .filter(|(_, _, f)| !f.is_zero())
        };
        // Over a piece short enough along the axis (a face across it), each
        // coefficient is enclosed by its value over the whole piece: a
        // constant, the image of the degree of the piece, nothing to bound.
        let mut spread = [Scalar::ZERO; 3];
        for (col, i, f) in functions() {
            spread[i] = spread[i].add(f.evaluate(t_range).width().mul(factor[col]));
        }
        let constant = spread.iter().all(|s| !s.upper().definitely_greater(target));
        let degree = if constant {
            0
        } else {
            self.interpolant_degree()
        };
        // Each coefficient's polynomial in `s = (t - middle) / half`, and
        // the bound on how far the composed point can be off, in the frame.
        let mut error = [Scalar::ZERO; 3];
        let mut polynomials = [[None, None, None], [None, None, None], [None, None, None]];
        for (col, i, f) in functions() {
            if constant {
                polynomials[col][i] = Some(vec![f.evaluate(t_range)]);
                continue;
            }
            let (coefficients, remainder) = f.interpolant(middle, half, degree)?;
            error[i] = error[i].add(remainder.abs().mul(factor[col]));
            polynomials[col][i] = Some(coefficients);
        }
        if error.iter().any(|e| e.upper().definitely_greater(target)) {
            return Ok(None);
        }
        // s^n W^(d-n) for n = 0..=d, as `S = (T - middle W) / half`; a
        // piece with no reach along the axis is a constant one.
        let scale = if constant {
            Scalar::ONE
        } else {
            Scalar::ONE.div(half)?
        };
        let u =
            Bernstein::combination([(Some(scale), &local[2]), (Some(middle.mul(scale).neg()), w)])?
                .expect("two terms");
        let basis: Vec<Bernstein> = (0..=degree)
            .map(|n| u.power(n)?.product(&w.power(degree - n)?))
            .collect::<GeopResult<_>>()?;
        let composed = |coefficients: &Vec<Scalar>| -> GeopResult<Bernstein> {
            let terms = (coefficients.iter().zip(&basis))
                .filter(|(c, _)| !c.is_exactly(Scalar::ZERO))
                .map(|(c, b)| ((!c.is_exactly(Scalar::ONE)).then_some(*c), b));
            Ok(Bernstein::combination(terms)?.unwrap_or_else(|| Bernstein::constant(Scalar::ZERO)))
        };
        let factors = [&local[0], &local[1], w];
        // N_i = Â_i x + B̂_i y + Ĉ_i W, in the frame.
        let numerators: Vec<Option<Bernstein>> = (0..3)
            .map(|i| {
                let products = (0..3)
                    .filter_map(|col| polynomials[col][i].as_ref().map(|c| (col, c)))
                    .map(|(col, c)| composed(c)?.product(factors[col]))
                    .collect::<GeopResult<Vec<_>>>()?;
                Bernstein::combination(products.iter().map(|p| (None, p)))
            })
            .collect::<GeopResult<_>>()?;
        let weight = basis[0].product(w)?;
        // Back in the world: origin W' + Σ_i e_i N_i.
        let image: Vec<Bernstein> = (0..3)
            .map(|k| {
                let mut terms = Vec::new();
                if !self.origin[k].is_exactly(Scalar::ZERO) {
                    terms.push((Some(self.origin[k]), &weight));
                }
                for (e, n) in self.frame.iter().zip(&numerators) {
                    if let Some(n) = n
                        && !e[k].is_exactly(Scalar::ZERO)
                    {
                        terms.push(((!e[k].is_exactly(Scalar::ONE)).then_some(e[k]), n));
                    }
                }
                Ok(Bernstein::combination(terms)?
                    .unwrap_or_else(|| Bernstein::constant(Scalar::ZERO)))
            })
            .collect::<GeopResult<_>>()?;
        // All of one degree: the numerators', or the weight's if higher.
        let degree = (image.iter().chain([&weight]).map(Bernstein::degree))
            .fold([0, 0], |a, b| [a[0].max(b[0]), a[1].max(b[1])]);
        let image: Vec<Bernstein> = (image.iter())
            .map(|b| b.elevated(degree))
            .collect::<GeopResult<_>>()?;
        let weight = weight.elevated(degree)?;
        // Each control point widened by what the error reaches along each
        // of the world's axes: a convex combination of points each within
        // it of where it should be is within it too.
        let widen: [Scalar; 3] = std::array::from_fn(|k| {
            let e = (self.frame.iter().zip(error))
                .fold(Scalar::ZERO, |sum, (axis, e)| sum.add(axis[k].abs().mul(e)))
                .upper()
                .to_f64();
            Scalar::new(-e, e)
        });
        let image: [Bernstein; 3] = std::array::from_fn(|k| {
            if widen[k].is_exactly(Scalar::ZERO) {
                return image[k].clone();
            }
            let coefficients = (image[k].coefficients().iter().zip(weight.coefficients()))
                .map(|(c, w)| c.add(widen[k].mul(*w)))
                .collect();
            Bernstein::new(weight.degree(), coefficients)
        });
        let [x, y, z] = image;
        Ok(Some([x, y, z, weight]))
    }

    /// Fails where `x`, the coordinate towards a bend's centre, could reach
    /// the axis the bend winds round: the radius and past it.
    fn check_axis(&self, x: Scalar) -> GeopResult<()> {
        let cx = self.columns[2][0];
        if !cx.is_trig() {
            return Ok(());
        }
        let radius = cx.constant;
        let reaches = if radius.definitely_greater(Scalar::ZERO) {
            x.could_be_greater(radius) || x.could_be_equal(radius)
        } else {
            x.could_be_less(radius) || x.could_be_equal(radius)
        };
        if reaches {
            return Err(GeopError::new(format!(
                "AxialMap::bend: material reaches {x:?} towards the bend's centre, as far as its radius {radius:?} or past it, where it would be turned inside out"
            )));
        }
        Ok(())
    }

    /// The law applied to `curve`: each Bézier piece, halved while its
    /// image is not within the target, joined back at knots of full
    /// multiplicity at the parameters they were cut at.
    fn law_curve(&self, curve: &NurbCurve3D) -> GeopResult<NurbCurve3D> {
        let mut todo: Vec<NurbCurve3D> = curve.bezier_pieces()?;
        todo.reverse();
        let mut done: Vec<(Scalar, [Bernstein; 4])> = Vec::new();
        while let Some(piece) = todo.pop() {
            let p = piece.degree;
            let h: [Bernstein; 4] = std::array::from_fn(|k| {
                Bernstein::new([p, 0], piece.control_points.iter().map(|c| c[k]).collect())
            });
            match self.law_piece(&h)? {
                Some(image) => done.push((piece.domain().0, image)),
                None => {
                    if done.len() + todo.len() + 2 > MAX_PIECES {
                        return Err(GeopError::new(format!(
                            "AxialMap: the curve needs more than {MAX_PIECES} pieces to be deformed within the target; its domain {:?}",
                            curve.domain()
                        )));
                    }
                    let (left, right) = piece.split_mid()?;
                    todo.push(right);
                    todo.push(left);
                }
            }
        }
        let degree = done[0].1[3].degree()[0];
        let mut points: Vec<Vector4> = Vec::new();
        let mut knots = vec![curve.domain().0; degree + 1];
        for (i, (start, image)) in done.iter().enumerate() {
            if i > 0 {
                knots.extend(std::iter::repeat_n(*start, degree));
            }
            let piece: Vec<Vector4> = (0..=degree)
                .map(|j| Vector4::from_array(std::array::from_fn(|k| image[k].coefficients()[j])))
                .collect();
            match points.last_mut() {
                Some(last) => {
                    *last = last.union(&piece[0]);
                    points.extend_from_slice(&piece[1..]);
                }
                None => points.extend(piece),
            }
        }
        knots.extend(std::iter::repeat_n(curve.domain().1, degree + 1));
        NurbCurve3D::try_new(degree, points, knots)
    }

    /// The law applied to `surface`: its Bézier patches, on a grid refined
    /// (each cell halved along the direction it runs further along the
    /// axis) until every cell's image is within the target, joined back at
    /// knots of full multiplicity where the grid's lines were.
    fn law_surface(&self, surface: &NurbSurface3D) -> GeopResult<NurbSurface3D> {
        let mut grid = [surface.breakpoints_u(), surface.breakpoints_v()];
        loop {
            let cells = (grid[0].len() - 1) * (grid[1].len() - 1);
            if cells > MAX_PIECES {
                return Err(GeopError::new(format!(
                    "AxialMap: the surface needs more than {MAX_PIECES} patches to be deformed within the target; its domain {:?} x {:?}",
                    surface.domain_u(),
                    surface.domain_v()
                )));
            }
            let mut halve = [
                vec![false; grid[0].len() - 1],
                vec![false; grid[1].len() - 1],
            ];
            let mut images = Vec::with_capacity(cells);
            for i in 0..grid[0].len() - 1 {
                for j in 0..grid[1].len() - 1 {
                    let cell = surface
                        .sub_surface((grid[0][i], grid[0][i + 1]), (grid[1][j], grid[1][j + 1]))?;
                    let h: [Bernstein; 4] = std::array::from_fn(|k| {
                        Bernstein::new(
                            [cell.degree_u, cell.degree_v],
                            cell.control_points.iter().map(|c| c[k]).collect(),
                        )
                    });
                    match self.law_piece(&h)? {
                        Some(image) => images.push(image),
                        None => {
                            let [along_u, along_v] = self.runs_along_axis(&cell)?;
                            // Both where neither runs twice as far as the
                            // other along the axis.
                            halve[0][i] |= along_u * 2.0 >= along_v;
                            halve[1][j] |= along_v * 2.0 >= along_u;
                        }
                    }
                }
            }
            if images.len() == cells {
                return assemble(&grid, images);
            }
            for (breaks, halve) in grid.iter_mut().zip(halve) {
                let mut refined = vec![breaks[0]];
                for (pair, halve) in breaks.windows(2).zip(halve) {
                    if halve {
                        refined.push(pair[0].add(pair[1]).div(Scalar::TWO)?.sharpen());
                    }
                    refined.push(pair[1]);
                }
                *breaks = refined;
            }
        }
    }

    /// How far a patch's control net runs along the axis along `u` and
    /// along `v`: the largest sum of the steps along a row of it. Only
    /// chooses which way to cut a patch, a free choice.
    fn runs_along_axis(&self, cell: &NurbSurface3D) -> GeopResult<[f64; 2]> {
        let t: Vec<f64> = (cell.control_points.iter())
            .map(|p| Ok(self.local(&p.dehomogenized()?)[2].to_f64()))
            .collect::<GeopResult<_>>()?;
        let (nu, nv) = (cell.num_u, cell.num_v);
        let along_u = (0..nv)
            .map(|j| {
                (1..nu)
                    .map(|i| (t[i * nv + j] - t[(i - 1) * nv + j]).abs())
                    .sum::<f64>()
            })
            .fold(0.0, f64::max);
        let along_v = (0..nu)
            .map(|i| {
                (1..nv)
                    .map(|j| (t[i * nv + j] - t[i * nv + j - 1]).abs())
                    .sum::<f64>()
            })
            .fold(0.0, f64::max);
        Ok([along_u, along_v])
    }
}

/// The patches `images` of the cells of `grid` over `surface`'s domain
/// (`u`-index major), joined into one surface: knots of full multiplicity
/// at the grid's lines, the control points two cells share the union of
/// theirs. The parametrization is the surface's.
fn assemble(grid: &[Vec<Scalar>; 2], images: Vec<[Bernstein; 4]>) -> GeopResult<NurbSurface3D> {
    let [du, dv] = images[0][3].degree();
    let (cu, cv) = (grid[0].len() - 1, grid[1].len() - 1);
    let (nu, nv) = (cu * du + 1, cv * dv + 1);
    let mut points: Vec<Option<Vector4>> = vec![None; nu * nv];
    for (index, image) in images.iter().enumerate() {
        let (ci, cj) = (index / cv, index % cv);
        for a in 0..=du {
            for b in 0..=dv {
                let p = Vector4::from_array(std::array::from_fn(|k| {
                    image[k].coefficients()[a * (dv + 1) + b]
                }));
                let slot = &mut points[(ci * du + a) * nv + cj * dv + b];
                *slot = Some(match slot {
                    None => p,
                    Some(q) => q.union(&p),
                });
            }
        }
    }
    let knots = |breaks: &[Scalar], degree: usize| {
        let mut knots = vec![breaks[0]; degree + 1];
        for &k in &breaks[1..breaks.len() - 1] {
            knots.extend(std::iter::repeat_n(k, degree));
        }
        knots.extend(std::iter::repeat_n(breaks[breaks.len() - 1], degree + 1));
        knots
    };
    NurbSurface3D::try_new(
        du,
        dv,
        points
            .into_iter()
            .map(|p| p.expect("every control point set"))
            .collect(),
        knots(&grid[0], du),
        knots(&grid[1], dv),
    )
}

impl Deformation for AxialMap {
    /// The image of every piece `p` could lie in, united: where `p` could
    /// be on a zone's end, the law and the affine map there both enclose
    /// it.
    fn point(&self, p: &Vector3) -> GeopResult<Vector3> {
        let local = self.local(p);
        let mut image: Option<Vector3> = None;
        for piece in self.pieces(local[2]) {
            let q = match piece {
                Piece::Before => self.tangent_motion(self.start)?.apply(p),
                Piece::After => self.tangent_motion(self.end)?.apply(p),
                Piece::Law => {
                    self.check_axis(local[0])?;
                    self.law_point(local)
                }
            };
            image = Some(image.map_or(q, |i| i.union(&q)));
        }
        Ok(image.expect("every t falls in some piece"))
    }

    fn curve(
        &self,
        curve: &NurbCurve3D,
        at: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbCurve3D> {
        match self.piece_of(&curve.control_points, at)? {
            Piece::Before => Ok(curve.transform(&self.tangent_motion(self.start)?)),
            Piece::After => Ok(curve.transform(&self.tangent_motion(self.end)?)),
            Piece::Law => self.law_curve(curve),
        }
    }

    fn surface(
        &self,
        surface: &NurbSurface3D,
        at: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<NurbSurface3D> {
        match self.piece_of(&surface.control_points, at)? {
            Piece::Before => Ok(surface.transform(&self.tangent_motion(self.start)?)),
            Piece::After => Ok(surface.transform(&self.tangent_motion(self.end)?)),
            Piece::Law => self.law_surface(surface),
        }
    }

    /// A positive taper, a twist and a bend all keep a frame right-handed.
    fn mirrors(&self) -> bool {
        false
    }

    fn affine(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_fixtures::line;

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    fn zone(a: f64, b: f64) -> (Scalar, Scalar) {
        (Scalar::from_f64(a), Scalar::from_f64(b))
    }

    /// A patch slanted across the axis `z`, from `z = 0` to `z = 2`.
    fn slanted() -> NurbSurface3D {
        NurbSurface3D::bilinear(
            v(0.2, -0.3, 0.0),
            v(1.0, 0.1, 0.2),
            v(0.8, 0.5, 2.0),
            v(-0.1, 0.4, 1.7),
        )
        .unwrap()
    }

    fn none() -> GeopResult<Vector3> {
        Err(GeopError::new("no point asked for"))
    }

    /// The image of every point of a surface is where the map puts it:
    /// sampled on a grid, each composed point encloses the map's own.
    fn assert_maps_surface(map: &AxialMap, surface: &NurbSurface3D) -> NurbSurface3D {
        let image = map.surface(surface, &none).unwrap();
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        for i in 0..=6 {
            for j in 0..=6 {
                let u = Scalar::interpolate(u0, u1, Scalar::from_f64(i as f64 / 6.0)).sharpen();
                let w = Scalar::interpolate(v0, v1, Scalar::from_f64(j as f64 / 6.0)).sharpen();
                let expected = map.point(&surface.evaluate(u, w).unwrap()).unwrap();
                let got = image.evaluate(u, w).unwrap();
                assert!(
                    got.could_be_equal(&expected),
                    "at ({u:?}, {w:?}): {got:?} vs {expected:?}"
                );
                for k in 0..3 {
                    assert!(got[k].width().to_f64() < 1e-7, "{got:?}");
                }
            }
        }
        image
    }

    /// A taper of a plane through the axis's zone is exact: of degree two,
    /// every control point a few roundings wide.
    #[test]
    fn a_taper_is_exact_and_quadratic() {
        let map = AxialMap::taper(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            None,
            zone(0.0, 2.0),
            Scalar::from_f64(0.5),
        )
        .unwrap();
        let surface = slanted();
        let image = assert_maps_surface(&map, &surface);
        assert_eq!((image.degree_u, image.degree_v), (2, 2));
        assert!(map.as_motion().unwrap().is_none());
    }

    /// A pinch scales only the way across it is given: a point at `(1, 1,
    /// 1)`, halfway along a zone pinching `x` to a half, goes to `(0.75, 1,
    /// 1)`. Exact, as a taper is.
    #[test]
    fn a_pinch_scales_one_way_across() {
        let map = AxialMap::taper(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            Some(&v(1.0, 0.0, 0.0)),
            zone(0.0, 2.0),
            Scalar::from_f64(0.5),
        )
        .unwrap();
        let p = map.point(&v(1.0, 1.0, 1.0)).unwrap();
        assert!(p.could_be_equal(&v(0.75, 1.0, 1.0)), "{p:?}");
        assert_maps_surface(&map, &slanted());
    }

    /// A shear is affine: the motion it is, the same before, over and past
    /// the zone, moving a plane across the axis by the offset times its
    /// distance from the zone's start over the zone's length.
    #[test]
    fn a_shear_is_a_motion() {
        let map = AxialMap::shear(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            &v(0.0, 1.0, 0.0),
            zone(1.0, 3.0),
            Scalar::from_f64(0.5),
        )
        .unwrap();
        let motion = map.as_motion().unwrap().expect("a shear is affine");
        for (p, q) in [
            (v(1.0, 0.0, 0.0), v(1.0, -0.25, 0.0)),
            (v(0.0, 2.0, 2.0), v(0.0, 2.25, 2.0)),
            (v(0.5, 0.0, 5.0), v(0.5, 1.0, 5.0)),
        ] {
            assert!(motion.apply(&p).could_be_equal(&q), "{p:?}");
            assert!(map.point(&p).unwrap().could_be_equal(&q), "{p:?}");
        }
        assert!(!motion.mirrors());
    }

    /// A twist of a quarter turn and a bend of a half turn of a slanted
    /// patch enclose the true image to well within its width.
    #[test]
    fn a_twist_and_a_bend_enclose_the_true_image() {
        let surface = slanted();
        let twist = AxialMap::twist(
            v(0.1, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            zone(0.0, 2.0),
            Scalar::from_f64(1.5),
        )
        .unwrap();
        assert_maps_surface(&twist, &surface);
        let bend = AxialMap::bend(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            &v(-1.0, 0.0, 0.0),
            zone(0.0, 2.0),
            Scalar::from_f64(3.0),
        )
        .unwrap();
        assert_maps_surface(&bend, &surface);
    }

    /// A line along the axis, bent: an arc of the bend's radius, its ends
    /// where the bend's law puts them.
    #[test]
    fn a_line_along_the_axis_bends_into_an_arc() {
        let bend = AxialMap::bend(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            &v(1.0, 0.0, 0.0),
            zone(0.0, 1.0),
            Scalar::PI.div(Scalar::TWO).unwrap(),
        )
        .unwrap();
        let curve = line([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let image = bend.curve(&curve, &none).unwrap();
        let radius = 2.0 / std::f64::consts::PI;
        let end = image.evaluate(image.domain().1).unwrap();
        assert!(
            end.could_be_equal(&v(radius, 0.0, radius)) || (end[0].to_f64() - radius).abs() < 1e-9,
            "{end:?}"
        );
        let mid = image.evaluate(image.mid_parameter().unwrap()).unwrap();
        let r = ((mid[0].to_f64() - radius).powi(2) + mid[2].to_f64().powi(2)).sqrt();
        assert!((r - radius).abs() < 1e-9, "{mid:?}");
    }

    /// Before and after the zone the map is affine and continues the law:
    /// a twist carries everything past its end turned by its angle.
    #[test]
    fn past_the_zone_a_twist_turns_rigidly() {
        let twist = AxialMap::twist(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            zone(0.0, 1.0),
            Scalar::PI.div(Scalar::TWO).unwrap(),
        )
        .unwrap();
        let p = twist.point(&v(1.0, 0.0, 3.0)).unwrap();
        assert!(
            p.could_be_equal(&v(0.0, 1.0, 3.0)) || (p[1].to_f64() - 1.0).abs() < 1e-12,
            "{p:?}"
        );
        assert!(
            twist
                .point(&v(1.0, 0.0, -2.0))
                .unwrap()
                .could_be_equal(&v(1.0, 0.0, -2.0))
        );
        // On the end itself both pieces enclose the point.
        let q = twist.point(&v(0.0, 2.0, 1.0)).unwrap();
        assert!(q.could_be_equal(&v(-2.0, 0.0, 1.0)), "{q:?}");
    }

    /// The cubic standing in for `cos` over a reach of 0.1 is within its
    /// bound of it, and the bound within a few times the true error,
    /// `0.1^4 / (8 · 4!)` at most.
    #[test]
    fn the_interpolant_of_a_cosine_is_within_its_bound() {
        let one = Scalar::ONE;
        let f = AxialFunction::trig(Scalar::ZERO, one, Scalar::ZERO, one, Scalar::ZERO);
        let (middle, reach) = (Scalar::from_f64(0.3), Scalar::from_f64(0.1));
        let (coefficients, bound) = f.interpolant(middle, reach, 3).unwrap();
        let bound = bound.upper().to_f64();
        assert!(bound < 4.0 * 1e-4 / 192.0, "{bound}");
        for k in 0..=20 {
            let s = -0.1 + 0.01 * k as f64;
            let p: f64 =
                (coefficients.iter().rev()).fold(0.0, |acc, c| acc * (s / 0.1) + c.to_f64());
            assert!(
                (p - (0.3 + s).cos()).abs() <= bound,
                "at {s}: {p} vs {}",
                (0.3 + s).cos()
            );
        }
    }

    /// A bend of a whole turn, and one whose axis the body reaches, are
    /// refused.
    #[test]
    fn a_full_turn_and_material_past_the_centre_are_refused() {
        let full = AxialMap::bend(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            &v(1.0, 0.0, 0.0),
            zone(0.0, 1.0),
            Scalar::from_f64(7.0),
        );
        assert!(full.is_err());
        // Radius 1 / 3: a point at x = 0.5 lies past the centre.
        let bend = AxialMap::bend(
            v(0.0, 0.0, 0.0),
            &v(0.0, 0.0, 1.0),
            &v(1.0, 0.0, 0.0),
            zone(0.0, 1.0),
            Scalar::from_f64(3.0),
        )
        .unwrap();
        let error = bend.point(&v(0.5, 0.0, 0.5)).unwrap_err();
        assert!(format!("{error}").contains("radius"), "{error}");
    }
}
