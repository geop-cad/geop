# Curve containment by per-axis clipping

Implemented (Clip B + section 5) in `curve.rs`; `curve_bisect.rs` is the
previous hull-test-and-bisect search it is benchmarked against
(`examples/curve_contains_bench.rs`).

Given a NURBS curve and a point $p$, find an enclosure of the parameter set

$$
T = \{\, t \in [a, b] : C(t) = p \,\}.
$$

The answer is an interval $\hat T \supseteq T$ (a *proof*: $\hat T = \emptyset$
means the curve definitely misses $p$). All quantities below are `Scalar`s,
i.e. intervals, and every step must keep the inclusion property: for every
real instantiation of the inputs inside their intervals, the real result lies
inside the computed interval. No epsilons appear anywhere (see `AGENTS.md`).

## 1. Reduce to scalar spline functions (no division)

The curve is stored homogeneously with control points
$P_i = (w_i x_i,\ w_i y_i,\ w_i z_i,\ w_i)$ and B-spline basis $N_{i,p}(t)$ on
the knot vector $u_0 \le \dots \le u_{n+p+1}$:

$$
C(t) = \frac{\sum_i w_i\,\mathbf{x}_i\,N_{i,p}(t)}{W(t)}, \qquad
W(t) = \sum_i w_i\,N_{i,p}(t).
$$

All weights are definitely positive — `NurbCurve::try_new` rejects anything
else, and `split` only forms convex combinations of existing control points.
Then $W(t) > 0$ on $[a, b]$ because the basis is non-negative and sums to one.

With $W(t) > 0$, for each axis $k \in \{x, y, z\}$:

$$
C_k(t) = p_k \iff g_k(t) := \sum_i d_{k,i}\, N_{i,p}(t) = 0,
\qquad d_{k,i} = w_i x_{k,i} - p_k\, w_i = P_{i,k} - p_k\, P_{i,w}.
$$

So each axis gives a *polynomial* (non-rational) spline function whose
coefficients are computed directly from the stored homogeneous coordinates,
with one multiplication and one subtraction per coefficient — no division, so
no interval blow-up from small weights. Because $p_k$ is an interval,
$d_{k,i}$ is an interval $[d_{k,i}^-, d_{k,i}^+]$.

Then

$$
T = Z_x \cap Z_y \cap Z_z, \qquad Z_k = \{ t \in [a,b] : g_k(t) = 0 \},
$$

and any enclosures $\hat Z_k \supseteq Z_k$ give $\hat T = \hat Z_x \cap \hat Z_y
\cap \hat Z_z \supseteq T$ (intersection of enclosures is still an enclosure:
both constraints hold simultaneously — `Scalar::intersect`, with an empty
result meaning "rejected"). For 2-D pcurves ($D = 3$) only $x, y$ are used.

## 2. The graph of $g_k$ is itself a B-spline curve

This is what makes "clip against the $t$-axis" legitimate. Let the **Greville
abscissae** be

$$
\xi_i = \frac{u_{i+1} + \dots + u_{i+p}}{p}, \qquad i = 0, \dots, n.
$$

B-splines have linear precision, $\sum_i \xi_i N_{i,p}(t) = t$, hence the
planar graph

$$
G_k(t) = \bigl(t,\ g_k(t)\bigr) = \sum_i (\xi_i,\ d_{k,i})\, N_{i,p}(t)
$$

is a B-spline curve with control points $Q_{k,i} = (\xi_i, d_{k,i})$. By the
convex hull property, $G_k([a,b]) \subseteq \operatorname{conv}\{Q_{k,i}\}$.

Note the first coordinate of the control points is $\xi_i$, **not** the index
$i$ and not an arbitrary spacing. (For a Bézier segment on $[a, b]$,
$\xi_i = a + \tfrac{i}{n}(b - a)$.) $\xi_i$ is non-decreasing in $i$ and, for
a clamped knot vector, $\xi_0 = a$, $\xi_n = b$. It is computed in interval
arithmetic from the (possibly unsharp) knots.

### Accounting for interval coefficients

Because $d_{k,i} \in [d_{k,i}^-, d_{k,i}^+]$, the true $g_k$ is squeezed
between two splines:

$$
g_k^-(t) := \sum_i d_{k,i}^- N_{i,p}(t) \;\le\; g_k(t) \;\le\;
\sum_i d_{k,i}^+ N_{i,p}(t) =: g_k^+(t),
$$

using $N_{i,p} \ge 0$. Let $L_k$ be the **lower** convex hull of the points
$(\xi_i, d_{k,i}^-)$ and $U_k$ the **upper** convex hull of
$(\xi_i, d_{k,i}^+)$ (as piecewise-linear functions of $t$ on
$[\xi_0, \xi_n]$). By the convex hull property $L_k \le g_k^-$ and
$g_k^+ \le U_k$, so

$$
g_k(t) = 0 \;\Rightarrow\; L_k(t) \le 0 \le U_k(t).
$$

$L_k$ is convex and $U_k$ is concave, so $\{L_k \le 0\}$ and $\{U_k \ge 0\}$
are each a single interval, and so is their intersection. That intersection
is $\hat Z_k$. This is the precise version of "intersect the convex hull with
the $t$-axis".

## 3. Clip A: exact hull clip

The left end of $\{L_k \le 0\}$ is the leftmost point of
$\operatorname{conv}\{(\xi_i, d_i^-)\} \cap \{y \le 0\}$. The leftmost point
of a convex polygon clipped by a half-plane is either a vertex inside the
half-plane or a crossing of a hull edge with the line $y = 0$. Every segment
between two input points lies inside the hull, so taking the extreme over
*all* points and *all* pairs (not just hull edges) gives exactly the same
extreme, without constructing the hull:

$$
t_{\text{lo}} = \min\Bigl(
  \{\xi_i : d_i^- \le 0\} \;\cup\;
  \{\tau_{ij} : d_i^- > 0 \ge d_j^-\}
\Bigr),
\qquad
\tau_{ij} = \xi_i + d_i^-\,\frac{\xi_j - \xi_i}{d_i^- - d_j^-},
$$

and symmetrically $t_{\text{hi}}$ with $\max$; likewise for
$\{U_k \ge 0\}$ with $d^+$ and the inequalities flipped.

Interval-arithmetic rules that keep this sound when $\xi_i$, $d_i^\pm$ are
themselves unsharp:

- A candidate is included if its condition **could** hold
  (`could_be_less`/`could_be_equal`), never only if it definitely holds.
  Including an extra candidate can only widen the result.
- The min/max over candidate enclosures is taken on their outer bounds, i.e.
  the result is the `union` of all candidates.
- If $d_i^- - d_j^-$ could be zero, the crossing is not computed; both
  $\xi_i$ and $\xi_j$ are then within the widths of $0$ and included directly
  (since $\tau_{ij} \in [\xi_i, \xi_j]$ whenever it exists, their union covers
  it).
- If no candidate exists, $Z_k = \emptyset$ and the segment is rejected.

Cost $O(n^2)$ in the number of control points of the segment. An $O(n)$
monotone-chain hull (the $\xi_i$ are already sorted) gives the same result in
exact arithmetic, but its orientation tests are three-valued under intervals:
both keeping a doubtful vertex and dropping one move the chain *above* the true
lower hull, which is unsound. The pairwise form has no such decision to get
wrong, and segments are small after a few subdivisions.

## 4. Clip B: fat line clip (cheaper, looser)

Replace the hull by a parallelogram-shaped band around the chord from
$Q_0$ to $Q_n$. With $d_i$ the full interval coefficients:

$$
s = \frac{d_n - d_0}{\xi_n - \xi_0}, \qquad
\ell(t) = d_0 + s\,(t - \xi_0), \qquad
\delta_i = d_i - \ell(\xi_i), \qquad
\Delta = \bigcup_i \delta_i .
$$

$\Delta$ is the `union` of the **signed vertical** offsets of all control
points from the chord. It contains $0$ (from $i = 0$ and $i = n$) and is in
general **not** symmetric, so it must not be replaced by $\pm$ half of a
"tolerance": a symmetric band either discards the real asymmetry (looser) or,
if built from half the maximal distance as a symmetric offset, fails to cover
the points on the far side (unsound). Vertical offsets are used rather than
perpendicular distances because the band is intersected with a horizontal
line; the perpendicular form would need a $\sqrt{1+s^2}$ and gains nothing.

All $Q_i$ lie in the convex band $\{(t, y) : y - \ell(t) \in \Delta\}$, so by
the convex hull property $g_k(t) \in \ell(t) + \Delta$ for all $t \in [a, b]$.
Hence $g_k(t) = 0$ requires $\ell(t) \in -\Delta$, i.e.

$$
\hat Z_k = \Bigl(\xi_0 - \frac{d_0 + \Delta}{s}\Bigr) \cap [a, b]
\qquad \text{if } s \text{ is definitely non-zero.}
$$

This is one interval expression; interval arithmetic's inclusion property makes
it a valid enclosure directly (the correlation between $d_0$, $s$ and $\Delta$
is lost, which only widens the result).

Before dividing, one range test that is valid for every $s$: on $[a, b]$ the
line $\ell$ stays between $d_0$ and $d_n$, so the band there lies in
$\operatorname{hull}(d_0, d_n) + \Delta$. If that misses $0$
(`!d_0.union(d_n).add(Δ).could_be_equal(ZERO)`), $\hat Z_k = \emptyset$. This
also decides the case the division cannot: if $s$ could be $0$ (the chord is
horizontal within its width) and the test did not reject, $\hat Z_k = [a, b]$
(no information from this axis). Testing only $d_0 + \Delta$ there would be
unsound — an $s$ that merely *could* be zero may still be non-zero, and $\ell$
then moves across the whole $\operatorname{hull}(d_0, d_n)$.

Clip B is $O(n)$ and never tighter than Clip A (the band contains the hull).
`curve.rs` uses Clip B only; Clip A could be added as a second stage run only
when Clip B neither rejects nor contracts.

## 5. Iteration and termination

One round on a segment with domain $[a, b]$:

1. Compute $\hat T = \hat Z_x \cap \hat Z_y \cap \hat Z_z$ (Clip B).
2. $\hat T = \emptyset$ → reject the segment.
3. If the clip removed less than 20% of $[a, b]$ (the usual Bézier-clipping
   rule; happens when the segment contains several solutions, or a tangency),
   split at the midpoint as `curve_bisect::curve_could_contain` does and push both
   halves.
4. Otherwise, restrict the segment to $\hat T$ (`NurbCurve::sub_curve`, cutting
   at the sharp outer bounds `lower()`/`upper()` of $\hat T$) and repeat.

The split/restriction parameters are **free choices** and are sharpened before
splitting (see `AGENTS.md`); the enclosure $\hat T$ itself is an *answer*, so
the restriction must use its outer bounds, never a sharpened midpoint, and the
reported result is the union of the final segments' domains.

Termination uses the existing tunables only. A segment is reported with its
clipped $\hat T$ (tighter than its domain, and still an enclosure) once either

- $\hat T$ is narrow (not `definitely_greater` than `min_subdivision_size`)
  and the curve evaluated over the interval $\hat T$ contains $p$ with every
  coordinate's width within `min_subdivision_size` — if that evaluation
  *misses* $p$, the segment is rejected instead. Interval de Boor evaluation
  only encloses $C(\hat T)$ when $\hat T$ lies in a single knot span, so
  this is skipped when an interior knot is not definitely outside $\hat T$;
  or
- its chord is not `definitely_greater` than `min_subdivision_size`.

A narrow $\hat T$ alone is not enough: one axis can pin $t$ down while the
others were never checked at that precision, which would report points that
are far from the curve. The evaluation is also what ends a search whose
solution sits exactly on a domain end, where $\hat T$ collapses but no cut is
possible. The whole search stops after `max_nodes` segments. These affect
effort, not what the returned enclosure means.

## Limitations

- Neither this test nor the 3-D hull (GJK) test dominates the other: the
  per-axis clips can each keep an interval although the 3-D hull misses $p$,
  while conversely disjoint per-axis intervals can reject a segment whose 3-D
  hull contains $p$ (the clips use the parameter $t$, the hull does not). Both
  are sound, so they combine by running both and rejecting if either rejects.
- Near a tangential or repeated contact ($g_k$ has a double root), clipping
  contracts slowly; step 3 bounds the damage by falling back to bisection.
