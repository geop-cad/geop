# Curve–curve intersection by per-axis clipping

Design extension of [curve containment](../contains/curve.md). Implemented
in [`curve_curve.rs`](curve_curve.rs) (benchmarked against the previous
hull-and-bisect [`curve_curve_bisect.rs`](curve_curve_bisect.rs) by `examples/intersection_bench.rs`), with
these choices:

- Clip B only, through the shared `fat_line::clip_tensor`.
- Besides the axis equations, it clips **free-choice combinations**
  $g_n=\sum_k n_k g_k$. These vanish wherever every $g_k$ does, so they are
  exactly as sound. The directions $n$ are taken perpendicular to the *other*
  curve's chord, which makes $g_n$ nearly independent of that curve's
  parameter. This recovers the coupling §3's diagonal example shows the axis
  projections lose (it yields $2s-1$ and $2t-1$), and replaces the axes
  unless a chord is degenerate.
- A parameter pinned exactly onto a clamped end is handed to `contains::curve` as
  point containment of that endpoint.
- Overlapping boxes merge (componentwise union) into unresolved clusters.
- Exhausting `max_nodes` is an error.
- Refinement (§4) is not implemented.
- The no-coincidence premise holds for `curve_curve_crossings`. The public
  `curve_curve_intersect` wraps it with the old signature and `Intersections`
  contract, and finds overlaps directly (`coincidence.rs`). An overlap can only
  begin or end where one curve's end lies on the other, so it tests the
  midpoint between consecutive such candidates. The crossing search then runs
  only on the stretches between overlaps. Assume no coincident arcs: intersections
are distinct spatial points. Tangential contacts are still allowed. The
construction encloses all parameter pairs; it does not infer coincidence
from a number of surviving leaves.

## 1. Turn equality into a tensor-product spline

Let the homogeneous curves be

$$
A(s)=\frac{H_A(s)}{W_A(s)},\quad (H_A,W_A)=\sum_i P_iN_i(s),\qquad
B(t)=\frac{H_B(t)}{W_B(t)},\quad (H_B,W_B)=\sum_j Q_jM_j(t).
$$

With definitely positive weights, both denominators are nonzero. Hence

$$
A_k(s)=B_k(t)\iff
g_k(s,t)=H_{A,k}(s)W_B(t)-W_A(s)H_{B,k}(t)=0.
$$

Its tensor-product coefficients are simply

$$
d_{k,ij}=P_{i,k}Q_{j,w}-P_{i,w}Q_{j,k},\qquad
g_k=\sum_{i,j}d_{k,ij}N_i(s)M_j(t).
$$

There is no degree elevation or common knot vector to compute: the factors
use **independent parameters**. If the curves have degrees $p,q$, $g_k$ has
bidegree $(p,q)$. Equating curves at one shared parameter would miss crossings
whose two parameters differ. Use two Cartesian equations for 2-D pcurves,
three for 3-D curves.

The desired set is

$$
R=\{(s,t)\in I\times J:g_k(s,t)=0\text{ for every }k\}.
$$

Represent its enclosure as a list of paired boxes $I_r\times J_r$, not
independent lists of $s$ and $t$ values. Pairing is part of the answer.

## 2. Clip in both parameter directions

Let $\xi_i,\eta_j$ be the curves' Greville abscissae. Then

$$
(s,t,g_k(s,t))=\sum_{i,j}(\xi_i,\eta_j,d_{k,ij})N_i(s)M_j(t).
$$

This is exactly the scalar graph construction in
[surface containment](../contains/surface.md). Collapse the opposite index:

$$
D^s_{k,i}=\bigcup_j d_{k,ij},\qquad
D^t_{k,j}=\bigcup_i d_{k,ij}.
$$

Clip A or Clip B from the curve note applied to $(\xi_i,D^s_{k,i})$ yields
$\hat I_k$, and to $(\eta_j,D^t_{k,j})$ yields $\hat J_k$. Retain

$$
\hat B=\left(I\cap\bigcap_k\hat I_k\right)
       \times\left(J\cap\bigcap_k\hat J_k\right).
$$

Any empty factor rejects this curve pair. The proof is the convex hull
property after projection; row unions cover all choices of the other
parameter. No sampled closest point or geometric distance tolerance is used.

In particular, Clip B uses the signed offset band for each projected list.
If its slope could be zero, retain that direction unless a range test rejects
it. All uncertain signs and crossings follow the conservative interval rules
in the curve note. Degree-zero spans and collapsed domains use range or
boundary tests, as described in the surface note.

Coefficient construction and Clip B cost $O(n_A n_B)$ per spatial axis;
Clip A adds $O(n_A^2+n_B^2)$ after collapsing the indices. The tensor need
not be stored if coefficients are streamed into the row/column unions.

## 3. Global search over pairs of subcurves

1. Queue the original pair, or pairs of knot-span pieces. Use AABB, fat-axis,
   or geometric hull separation as additional rejection tests.
2. Form $d_{k,ij}$, clip both parameters, and reject an empty box.
3. Restrict both curves to the clips' outer bounds, keeping their original
   parameter coordinates, then rebuild the coefficients and repeat.
4. When contraction stalls, bisect one parameter and enqueue both child
   pairs. Balance subdivision so neither unresolved direction is neglected.
   Split parameters may be sharpened because they are free choices.
5. Retain small boxes as candidates for local refinement; continue searching
   every other box. `min_subdivision_size` is a handoff threshold.

Rebuilding matters: clipping $t$ reduces the rows unioned for $s$, allowing
further contraction. A single round cannot recover this coupling. Using
only a spatial overlap test would miss this parameter information entirely.

An example also shows the limitation of the projections. For
$A(s)=(s,s,0)$ and $B(t)=(t,1-t,0)$ on $[0,1]$, the residual equations are
$s-t=0$ and $s+t-1=0$. Each equation separately projects onto the whole of
both parameter intervals, so even exact projected hull clipping does not
contract. Together they have the unique solution $(1/2,1/2)$. Subdivision
or a coupled local contractor supplies information the projections lose.

## 4. Refine isolated candidates without inventing a root

Use

$$
F(s,t)=A(s)-B(t),\qquad J=[A'(s)\;-B'(t)].
$$

At a transversal crossing the two columns are independent. In 2-D this is
a square system; in 3-D it is overdetermined. Gauss–Newton can find a useful
seed but can also converge to a closest pair with nonzero residual. Solving
two coordinate equations in 3-D does not establish the third.

To preserve the cover, tighten only with an interval Newton/Krawczyk or other
verified contractor enclosing all zeros in the incoming box. A square
subsystem can contract it, but all remaining equations still need checking;
zero belonging to their interval ranges is only a necessary condition for
existence. Do not interpret a small least-squares residual as intersection.
Keep sharpened seeds separate from unsharpened enclosures. Ordinary Newton
updates are not automatically certified enclosures even if left unsharpened.

Parallel tangents make the Jacobian rank deficient despite the absence of
coincidence. Retain the incoming box if refinement is singular, leaves the
domain, or cannot be verified; continue subdivision or report it unresolved.
Tangencies and uncertain input data can prevent certification at finite
budget. The no-coincidence assumption does not remove this limitation.

## 5. Distinct answers and completeness

Different roots must remain in different boxes. Overlapping boxes from
adjacent subdivisions may describe one root, but overlap alone does not
prove that: a chain of boxes can cover several nearby roots. Merge them by
componentwise union only as an unresolved cluster, or after establishing
that they enclose the same root. Never average parameters or spatial points.
Boundary roots must survive in at least one incident piece.

Even distinct spatial intersections can have multiple parameter pairs at
seams or self-intersections. Keep these preimages until topology or a proven
identity permits grouping them. Degenerate constant pieces can even produce
nonisolated parameter roots without a shared spatial arc.

A small surviving box is a candidate, not an existence or uniqueness proof.
If returning certified distinct intersections is required, certify each
root and resolve all remaining boxes. Spatial ranges must be computed over
whole boxes with valid span-local evaluation or control hulls.

An empty completed search proves no intersection. Exhausting `max_nodes` or
an output cap means incomplete, not empty or coincident. In particular, the
current `Intersections::Coincident` convention of reaching `max_solutions`
is not the contract for this no-coincidence design: an implementation would
need an explicit incomplete/error outcome or a cover of unresolved boxes.
