# Curve–surface intersection by per-axis clipping

Design extension of [curve containment](../contains/curve.md). Implemented
in [`curve_surface.rs`](curve_surface.rs), with the same choices as
`curve_curve.rs`:

- Clip B only, on free-choice combinations of the axis equations instead of
  the axes themselves: the patch normal $e_u\times e_v$ (pins $t$), and
  $c\times e_v$ and $c\times e_u$ (pin $u$ and $v$), from the curve chord
  $c$ and the patch's mean edges. The axes are used only if these are
  degenerate.
- A pinned $t$ becomes `contains::surface` containment of the curve's endpoint; a
  pinned $u$ or $v$ becomes `curve_curve_crossings` against that boundary row.
- Exhausting `max_nodes` is an error.
- Refinement (§4) is not implemented.
- The public `curve_surface_intersect` wraps `curve_surface_crossings` the same
  way. Its candidates are the curve's ends found on the surface, plus where the
  curve meets the patch's four boundary curves (via the curve–curve wrapper, so
  a curve running along a boundary counts). It tests the midpoint between
  consecutive candidates for containment in the surface. Assume intersections are distinct
spatial points, with no arc of the curve lying on the surface. This still
allows isolated tangencies. The search operates on the untrimmed NURBS patch;
membership in a trimmed face is an additional condition.

## 1. A polynomial spline in three parameters

Write the homogeneous curve and surface as

$$
(H_C,W_C)=\sum_i P_iN_i(t),\qquad
(H_S,W_S)=\sum_{j,l}Q_{jl}M_j(u)L_l(v),
$$

so $C=H_C/W_C$ and $S=H_S/W_S$. Both denominators are positive on the whole
domain: `NurbCurve::try_new` and `NurbSurface::try_new` only accept
definitely positive weights (see [curve containment](../contains/curve.md)).

For $k\in\{x,y,z\}$, cross multiplication gives

$$
C_k(t)=S_k(u,v)\iff
g_k(t,u,v)=H_{C,k}(t)W_S(u,v)-W_C(t)H_{S,k}(u,v)=0,
$$

with coefficients

$$
d_{k,ijl}=P_{i,k}Q_{jl,w}-P_{i,w}Q_{jl,k},\qquad
g_k=\sum_{i,j,l}d_{k,ijl}N_i(t)M_j(u)L_l(v).
$$

The independent parameters make this a tensor-product spline of degrees
$(p_C,p_u,p_v)$ without multiplying basis functions in the same variable or
aligning knot vectors. No division by a control-point weight is needed.

Enclose the simultaneous zero set in $T\times U\times V$ with a list of
paired triples of intervals. Each triple keeps a curve parameter attached
to its surface preimage.

## 2. Project the graph and reuse the curve clips

Let $\xi_i,\eta_j,\zeta_l$ be the three Greville sequences. Linear precision
and partition of unity give

$$
(t,u,v,g_k)=\sum_{i,j,l}(\xi_i,\eta_j,\zeta_l,d_{k,ijl})N_iM_jL_l.
$$

Although this graph is four-dimensional, the cheap clips need only its
two-dimensional projections. Collapse all indices except the chosen one:

$$
D^t_{k,i}=\bigcup_{j,l}d_{k,ijl},\qquad
D^u_{k,j}=\bigcup_{i,l}d_{k,ijl},\qquad
D^v_{k,l}=\bigcup_{i,j}d_{k,ijl}.
$$

Apply Clip A or Clip B from [curve.md](../contains/curve.md) to
$(\xi_i,D^t_{k,i})$, $(\eta_j,D^u_{k,j})$, and $(\zeta_l,D^v_{k,l})$.
There are nine clips: three Cartesian equations for each of three parameter
directions. Their intersections give

$$
\hat B=\left(T\cap\bigcap_k\hat T_k\right)
\times\left(U\cap\bigcap_k\hat U_k\right)
\times\left(V\cap\bigcap_k\hat V_k\right).
$$

The proof is identical to the surface-containment projection: fixing the
other parameters forms convex combinations of the eliminated coefficients,
all enclosed by their union. Thus every true root survives every clip.
Taking a midpoint or intersecting the eliminated coefficients would destroy
that guarantee.

For Clip B use the signed fat-line offset union and divide only by a
definitely nonzero slope. A horizontal or uncertain slope contributes no
contraction unless the range test excludes zero. Retain uncertain candidates
in Clip A as specified in the original note. Zero-degree spans and collapsed
domains require the constant-span/boundary treatment in the surface note.

Forming the coefficients and all Clip B projections costs
$O(n_C n_u n_v)$ per Cartesian axis. Stream the tensor into the three sets
of unions if storage matters. Clip A then adds
$O(n_C^2+n_u^2+n_v^2)$. Projected hulls lose coupling between parameters;
the three factors of $\hat B$ are not independently solved equations.

## 3. Restriction and subdivision

1. Queue the original curve/patch pair or their knot-span pairs. Reject
   spatially disjoint bounds before constructing the tensor.
2. Clip all directions; reject if any retained interval is empty.
3. Restrict the curve in $t$ and the surface in $u,v$ at the clips' outer
   bounds. Rebuild coefficients and repeat while contraction is useful.
4. If clipping stalls, bisect one noncollapsed direction and keep both
   children. Use a fair split schedule; absolute widths in differently
   scaled parameter domains need not represent comparable geometric spans.
5. Hand small candidate boxes to local refinement when accuracy is needed,
   keeping the other boxes in the global search.

All restrictions preserve original parameter coordinates. Midpoint splits
are free choices and may be sharpened; candidate intervals may not. Keep
shared-boundary roots covered. If bounds are obtained by interval evaluation,
ensure the evaluator covers the entire knot-span box; split at knots or use
control hull bounds when one evaluator branch is insufficient.

For example, take $C(t)=(1/4,3/4,2t-1)$ and $S(u,v)=(u,v,0)$ on unit
domains, with unit weights. Then

$$
g_x=1/4-u,\qquad g_y=3/4-v,\qquad g_z=2t-1.
$$

The projected clips isolate $u=1/4$, $v=3/4$, $t=1/2$ directly, up to
interval rounding. If an oblique or curved configuration couples the
parameters, restrictions and recomputation progressively recover information
lost by the unions. Stalling is a reason to subdivide, not evidence of
coincidence.

## 4. Local refinement and transversality

For a candidate box use

$$
F(t,u,v)=C(t)-S(u,v),\qquad
J=[C'(t)\;-S_u(u,v)\;-S_v(u,v)].
$$

This is a square three-equation system. Its determinant is
$C'\cdot(S_u\times S_v)$, so a regular transversal crossing has a nonsingular
Jacobian. At a tangency, or a singular surface parameterization, it may be
singular even though the spatial intersection is isolated.

A verified local contraction can use a sharp seed $x_0$ in a box $B$, a
fixed approximate inverse $Y$, and an interval Jacobian enclosure $J(B)$:

$$
K(B)=x_0-YF(x_0)+(I-YJ(B))(B-x_0).
$$

Every root in $B$ lies in $K(B)$, by the mean-value relation, provided the
function and Jacobian evaluations are valid on the whole box. Thus
$B\cap K(B)$ is a sound contraction; a verified empty intersection excludes
roots. On smooth fixed-input systems, the standard Krawczyk inclusion and
nonsingularity conditions can additionally establish existence and uniqueness.
Uncertain input families require the corresponding quantified certification;
a small box alone does not certify a root for every possible geometry.

An ordinary Newton iteration is useful for choosing $x_0$, but its last
update is not automatically an enclosure of every solution. Sharpen only the
seed; return the verified, unsharpened box. If constructing a verified
contraction fails, retain the incoming box. Distinguish such failure from a
valid exclusion proof. Across nonsmooth knots use separate span boxes rather
than assuming one smooth Jacobian formula applies.

Refine where a parameter becomes a split point or otherwise needs accuracy;
do not unconditionally change every caller's search result. A singular
candidate can remain unresolved or be subdivided further. No epsilon can
turn it into a proven crossing or prove its absence.

## 5. Results and completion

`min_subdivision_size` controls the handoff, and `max_nodes` controls work.
Neither defines equality or proves that two candidates are the same root.
Use enclosure widths, not corner chords alone, to measure spatial extent:
a patch can have coincident corners and a large interior.

Keep distinct candidates separate. Shared subdivision boundaries can produce
duplicates; overlapping parameter boxes may be unioned as an unresolved
cluster, but only a proven common-root identity justifies counting the cluster
as one intersection. Surface seams and poles may give multiple or even
continuous parameter preimages of a single spatial point. No spatial
coincidence of a curve arc is needed for this degeneracy.

When producing a spatial point from an established root enclosure, preserve
the disagreement between the curve and surface evaluations with `union`,
never an average. A point near a trimmed face is not necessarily in it;
perform the face's trim-membership check separately when used in topology.

A completed empty cover proves there is no intersection. A nonempty cover
contains candidates until existence and distinctness are established. If a
budget or output cap is reached with pending boxes, return an explicit
incomplete/error result or retain those boxes as unresolved. Reaching
`max_solutions` cannot mean coincidence under this design's premise; the
current `Intersections::Coincident` convention would need a different
completion contract for this algorithm.
