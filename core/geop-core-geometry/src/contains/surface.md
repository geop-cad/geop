# Surface containment by per-axis clipping

Design extension of [curve containment](curve.md). Implemented in
[`surface.rs`](surface.rs) (benchmarked against the previous hull-and-bisect
[`surface_bisect.rs`](surface_bisect.rs) by `examples/surface_contains_bench.rs`), with these
choices:

- Clip B only; Clip A is not implemented.
- Positive weights are checked per patch; if the check fails, that patch gets
  no clip and plain subdivision takes over.
- Convergence uses the AABB of the control net (the enclosing range), then
  requires the geometric hull test to pass as well.
- A direction pinned exactly onto a clamped domain end (§3) is searched as a
  boundary curve with `curve::curve_could_contain`.
- Stalled patches bisect the direction that is widest relative to the
  surface's own domain.
- Budget exhaustion is an error (§5's "report incompleteness"), in `curve.rs`
  as well.
- Verified local refinement (§5) is not implemented; converged boxes are
  reported as candidates.

The same construction also supplies the clipping step for
[curve–curve](../intersection/curve_curve.md) and
[curve–surface](../intersection/curve_surface.md) intersection.

Given a tensor-product NURBS surface and a point $p$, enclose

$$
R = \{(u,v) \in U\times V : S(u,v)=p\}.
$$

Keep a collection of parameter boxes covering $R$. A single pair of intervals
can represent their componentwise union, as the existing containment API does,
but loses separation between different preimages. A surviving box means
"could contain", not proof that a solution exists. Empty means definitely
absent only after every part of the domain has been rejected.

## 1. Remove the rational denominator

Write the homogeneous control net as $P_{ij}=(H_{ij},w_{ij})$, with degrees
$p_u,p_v$ and bases $N_i(u),M_j(v)$:

$$
S(u,v)=\frac{H(u,v)}{W(u,v)},\qquad
(H,W)=\sum_{i,j}P_{ij}N_i(u)M_j(v).
$$

Require definitely positive weights on the patches to which this construction
is applied; then $W>0$. This is a precondition to check, not an assumption to
make merely because an object is a `NurbSurface`. More generally a denominator
proved nonzero suffices for the algebra below. An unresolved denominator
requires subdivision or an explicit unresolved result.

For each Cartesian axis $k$:

$$
S_k(u,v)=p_k \iff g_k(u,v)=0,\qquad
g_k=\sum_{i,j}d_{k,ij}N_iM_j,\qquad
d_{k,ij}=P_{ij,k}-p_k w_{ij}.
$$

These are polynomial tensor-product splines. Compute their coefficients
directly from homogeneous coordinates, without dehomogenizing control points.
All arithmetic, including the uncertain point $p$, uses interval `Scalar`s.

## 2. Project the graph onto each parameter axis

For positive degrees let $\xi_i$ and $\eta_j$ be the Greville abscissae of
the two knot vectors, computed as in the curve note. Linear precision gives

$$
(u,v,g_k(u,v))=\sum_{i,j}(\xi_i,\eta_j,d_{k,ij})N_i(u)M_j(v).
$$

The graph lies in the convex hull of this control net. Projecting onto the
$(u,g_k)$ plane gives the points $(\xi_i,d_{k,ij})$; projecting onto
$(v,g_k)$ gives $(\eta_j,d_{k,ij})$. Any zero must lie within the zero-height
section of both projected hulls.

There is no need to construct a three-dimensional hull. Collapse the unused
index by interval union:

$$
D^{u}_{k,i}=\bigcup_j d_{k,ij},\qquad
D^{v}_{k,j}=\bigcup_i d_{k,ij}.
$$

At fixed $v$, $\sum_j d_{k,ij}M_j(v)\in D^u_{k,i}$, since the basis is
nonnegative and sums to one. Thus the one-dimensional envelope with control
intervals $(\xi_i,D^u_{k,i})$ encloses every possible slice. The analogous
claim holds in $v$.

Apply Clip A or Clip B from [curve.md](curve.md) to these two lists. Call the
results $\hat U_k$ and $\hat V_k$. Then every solution in the current box
lies in

$$
\hat B=\left(U\cap\bigcap_k\hat U_k\right)
       \times\left(V\cap\bigcap_k\hat V_k\right).
$$

Use **union** over eliminated indices (alternative possibilities), and
**intersection** over Cartesian equations (simultaneous constraints).
Intersecting the clips of individual control rows would be wrong: a surface
zero need not be a zero of every row, or even of any row.

For degree zero in a parameter, linear precision via Greville abscissae is
unavailable. Work span by span: the function is constant in that parameter
on each span, so range tests can reject a span but cannot narrow its interior.

## 3. The cheap clip is still a fat line

For either projected list write its abscissae as $q_i$ and coefficient
intervals as $D_i$, on a clamped restricted domain $X$. Clip B is unchanged:

$$
s=\frac{D_n-D_0}{q_n-q_0},\qquad
\ell(x)=D_0+s(x-q_0),\qquad
\Delta=\bigcup_i\bigl(D_i-\ell(q_i)\bigr).
$$

First reject if $(D_0\cup D_n)+\Delta$ excludes zero. Otherwise, when $s$
is definitely nonzero,

$$
\hat X=X\cap\left(q_0-\frac{D_0+\Delta}{s}\right).
$$

If a denominator could be zero, retain $X$ or use Clip A; never divide
through zero. A collapsed domain is handled as a boundary evaluation.
Use the signed offset union, not a symmetric tolerance. Clip A's conservative
pairwise rules also apply to interval abscissae; do not sort or discard hull
vertices on an uncertain orientation test.

Forming all row/column unions and Clip B costs $O(n_u n_v)$ per Cartesian
axis. Clip A on the collapsed lists costs $O(n_u^2+n_v^2)$ in addition.
Projection discards correlation between $u$ and $v$, so it is weaker than
clipping the full graph hull by $g_k=0$, but reuses the curve construction.

## 4. Restrict, recompute, subdivide

1. Start with the full domain, or its tensor-product knot-span patches.
   Keep the existing AABB and geometric hull rejection tests as additional
   necessary conditions.
2. Compute the coefficients, project, and intersect the clips. Reject if
   either parameter interval is empty.
3. If the box contracts usefully, restrict the surface with `split_u` and
   `split_v` at the retained intervals' **outer bounds**. Recompute the
   control net and its clips: narrowing $V$ tightens the next $u$ projection,
   and vice versa. Maintain the original parameter coordinates.
4. If clipping stalls, bisect a noncollapsed parameter direction and keep
   both children. A fair rule must eventually split every direction that
   remains unresolved. The curve note's 20% rule is an optional scheduling
   heuristic, never an acceptance criterion.
5. Once boxes are small enough for the intended handoff, retain them as
   candidates or attempt verified local refinement. Do not drop other
   pending boxes after finding the first candidate.

Subdivision midpoints are free choices and may be sharpened. Computed answer
boxes may not. At knot boundaries retain all incident pieces; interval
evaluation must enclose the whole box, using span-local evaluation or control
hulls rather than a de Boor branch that covers only one span.

## 5. Containment, refinement, and termination

An overlapping range in all three coordinates is necessary, not sufficient:
different equations may vanish at different parameters. Even a tiny box can
be a false positive. Small corner chords also do not bound the extent of a
folded patch; use enclosing ranges if spatial size controls the handoff.

For local refinement use $F=S-p$ and $J=[S_u\;S_v]$. On a regular patch this
is a three-equation, two-unknown system. Newton or Gauss–Newton supplies a
seed, but a nearest foot point need not satisfy $S=p$. To replace a proven
cover by a smaller cover, use an interval root contractor valid over the
entire incoming box. A final unsharpened ordinary Newton step alone does not
prove that it encloses every root. Failure to verify refinement retains the
incoming box; it does not reject it. Check all three equations, including
any equation omitted to form a square subsystem.

`min_subdivision_size` controls when to hand off, not what equality means.
If `max_nodes` runs out, report incompleteness/error or include every pending
box in an explicitly unresolved cover. Never turn an unfinished search into
"definitely not contained". This stronger completeness contract would require
changing the current `surface.rs` budget behavior when implementing the note.

At a pole or collapsed surface direction, a single spatial point can have
an entire interval of preimages. Keep that parameter uncertainty: neither
clipping nor Newton can legitimately turn it into a unique sharp pair.
