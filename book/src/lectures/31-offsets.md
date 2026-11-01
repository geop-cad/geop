# Lecture 31: Offsetting a surface

<div class="goals">

**After this lecture you can**

- define the offset of a curve or surface, and show why it is generally not a NURBS;
- predict where an offset is regular, where it folds and where it collapses, from the curvature;
- explain the kernel's fitted offset: same parametrization, a shift fitted in the surface's own basis, and a bound proven by polynomials;
- derive why circles, cylinders, spheres and tori come out exact through this general path;
- state two ways a bound's locality can be thrown away, with the kernel's numbers;
- derive where a cone's offset closes in its apex.

</div>

The previous lecture called `NurbSurface3D::offset` for every face a shell keeps. A fillet's ball lives on offsets without building them ([Lecture 29](./29-fillets.md)). It looks simple: move every point by `$d$` along the normal. It is in fact the textbook example of an operation that leaves the class of surfaces you started in, and it is where interval honesty pays most directly.

## The definition, and why it leaves NURBS

For a surface `$S(u,v)$` with unit normal `$N$`, the **offset** at distance `$d$` is

`$$O(u,v)=S(u,v)+d\,N(u,v),\qquad N=\frac{S_u\times S_v}{\lvert S_u\times S_v\rvert}.$$`

The trouble is the square root in `$\lvert S_u\times S_v\rvert$`. Take the simplest curved case in the plane, the parabola `$S(t)=(t,t^2)$`. Then

`$$N(t)=\frac{(-2t,1)}{\sqrt{1+4t^2}},$$`

and `$\sqrt{1+4t^2}$` is no rational function of `$t$`. So the offset of a polynomial curve is not a rational curve: it cannot be a NURBS at all. There are exceptional polynomial curves whose hodograph has a polynomial norm (Pythagorean hodograph curves, after Farouki and Sakkalis), and we shall see below that circles and their relatives have the same luck, but in general we cannot have an exact representation. Three choices remain: store offsets symbolically (a new surface kind, which AGENTS.md excludes), approximate them to within a tolerance and trust it, or approximate them and **enclose**. Geop takes the third.

## Curvature decides

Before fitting anything, ask what kind of object the offset is. In the plane, differentiate `$O=S+dN$`. With `$N$` the left normal, signed curvature `$\kappa$` and `$T$` the unit tangent, Frenet gives `$dN/ds=-\kappa T$`, so

`$$O'(t)=S'(t)\,(1-d\,\kappa(t)).$$`

The offset runs parallel to the curve, scaled by the factor `$1-d\kappa$`. Therefore:

- while `$1-d\kappa>0$` it is **regular**, running the same way;
- at `$1-d\kappa=0$` it has a **cusp**: `$d=1/\kappa$` is the radius of curvature, and the offset point is the centre of curvature;
- where it is negative the offset runs backwards: it **folds**.

<figure class="demo" data-demo="offset-curve">
<figcaption>A cubic curve (black), its exact offset (thick green, point by point) and a C¹ piecewise cubic fit (thin orange). Move the distance towards the curve's inside, negative here: when it passes the radius of curvature, where the comb is tallest, red circles mark cusps and the offset folds. Add pieces to the fit: the measured miss falls towards sixteen-fold per doubling. The kernel measures nothing: it proves a bound, as described below, and its fit is in the surface's own basis.</figcaption>
</figure>

On a surface the same statement holds for each principal direction: along a principal direction with curvature `$\kappa_i$` the offset scales the surface by `$1-d\kappa_i$`. So, in the terms used in `core/geop-core-geometry/src/nurb_surface/offset_fit.rs`:

- **regular**, where both factors are positive;
- **collapsed**, where one of them is not, *anywhere*: every point has passed its centre of curvature, such as a cylinder shrunk past its axis;
- **folded**, where that changes within the surface, and that is an error, not an offset.

How can the kernel decide "both factors positive" over a whole patch, with no sampling? In the tangent basis the factors are the eigenvalues of `$I^{-1}(I-d\,II)$`, where `$I,II$` are the first and second fundamental forms. So regular means that `$I-d\,II$` is positive definite, meaning its first entry and its determinant are positive. The entries multiplied through by `$w^4$`, `$G_{ab}=P_a\cdot P_b-d(w^2/\lvert m\rvert)J_{ab}$`, are polynomials with an interval factor, whose sign can be read from their Bernstein coefficients: positive, negative, or undecided, and if undecided, split the box. `Shape::regular` does this, and `fitted_offset` then has three verdicts from which `offset` returns:

- `None` where collapsed: "a blend narrower than the walls a shell builds are thick", which the shell then contracts;
- an error where it folds, naming the two regions: "regular over ..., but passes ...";
- the fit otherwise.

## The fit

`offset`'s doc comment gives the contract: the result has "the same parametrization: the offset of the point at `$(u,v)$` is at `$(u,v)$` of the result, so a trim loop's `$(u,v)$` stay meaningful on it". So fit in `$S$`'s own space.

Write `$S=A/w$`, its homogeneous polynomial numerator over its weight, and look for the offset as `$S+D$` where the shift `$D=e/w$` has `$S$`'s denominator. The target for `$D$` is `$dN$`, so the target for the homogeneous control points of `$e$` is `$w\,d\,N$`. `fitted_shift` interpolates `$w\,d\,N$` at the **Greville points** of the basis, solving one linear system per direction (`solve_interpolation_system`). Then `$S+D$` has control points `$A$`'s moved by `$e$`'s, and weights unchanged.

This is where the kernel's general approach earns its keep. When is that fit exact? When `$wN$` is itself a polynomial of the basis. And that's precisely what happens for the classical shapes:

**A plane.** `$N$` is constant: `$wN=w\cdot\text{const}$`. The shift is a translation of every control point.

**A circle or cylinder.** Let `$S=A/w$` be a circular arc with centre `$c$` and radius `$R$`. Then `$N=(S-c)/R$`, so `$wN=(A-cw)/R$`, which is a polynomial of the same degree as `$A$`. So the fit is exact, and works out to `$O=c+(1+d/R)(S-c)$`: control points are scaled about the centre, and the weights are unchanged. A rational quadratic's offset is a rational quadratic in the same basis, with no special code.

The same reasoning covers the sphere, whose normal is `$(S-c)/R$`, and a torus, whose normal is that of the tube's circle; on a cone `$N$` depends only on the angle round the axis. The module says it directly: the fit is "exact to the rounding where the offset is a NURBS of that basis, as a plane's, a cylinder's, a cone's, a sphere's or a torus's is, and no branch on what kind of surface it is." As the tests `cylinders_change_radius`, `spheres_change_radius` and `tori_change_tube_radius` in `core/geop-core-geometry/src/nurb_surface/offset.rs` confirm.

Otherwise the fit "cannot follow the offset", and it is redone "refined and raised to cubic", up to `MAX_OFFSET_PIECES` pieces per span, in pursuit of `OFFSET_DRIFT` (`1e-6`).

## The bound

A fit alone is an approximation. To be an enclosure, we must bound `$E=D-dN$` over the whole domain, and widen control points by it. Sampling `$E$` proves nothing. Here is how to bound it from polynomials alone.

Over one span, `$S=A/w$` and `$D=e/w$`. The normal is `$N=m/\lvert m\rvert$`, with `$m$` a *polynomial* field, `$S_u\times S_v$` clear of denominators and with a pole's factor divided out (`normal_field`). Split `$D$` into parts along and across the normal:

`$$D=a\,N+b,\qquad b\perp N.$$`

Then, using the cross product with `$m$`:

`$$\lvert b\rvert^2=\frac{\lvert e\times m\rvert^2}{w^2\,m\cdot m},\qquad a^2-d^2=\frac{e\cdot e-d^2w^2}{w^2}-\lvert b\rvert^2.$$`

Where `$a$` has the sign of `$d$`,

`$$\lvert a-d\rvert=\frac{\lvert a^2-d^2\rvert}{\lvert a+d\rvert}\le\frac{1}{\lvert d\rvert}\left(\frac{\lvert e\cdot e-d^2w^2\rvert}{w^2}+\lvert b\rvert^2\right),$$`

and so `$\lvert E\rvert\le\lvert a-d\rvert+\lvert b\rvert$` is bounded by *polynomials alone*: `$e\times m$`, `$e\cdot e-d^2w^2$`, `$m\cdot m$` and `$w$`, each computed as a polynomial product by `Net` in `core/geop-core-geometry/src/nurb_surface/net.rs`. Notice that where the fit is exact, `$e\times m$` and `$e\cdot e-d^2w^2$` are identically zero as polynomials, and the bound is exactly the rounding. Each polynomial's range over a box comes from its Bernstein coefficients' hull. Lecture 6's convex hull property, one more time. A box whose bound is wider than `OFFSET_DRIFT` is split in four, and the hulls close in on the polynomials' ranges as the square of the box's size.

## A bound is as local as what it bounds

Two mistakes here cost a great deal of accuracy, and each of them is typical.

**One pad for the whole patch.** A shell gives each surface room (Lecture 30), carrying it on past its sides, and a rational arc carried on has weights that fall off. Its bound there was a thousand times wider than over the face. Padding every control point by the largest made a sphere's vertex `1e-7` wide (AGENTS.md). But a control point's basis function is zero outside a few spans. So a control point needs padding only by the bounds of the boxes it covers (`shifted`). The surface is still widened by at least the bound everywhere, since the basis is non-negative and sums to one.

**A ratio as the numerator's hull over the denominator's.** Bounding `$\lvert e\times m\rvert/(w\lvert m\rvert)$` by the largest numerator over the least denominator across a box multiplies the bound by how much the weight changes across it. There's a much better way. For positive `$d_i$`,

`$$\frac{\sum_iB_ia_i}{\sum_iB_id_i}\le\max_i\frac{a_i}{d_i},$$`

because `$\sum_iB_ia_i\le\max_i(a_i/d_i)\sum_iB_id_i$`. The ratio of Bernstein coefficients is as tight as either hull and costs nothing (`ratio_bound`). AGENTS.md: the vertex went from `6e-11` to `1.4e-12`.

The same holds when evaluating a rational surface over a box of `$(u,v)$`: `$A/w$` over a box divides two enclosures that move together. A shell's copy of a traced edge, fitted through foot points evaluated over the boxes they project to, came out eight times as wide as the edge, and two offsets in a row went past the `1e-4` an entity may carry (AGENTS.md). `NurbSurface3D::evaluate` now intersects the direct enclosure with the mean value form (the middle plus the partials over the box times the half widths). Both enclose the same points, and Lecture 5 says to intersect.

## An apex has no offset: close it where the offset does

A cone's tip is a pole with no tangent plane. The normal turns all the way round there, so "`$S+dN$`" gives a circle around the apex, not a point. On one side of the cone (`$d$` inward), the offset's rows pass through each other across the axis, a fold. On the other (`$d$` outward) they leave a hole around it. What a shell or a moved face wants is the offset cone, closed in its own apex.

So: where does a row close? Let `$t$` run along a row (around the cone), and `$s$` across it. The offset's row has tangent `$O_t=S_t+dN_t$`, and `$N_t$` along a principal direction is `$-\kappa_tS_t$`, with `$\kappa_t=N\cdot S_{tt}/S_t\cdot S_t$` the normal curvature along the row. Thus

`$$O_t\cdot S_t=S_t\cdot S_t-d\,N\cdot S_{tt}.$$`

That is the expression `row_turning` computes: "Positive where the offset's row runs the way the surface's does, zero where it closes to a point, negative where it has turned over." Nothing in it knows about cones. `closes_inside` searches for the zero of this function across the patch, and `closes_at_apex` handles the apex.

**A worked example.** A right cone with half-angle `$\alpha$` and apex at the origin, opening up: at distance `$\rho$` from the axis, height `$z=\rho\cot\alpha$`. Parametrize a row by the angle `$t$`. Then `$S_t\cdot S_t=\rho^2$`. `$S_{tt}=-\rho\,e_r$`, and the outward normal `$N=\cos\alpha\,e_r-\sin\alpha\,e_z$` gives `$N\cdot S_{tt}=-\rho\cos\alpha$`. So

`$$O_t\cdot S_t=\rho^2+d\rho\cos\alpha=\rho(\rho+d\cos\alpha),$$`

which, for `$d<0$`, vanishes at `$\rho=\lvert d\rvert\cos\alpha$`. That is the row at which the inward offset closes. It should reach the axis there, and it does: the offset's radial component there is `$\rho+d\cos\alpha=0$`. Its height is

`$$z'=\rho\cot\alpha+\lvert d\rvert\sin\alpha=\lvert d\rvert\frac{\cos^2\alpha+\sin^2\alpha}{\sin\alpha}=\frac{\lvert d\rvert}{\sin\alpha},$$`

so the inner cone's apex is a distance `$\lvert d\rvert/\sin\alpha$` up the axis, as elementary geometry confirms. For `$d>0$` the zero is at `$\rho=-d\cos\alpha<0$`, past the apex: the row is on the continuation of the cone through its tip.

So the algorithm is: cut the surface back to that row (`$d$` inward), or *carry it on through the apex* to that row (`$d$` outward), the normal continued through the apex with it; reparametrize over the old domain so the apex stays where it was in `$(u,v)$`; fit; and check on the offset that the row did close. `Closing` records which side closed and where the apex lies across it. Past an apex carried on inside the patch, one factor `$1-d\kappa$` changes sign together with the surface turning over: "that is the offset keeping its way round, not a fold", which `flipped` accounts for. An apex whose offset does not close in a point (an elliptic cone) is refused by name.

## Straight in `$(u,v)$` is not preserved

The kernel keeps a trim loop's `$(u,v)$` under the offset, except at an apex. A related trap: AGENTS.md's *Degenerate parametrizations* tells of the shell keeping an inner trim curve straight whenever the original was straight. That holds on a plane (affine coordinates) and along an iso-line. It fails for a sphere's meridian cut by a plane: the inner copy is a small circle. Hence the code's `iso_line` in `lib/geop-lib-shell/src/rebuild.rs`, which reads a straight pcurve as a control row only when its ends lie exactly on one parameter value.

## Using it: thicken, offset surface

The operations are thin over this: `offset_faces` and `thicken` in `lib/geop-lib-shell/src/shell.rs` take a sheet, and `OffsetSurface` and `Thicken` in `ops/geop-ops-surface/src/` offer them. Each offsets faces and rebuilds ([Lecture 30](./30-one-rebuild.md)), and the wall joining a free edge to its inner copy is `NurbSurface3D::ruled` between them.

For curves: `NurbCurve2D::offset_miss` in `core/geop-core-geometry/src/nurb_curve/offset.rs` is the plane-curve counterpart, bounding how far a fitted curve is from a curve's offset by the mean value form on boxes. It uses the same `$N'$` as above and serves the sketch offset of [Lecture 25](./25-sketch-features.md).

<div class="recap">

## Recap

- `$O=S+dN$` has a `$\sqrt{\ }$` in `$N$`, so the offset of a NURBS is generally no NURBS.
- Along a principal direction the offset scales by `$1-d\kappa$`. It is regular, collapsed or folded by the sign of that, proven via polynomial sign tests.
- Fit `$S+D$` in `$S$`'s own basis, with `$wdN$` interpolated at Greville points. Where `$wN$` is a polynomial of the basis (planes, cylinders, cones, spheres, tori), the fit is exact, with no branch on surface kinds.
- Bound the miss by polynomials only, over boxes, and widen each control point by what *its own* basis function covers.
- Ratios: take the largest ratio of Bernstein coefficients, not hull over hull.
- A cone's apex has no offset. Find where `$S_t\cdot S_t-dN\cdot S_{tt}$` vanishes, cut back or carry on to that row, and close there.

</div>

## Exercises

**1.** Show that a line's offset is exactly another line, and a parabola's isn't rational, by writing `$\lvert S'\rvert$`.

<details class="solution"><summary>Solution</summary>

For `$S=P+tv$`, `$\lvert S'\rvert=\lvert v\rvert$` is constant, so `$N$` is constant and `$O$` is a translation. For `$(t,t^2)$`, `$\lvert S'\rvert=\sqrt{1+4t^2}$`, and `$1+4t^2$` has no rational square root since it isn't a perfect square in `$\mathbb{R}(t)$`. A rational `$O$` would need `$N=(O-S)/d$` rational, and hence `$\sqrt{1+4t^2}$` rational.

</details>

**2.** Derive `$O'=S'(1-d\kappa)$` in the plane, starting from `$N=(-y',x')/\lvert S'\rvert$`. Check it for a circle of radius `$R$` traversed counter-clockwise, with `$d=-1$` (so `$d$` is towards the outside, given the left normal points inward).

<details class="solution"><summary>Solution</summary>

`$dN/dt=\lvert S'\rvert\,dN/ds=-\lvert S'\rvert\kappa T=-\kappa S'$`. So `$O'=S'+d(-\kappa S')=S'(1-d\kappa)$`. For the circle, `$\kappa=1/R$`, and with `$d=-1$`: `$O'=S'(1+1/R)$`, a circle of radius `$R+1$`.

</details>

**3.** A circular arc with control points `$(1,0),(1,1),(0,1)$`, middle weight `$\tfrac{\sqrt2}{2}$`, centre at the origin. Using `$O=c+(1+d/R)(S-c)$`, write its offset at `$d=0.5$`, with the normal outward. Check that `$(wN)$` is a polynomial of the same degree.

<details class="solution"><summary>Solution</summary>

`$R=1$`, factor 1.5: control points `$(1.5,0),(1.5,1.5),(0,1.5)$`, weights unchanged. And `$wN=w\cdot S=A$`, the homogeneous numerator (since `$c=0$`, `$R=1$`), of degree 2 in the same basis.

</details>

**4.** Bound `$\dfrac{1+t}{1+9t}$` over `$[0,1]$` two ways: by hull over hull, and by Bernstein coefficients. Which is tighter, and what is the true maximum?

<details class="solution"><summary>Solution</summary>

Numerator coefficients `$(1,2)$`, denominator `$(1,10)$`. Hull over hull: `$2/1=2$`. Largest ratio: `$\max(1/1,2/10)=1$`. True maximum at `$t=0$` is 1, so the second bound is tight. As the weight's range grows, so does the first bound's overestimate, without limit.

</details>

**5.** For a cone with `$\alpha=30°$` and an inward offset `$\lvert d\rvert=1$`: at what row `$\rho$` does the offset close, how far up the axis is the new apex, and what's the distance from there to the old cone's surface? Verify that it is 1.

<details class="solution"><summary>Solution</summary>

`$\rho=\cos30°=0.866$`. New apex at `$z'=1/\sin30°=2$`. Distance from `$(0,2)$` in the `$(\rho,z)$` half-plane to the line `$z=\rho\cot30°=1.732\rho$`, whose unit normal is `$(\cos30°,-\sin30°)$`: `$\lvert0\cdot0.866-2\cdot0.5\rvert=1$`, as required.

</details>

**6.** Find `MAX_OFFSET_BOXES` and `BOUND_SPLITS` in `offset_fit.rs`. For each, say whether it affects *what the enclosure means* or only how hard it tries. What could the result look like if both run out?

<details class="solution"><summary>Hint</summary>

Both only bound effort. The doc comment on `BOUND_SPLITS` says: "Bounds effort and how wide the offset may come out, not whether it encloses the true one." Running out of splits leaves a wider, but still valid, enclosure. Running out of boxes makes the offset "refused as not enclosed". Neither returns a tighter-looking wrong answer, which is Lecture 5's rule for budget parameters.

</details>
