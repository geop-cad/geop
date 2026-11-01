# Lecture 10: Surfaces

<div class="goals">

**After this lecture you can**

- define a tensor-product NURBS surface from a net of control points and two knot vectors;
- evaluate one by two rounds of de Casteljau or de Boor, and compute its partial derivatives by differencing the net;
- compute the normal, the first and second fundamental forms and the curvatures, from first principles;
- write a plane, a ruled surface and a surface of revolution as NURBS, and recognise a pole;
- say what the surface being smooth and its coordinates being singular means at a pole;
- find these pieces in `core/geop-core-geometry/src/nurb_surface/`.

</div>

A solid is bounded by surfaces. We now have everything needed to define one, with almost no new mathematics. The trick is called the *tensor product*: a surface is a curve whose control points move along curves.

## From a moving curve to a net

Take a curve `$C(v)=\sum_jN_{j,q}(v)Q_j$`. Now let each control point move along its own curve, with a common parameter `$u$` and a common basis:

`$$Q_j(u)=\sum_iN_{i,p}(u)P_{ij}.$$`

Substituting gives

`$$S(u,v)=\sum_{i}\sum_{j}N_{i,p}(u)\,N_{j,q}(v)\,P_{ij}.$$`

That is a **tensor-product B-spline surface**: a rectangular **control net** `$P_{ij}$`, degree `$p$` and knots `$U$` in `$u$`, degree `$q$` and knots `$V$` in `$v$`, over the rectangle `$[u_p,u_{n+1}]\times[v_q,v_{m+1}]$`. With weights it is a **NURBS surface**, the homogeneous points `$(w_{ij}P_{ij},w_{ij})$` obeying exactly this polynomial formula upstairs:

`$$S(u,v)=\frac{\sum_{i,j}N_{i,p}(u)N_{j,q}(v)w_{ij}P_{ij}}{\sum_{i,j}N_{i,p}(u)N_{j,q}(v)w_{ij}}.$$`

What makes this powerful: fix one parameter, and the rest is a curve algorithm. Fix `$u=u_0$`, and `$S(u_0,v)$` is a B-spline in `$v$` with control points `$Q_j(u_0)$`. This is an **iso-curve** (or iso-parametric curve). So:

- **Evaluate**: de Boor along `$u$` on each column of the net gives one row of points; de Boor along `$v$` on that row gives the point. The order is irrelevant.
- **Split**: insert the same knot in every row. This is what "works on rows" meant in [Lecture 9](./09-subdivision.md).
- **Convex hull**: with non-negative weights, the products `$N_{i,p}N_{j,q}$` are non-negative and sum to one. So `$S(u,v)$` lies in the hull of the active `$(p+1)(q+1)$` net points.
- **Boundary**: for clamped knots, the boundary of the patch is four curves whose control points are the outer rows of the net, and the corners are the corner control points.

The demo shows the two-stage evaluation for a bicubic Bézier patch: sixteen control points, cubic in each direction.

<figure class="demo" data-demo="surface-patch">
<figcaption>Drag to turn the patch. Sliders move the point (u, v). Faint lines: each column of the net as a curve in u. Orange: the four points where these curves are at the current u, their control polygon (dashed), and the iso-curve they define in v; the red point is on it. That is de Casteljau in two stages. Green and purple are the partial derivatives, red the normal. Choose a control point and a height to change the net. The readout gives the fundamental forms and curvatures from the same formulas as in the text.</figcaption>
</figure>

Things to check in the demo: move `$v$` only, with `$u$` still: the orange curve does not change. Choose the corner `$P_{00}$` and raise it: the surface's corner follows exactly. Raise an inner point `$P_{11}$`: the surface follows only part of the way.

## Partial derivatives

Differentiate the double sum with respect to `$u$`, holding `$v$`. By Lecture 7's derivative formula, applied to every column, `$S_u$` is again a tensor-product surface, whose net is differences along `$u$`:

`$$S_u(u,v)=\sum_{i,j}N_{i+1,p-1}(u)\,N_{j,q}(v)\,\frac{p\,(P_{i+1,j}-P_{ij})}{u_{i+p+1}-u_{i+1}}.$$`

For a Bézier patch on `$[0,1]^2$`, this is `$p\sum B_i^{p-1}(u)B_j^q(v)(P_{i+1,j}-P_{ij})$`. The same goes for `$S_v$`. Second derivatives come from differencing twice: `$S_{uu}$` along `$u$` twice, `$S_{vv}$` along `$v$` twice, and `$S_{uv}$` once each way. A key fact is that all these nets are finite data, so *their hulls bound the derivative over a whole box of `$(u,v)$`*. Tangent directions and normals of a piece of surface can be bounded by proof, not by sampling.

For a rational surface one differentiates upstairs and applies Leibniz in two variables, as for curves. With `$A=wS$`:

`$$S_u=\frac{A_u-w_uS}{w},\qquad S_{uv}=\frac{A_{uv}-w_{uv}S-w_uS_v-w_vS_u}{w}.$$`

<div class="kernel">

**In the kernel.** Storage is in `core/geop-core-geometry/src/nurb_surface/mod.rs`: `NurbSurface<D>`, with `degree_u`, `degree_v`, `num_u`, `num_v`, one flat `control_points` vector stored "row-major: `control_points[i * num_v + j]`", two knot vectors, and the cached box. `NurbSurface3D` is `NurbSurface<4>`. `try_new` checks the same things as for curves: the sizes, and that every weight is definitely positive.

`NurbSurface::evaluate` is in `nurb_surface/evaluate.rs`. `partials`, `derivatives` and `second_derivatives` are in `nurb_surface/normal.rs`, the last with exactly the formula above for the mixed partial. `partials` differences first and evaluates afterwards: the order that keeps interval enclosures tight, as we saw in [Lecture 7](./07-bsplines.md), by a factor the source measures at a hundred.

A rectangle of knots gives a rectangle of cells. `iso_curve` (`nurb_surface/split.rs`) extracts one iso-line as a curve by knot insertion. `split_u`, `split_v`, `sub_surface` split on the rows. And `aabb` gives the box of the net: `could_overlap` proves two surfaces apart by comparing boxes.

</div>

## The normal

`$S_u$` and `$S_v$` span the **tangent plane** at a point, if they are independent. The **unit normal** is

`$$n=\frac{S_u\times S_v}{\lvert S_u\times S_v\rvert}.$$`

Two things to keep in mind.

**Its direction has meaning, and the parametrization decides it.** Swapping `$u$` and `$v$`, or reversing `$u$`, flips the normal. In a solid, the normal must point out of the material. The kernel has no orientation flag on a face: "Orientation has no flag. It lives in the surface's parametrization" (from the [topology crate page](../core/geop-core-topology.md)). `reverse_face` mirrors the surface in `$u$` and its pcurves with it.

**It can fail.** Where `$S_u\times S_v=0$` the formula is `$0/0$`. It may be that the surface itself has a crease or a tip. It may equally well be that a perfectly smooth surface has a bad coordinate system there. We will now look at the latter, the most important kind.

## Fundamental forms and curvature

Everything a kernel needs to know about the local shape of a surface comes from two quadratic forms. They are worth deriving rather than memorizing.

**First fundamental form.** Move in the domain at a velocity `$(\dot u,\dot v)$`. The velocity in space is `$S_u\dot u+S_v\dot v$`, of squared length

`$$\lvert\dot S\rvert^2=E\dot u^2+2F\dot u\dot v+G\dot v^2,\quad E=S_u\cdot S_u,\ F=S_u\cdot S_v,\ G=S_v\cdot S_v.$$`

So `$E,F,G$` measure lengths and angles on the surface, using only the parameters. A patch of surface of domain size `$\mathrm{d}u\,\mathrm{d}v$` has area `$\sqrt{EG-F^2}\,\mathrm{d}u\,\mathrm{d}v=\lvert S_u\times S_v\rvert\,\mathrm{d}u\,\mathrm{d}v$`.

**Second fundamental form.** Now take a curve on the surface, `$\gamma(s)=S(u(s),v(s))$`, with arc length `$s$`. Its acceleration `$\ddot\gamma$` has a component in the tangent plane, which says how the curve turns *within* the surface, and one along `$n$`, which says how much the surface forces it to bend. Since `$\dot\gamma\cdot n=0$` everywhere, differentiating gives `$\ddot\gamma\cdot n=-\dot\gamma\cdot\dot n$`, and expanding `$\ddot\gamma$`:

`$$\ddot\gamma\cdot n=L\dot u^2+2M\dot u\dot v+N\dot v^2,\quad L=S_{uu}\cdot n,\ M=S_{uv}\cdot n,\ N=S_{vv}\cdot n.$$`

This `$\ddot\gamma\cdot n$` is the **normal curvature** `$\kappa_n$` in the direction `$(\dot u,\dot v)$`. It depends only on the direction, not on the curve. For a general (non-unit) velocity:

`$$\kappa_n=\frac{L\dot u^2+2M\dot u\dot v+N\dot v^2}{E\dot u^2+2F\dot u\dot v+G\dot v^2}.$$`

As a direction turns, `$\kappa_n$` varies between a maximum and a minimum, the **principal curvatures** `$\kappa_1,\kappa_2$`. They are the eigenvalues of `$\mathrm{I}^{-1}\mathrm{II}$`, where `$\mathrm{I}=\begin{pmatrix}E&F\\F&G\end{pmatrix}$` and `$\mathrm{II}=\begin{pmatrix}L&M\\M&N\end{pmatrix}$`. Their determinant and half trace are the **Gaussian** and **mean** curvature:

`$$K=\kappa_1\kappa_2=\frac{LN-M^2}{EG-F^2},\qquad H=\frac{\kappa_1+\kappa_2}{2}=\frac{LG-2MF+NE}{2(EG-F^2)}.$$`

`$K>0$` means the surface locally stays on one side of the tangent plane, like a dome. `$K<0$` means a saddle. A surface with `$K=0$` at every point, like a plane or a cylinder, is called *developable*: it can be unrolled to a flat sheet without stretching.

### A worked example

Take the bicubic form of `$z=xy$`: `$S(u,v)=(u,v,uv)$` on `$[0,1]^2$` (with the control heights `$\frac i3\frac j3$`; try the "saddle" preset, which has this shape up to scale). Then

`$$S_u=(1,0,v),\quad S_v=(0,1,u),\quad S_{uu}=S_{vv}=0,\quad S_{uv}=(0,0,1),$$`

`$$S_u\times S_v=(-v,-u,1),\quad n=\frac{(-v,-u,1)}{d},\quad d=\sqrt{1+u^2+v^2}.$$`

`$$E=1+v^2,\ F=uv,\ G=1+u^2;\quad EG-F^2=d^2;\quad L=N=0,\ M=\frac1d.$$`

So `$K=\frac{-M^2}{d^2}=-\frac{1}{d^4}$` and `$H=\frac{-2MF}{2d^2}=-\frac{uv}{d^3}$`. At `$(0,0)$`: `$K=-1$`, a true saddle. These two formulas are what `book/scripts/demo_tests/surface-patch.test.mjs` checks the demo against.

<div class="kernel">

**In the kernel.** `core/geop-core-geometry/src/nurb_surface/curvature.rs` holds these. `normal_curvature` computes the formula for a given 3-D tangent `along`, by first solving for `$(\dot u,\dot v)$` from `along·Su` and `along·Sv` with `$E,F,G$` (and it reports, in its error context, that the first fundamental form could be singular). `mean_curvature` is the formula for `$H$` above. `curvature_radius` is deliberately cruder: "a conservative radius of curvature" from the diagonal terms alone, used to size marching steps, with its doc comment explaining when this is exact and why it serves. `curvature_radius_along` reports the radius in a direction, or none where the surface runs straight that way, which fixed a marching bug that [Lecture 16](./16-tracing-and-pcurves.md) describes.

Second derivatives also enter `NurbSurface::project`, where AGENTS.md's "Newton needs the second derivatives where it is far from flat" uses them ([Lecture 14](./14-newton.md)).

</div>

## The surfaces of a CAD system, as NURBS

Everything a mechanical part is made of is exactly representable.

**A plane** or planar rectangle is a bilinear patch: degree (1,1), four control points, weights 1 (`NurbSurface3D::bilinear`). It is the simplest tensor product, and its normal is `$(P_{10}-P_{00})\times(P_{01}-P_{00})$`.

**A ruled surface** joins two curves by straight lines: `$S(u,v)=(1-v)A(u)+vB(u)$`. If `$A$` and `$B$` have the same degree and knots, the net is simply their two control polygons side by side: degree `$(p,1)$`. Making two arbitrary curves compatible (the same degree and knots) is by degree elevation and knot insertion, `NurbCurve::compatible` in `nurb_curve/compatible.rs`, and then `NurbSurface3D::ruled`. An **extrusion** is a ruled surface between a curve and its translate, `NurbCurve::sweep` in `nurb_curve/sweep.rs`. So a **cylinder** is a circle extruded: degree (2,1).

**A surface of revolution** turns a profile `$P(v)=\sum_jN_j(v)w_jP_j$` about an axis. Each profile control point describes a circle, which Lecture 8 tells us how to write exactly:

`$$\text{row } j\text{ of the net: the arc through the turn of } P_j,\text{ with weights } w_j\cdot(1,\ \cos\tfrac\theta2,\ 1,\ \cos\tfrac\theta2,\ \dots).$$`

So the net is a product, and the weights multiply. `NurbSurface::revolve` in `nurb_surface/revolve.rs` does this: `$u$` is the turn over `$[0,1]$`, with rational quadratic pieces of at most a quarter turn, and `$v$` is the profile's own parameter, so the degree is `$(2,q)$`. A **cone** revolves a straight line that meets the axis. A **sphere** revolves a half circle. A **torus** revolves a circle clear of the axis. They are not kinds in the kernel, only control points.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"turbine_blade","example":"turbine_blade"}'>
<figcaption>What these surfaces can do. This turbine blade was built by the Geop kernel, then meshed by it: every face a NURBS patch with no type of its own.</figcaption>
</figure>

## Poles: where the coordinates fail

Take a revolved surface whose profile touches the axis: a sphere's pole or a cone's tip. The profile's control point on the axis does not move as it turns. So a whole **row of the net collapses to one point**. The surface at that boundary is the same point no matter what the turn `$u$` is, so `$S_u=0$` there, and `$S_u\times S_v=0$`: the formula for the normal fails.

The surface may be perfectly fine. For a sphere, the pole is a point like any other, with a good tangent plane; a different coordinate system would not notice it. This is the same situation as the Earth's north pole in latitude and longitude: every longitude names one point, and no step in longitude moves you, but the ground is flat there as anywhere else.

AGENTS.md's "Degenerate parametrizations: the surface is fine, the coordinates are not" gives three real bugs, and each comes from confusing the coordinates with the geometry:

- **A seed on the pole never leaves it.** `NurbSurface::project` skipped any Newton step whose Jacobian could be singular. From a seed on the pole every step is singular, so it returned the pole for every target. The fix is to step along the parameter that still moves the point, down a meridian: which meridian is a free choice, so take the one that heads most towards the target (`off_pole` in `nurb_surface/project.rs`).
- **Decide tangency where the geometry is, not at moving foot points.** Two quarters of a sphere meet at its pole, as one smooth surface. Comparing normals at foot points clamped to each patch's domain made them two nearly equal conditions, and a singular system.
- **Straight in `$(u,v)$` is not preserved by offsetting.** A sphere's meridian cut by a plane comes out as a small circle on the inner copy.

So how does the kernel get a normal at a pole? From the net, directly. `NurbSurface::normal` first tries `$S_u\times S_v$`. If that cannot be normalized, it looks for a pole on a boundary row the point touches (`rows_touching`, `pole`, `pole_normal`). Leaving a pole, the derivative across the row is a positive combination of the **spokes**, the vectors from the pole to the next row's control points. So the pole has a tangent plane exactly when those spokes are coplanar, and its normal is that plane's. If the spokes are not coplanar, as for a cone's apex, there is no single normal, and it is an error: "not an arbitrary pick among them".

Check that against a sphere and a cone. A sphere's next row near the pole lies in the tangent plane at the pole, because a circle's control polygon leaves along the tangent: the spokes are coplanar. A cone's next row is a circle further down, and the spokes form a cone. So the sphere's pole has a normal, and the cone's apex has none, by the same test, with no branch on the kind of surface.

What does a loop on such a face look like? [Lecture 18](./18-poles-seams-splicing.md) answers that. A collapsed row becomes a coedge that lies on a vertex, and the pcurve runs along the row.

## What the hull does for surfaces

We close with the property that makes all this searchable. Each net cell's hull encloses a piece of surface. Split in `$u$` and `$v$` and the net refines, and the distance from the net to the surface falls with the square of the cell's size, as for curves. So a question like "could this surface contain this point?" comes down to a question about a finite array of numbers. That is `surface_could_contain` in `core/geop-core-geometry/src/contains/surface.rs`, with fat-line clipping in place of plain bisection, as [Lecture 13](./13-fat-lines.md) will explain. For the cheapest case there is `NurbSurface::aabb`.

## Recap

<div class="recap">

- A tensor-product surface is a net of control points with two knot vectors. Fix one parameter, and the rest is a curve algorithm: evaluate, split and bound are all row-wise.
- Derivatives are nets of differences, so they can be bounded by their hulls over a whole box.
- `$E,F,G$` measure lengths; `$L,M,N$` measure bending along the normal. Their ratio gives normal curvature, and `$K$`, `$H$` summarize it.
- Planes, rulings, cylinders, cones, spheres and tori are control points, not kinds.
- A collapsed row is a pole: `$S_u\times S_v$` vanishes though the surface is fine. Geop reads the normal off the spokes instead, which also tells a sphere's pole from a cone's apex.

</div>

## Exercises

**1.** Write the net, weights and knots for a quarter of the unit cylinder of height 2, as a NURBS surface of degree (2,1).

<details class="solution"><summary>Solution</summary>

`$u$` round the arc: `$U=(0,0,0,1,1,1)$`, `$V=(0,0,1,1)$`. Rows at `$z=0$`: `$(1,0,0),(1,1,0),(0,1,0)$`, and at `$z=2$`: `$(1,0,2),(1,1,2),(0,1,2)$`, with weights `$1,\frac{\sqrt2}2,1$` for each. Upstairs the middle points are `$(h,h,0,h)$` and `$(h,h,2h,h)$`, `$h=\frac{\sqrt2}2$`.

</details>

**2.** For the quarter cylinder of exercise 1 at `$u=0$`, compute `$S_u$`, `$S_v$`, `$n$`, `$E,F,G$` and `$L,M,N$`, and hence `$K$` and `$H$`. Use `$S_u(0)=2w_1(P_1-P_0)$`, and find `$S_{uu}$` from the circle: at parameter speed `$\sigma$`, a unit circle's second derivative has normal component `$\sigma^2$` towards the axis.

<details class="solution"><summary>Solution</summary>

With `$h=\frac{\sqrt2}2$`, `$S_u=2h(0,1,0)=(0,\sqrt2,0)$`, `$S_v=(0,0,2)$`. `$n=(1,0,0)$`, outward. `$E=2$`, `$F=0$`, `$G=4$`. `$S_{uu}\cdot n=-\sigma^2=-2$`, so `$L=-2$`; `$M=N=0$`. So `$K=0$`, as for a developable surface, and `$H=\frac{LG}{2EG}=\frac{L}{2E}=-\tfrac12$`: half of the unit circle's curvature, negative for this outward normal, independent of the parametrization. Note `$\kappa_n=L/E=-1$` along `$u$`.

</details>

**3.** Show that for `$F=0$`, `$\kappa_n$` along `$u$` is `$L/E$`. Compare with `curvature_radius` in `nurb_surface/curvature.rs`, and say exactly when its result is a true principal curvature.

<details class="solution"><summary>Solution</summary>

Set `$\dot v=0$` in the formula: `$\kappa_n=L/E$`. These two are the principal curvatures when `$F=0$` and `$M=0$`, as then `$\mathrm{I}^{-1}\mathrm{II}$` is diagonal: that holds for flat faces and for revolved patches. In general `curvature_radius` is a guide for sizing steps, the tighter of the two bends along the parameter directions, and `mean_curvature` and `normal_curvature` are there for an arbitrary parametrization.

</details>

**4.** Why must a surface's two knot vectors be separate, with a single net? A designer asks for a patch with an extra row of control points only in one part of it: what happens? (This is a real limit of tensor products. It is why trimmed faces and multiple patches exist.)

<details class="solution"><summary>Solution</summary>

The net must be a rectangle, because `$S$` is a product of two one-dimensional bases: a knot in `$u$` applies to every `$v$`, adding a whole row. There is no local refinement. Consequences: complex shapes are built of several patches, joined along edges and trimmed by curves in `$(u,v)$`. [Lecture 17](./17-brep.md) covers how a face is a trimmed patch.

</details>

**5.** A revolved profile has one control point on the axis, and its other control points clear of it. Which row of the net collapses? Which partial vanishes at that row, and which does not? Compute `$S_v$` at the pole for a sphere of radius 1, with `$v$` the parameter along a quarter-arc profile from the pole to the equator. Does its direction depend on `$u$`?

<details class="solution"><summary>Solution</summary>

The row in `$u$` at the pole's index collapses: `$S_u=0$` there, because no change of `$u$` moves the point. `$S_v$` does not vanish: it is the tangent to the profile at the pole, of length `$\sqrt2$` (from the arc's end speed, computed in [Lecture 8](./08-nurbs.md)), and points along the meridian. Its direction *does* depend on `$u$`: it turns around as `$u$` does. All meridians meet at the pole, each with its own tangent. Those directions are coplanar: they span the tangent plane. That's the spokes.

</details>

**6.** Show that a tensor-product Bézier patch with an axis-aligned box has its normal bounded by the cross products of the difference nets. How would you bound the normal over a sub-box? Give a condition under which the patch cannot fold over onto itself.

<details class="solution"><summary>Hint</summary>

`$S_u$` is a Bézier patch whose control points are `$p(P_{i+1,j}-P_{ij})$`, and likewise `$S_v$`: bound each by their hulls `$H_u,H_v$`. Every normal direction is in `$H_u\times H_v$`, computed for example in interval arithmetic. If there is a direction `$d$` with `$d\cdot(a\times b)>0$` for all `$a\in H_u$`, `$b\in H_v$`, the normal never turns all the way, and the patch projects without folding on the plane normal to `$d$`. Splitting tightens all bounds.

</details>
