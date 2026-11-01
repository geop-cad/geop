# Lecture 28: Lofts and filling surfaces

<div class="goals">

**After this lecture you can**

- say what a loft is, and solve its real problem: deciding which point of one section goes with which point of the next;
- make curves compatible, so that they can be rows of one surface;
- derive the Coons patch as a Boolean sum of two ruled surfaces, and write its control net with Greville abscissae;
- extend that to the Gordon surface through a network of curves;
- explain the vertex enclosure problem, and why a filled patch's corners must be flat points;
- state how a patch can be proven to stay on one side of a curved face it should be tangent to.

</div>

A sweep carries a single profile along a path. Often a designer has something different in mind: the root section of a blade, and its section at the tip, and "whatever's smooth in between". This is a **loft**. Or there is no section at all, only a closed loop of edges around a hole in a surface model, and the wish to "fill that smoothly". This is a **fill**. Both produce new surfaces from curves, and the second is where continuity across a boundary, the theme of this part, becomes demanding.

## The real problem of a loft: correspondence

Suppose two sections are given, a square and a circle. Skinning them is easy once you have decided which points correspond. But the kernel's grid of Lecture 26 requires that each section have *the same number of curves*, and that the `$i$`-th curves of all sections be rows of one surface. So before any surface exists, a lot of decisions are needed about correspondence, and it is these that decide whether the result looks like what the designer meant. Twist the correspondence by one joint and you get a wrung-out towel.

<div class="kernel">

**In the kernel.** `correspond` in `lib/geop-lib-sweep/src/loft.rs` carries this out, and the module's doc comment is the full specification. In order:

- **Winding.** Each section is turned to run the same way round as seen along the loft, mirroring its frame where its plane faces the other way.
- **Matched points.** Points the designer has matched across closed sections are lofted into each other. Between two matched points, corners are matched too wherever every matched section has as many.
- **Corners.** With no points matched, the sections' *corners* are the matched points: joints where the tangent turns, the thing the designer sees, not "the kernel's own split of an arc at a quarter turn". Each section is turned to line its corners up best with the previous one. Sections with corners must have equal numbers of them, or the loft is refused by name: "a D of two corners into a triangle needs matched points or guides to say what goes where". Where the kernel cannot tell what is meant, it does not guess.
- **Runs.** Between two matched points, each section's smooth run is cut where any other has a joint, at the same fraction of the run's length (`RunCuts::merged`), so the flat bottom of a D meets the flat bottom of the next.
- **Start.** A closed section not matched starts at whichever joint lines its joints up best with its neighbour.
- **Curves.** Finally the `$i$`-th curves are made compatible.

</div>

## Compatibility

To be rows of one surface, curves must share a degree and a knot vector. Neither requirement limits what you can draw, because neither operation changes a curve.

**Raising the degree.** Lecture 6's exercise: `$Q_i=\frac{i}{p+1}P_{i-1}+\bigl(1-\frac{i}{p+1}\bigr)P_i$`, for a Bézier piece, on homogeneous control points. `NurbCurve::elevate_degree` cuts a curve into Bézier pieces, raises each, and keeps them joined, each interior knot then having multiplicity `$p+1$`.

**Merging knots.** Lecture 9's knot insertion: give each curve every knot any of them has, at the highest multiplicity.

`NurbCurve::compatible` in `core/geop-core-geometry/src/nurb_curve/compatible.rs` does both. One detail is particular to an interval kernel: "Knots of different curves that could be equal are taken as one, the union of their enclosures, which every curve then carries." Two curves with knots `$[0.5-\epsilon,0.5+\epsilon]$` and `$[0.5,0.5+2\epsilon]$` should not end up with two separate knots that close together: the span between them could be zero, and Boehm's insertion divides by it.

Let us do a small example. A line in degree 1 with knots `$[0,0,1,1]$` from `$(0,0)$` to `$(1,0)$`, and a parabola of degree 2 with knots `$[0,0,0,1,1,1]$` and control points `$(0,1),(\tfrac12,2),(1,1)$`. Raise the line: `$Q_0=(0,0)$`, `$Q_1=\tfrac12(0,0)+\tfrac12(1,0)=(\tfrac12,0)$`, `$Q_2=(1,0)$`. Now both are degree 2 with one knot vector, and the ruled surface between them has control net

`$$\begin{pmatrix}(0,0)&(\tfrac12,0)&(1,0)\\(0,1)&(\tfrac12,2)&(1,1)\end{pmatrix}.$$`

Two sections skinned give exactly that: ruled walls, straight lines between corresponding points. Three or more sections are joined by `through`, which writes each span's inner control rows as a *sum of the sections at any stations*, a `Span::Through`: the tangent across a section is taken from its neighbours, spaced by how far apart their centres are. The doc comment gives the reason for that free choice: "spacing by their distances keeps a section far from the others from pulling the walls past it". **Guide curves** instead shape the walls by displacing the ruled surface with a map that is affine across the section, so the walls' edges are the guides themselves. One guide translates the sections along with it, two also turn and stretch them, and three fix any affine change.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"turbine_blade","example":"turbine_blade"}'>
<figcaption>The kernel's <code>turbine_blade</code> example: a blade lofted through sections on three offset datum planes, patterned six times round a revolved hub and joined to it. This view has 8307 triangles, so turn it gently on a phone.</figcaption>
</figure>

## The Coons patch

We now change the data: instead of sections in one direction, four curves round a loop. Let `$c_0(u),c_1(u)$` run along `$v=0$` and `$v=1$`, and `$d_0(v),d_1(v)$` along `$u=0$` and `$u=1$`, with matching corners. We seek a surface `$S(u,v)$` with those four sides.

A ruled surface between `$c_0$` and `$c_1$` gets two sides right:

`$$L_c(u,v)=(1-v)\,c_0(u)+v\,c_1(u).$$`

Its sides at `$u=0$` and `$u=1$` are straight lines between the corners. Similarly

`$$L_d(u,v)=(1-u)\,d_0(v)+u\,d_1(v)$$`

gets the other two right. Neither alone suffices. Adding them gets all four sides right, except each side is *off by the same straight line*. On `$u=0$`, say, `$L_d$` gives `$d_0(v)$`, while `$L_c$` gives the line from `$c_0(0)$` to `$c_1(0)$`, a straight line that also appears in the bilinear interpolant of the corners, `$B$`. So subtract that:

`$$\boxed{S(u,v)=L_c(u,v)+L_d(u,v)-B(u,v)}$$`

`$$B(u,v)=(1-u)(1-v)P_{00}+u(1-v)P_{10}+(1-u)vP_{01}+uvP_{11}.$$`

To check: at `$v=0$`, `$L_c=c_0(u)$`, `$L_d=(1-u)P_{00}+uP_{10}$`, and `$B=(1-u)P_{00}+uP_{10}$`. These cancel, and `$S=c_0(u)$`. The other three sides work the same way.

This is the **bilinearly blended Coons patch** (Coons, 1967). There is a tidy way to see it. Let `$\mathcal P_u$` be the operator "interpolate linearly in `$u$` between the sides `$u=0,1$`" and `$\mathcal P_v$` likewise. Each projects: `$\mathcal P_u^2=\mathcal P_u$`. Their composition `$\mathcal P_u\mathcal P_v$` is `$B$`. The formula `$S=(\mathcal P_u+\mathcal P_v-\mathcal P_u\mathcal P_v)f$` is the **Boolean sum**, the same inclusion–exclusion you know from probability.

<figure class="demo" data-demo="coons-patch">
<figcaption>Drag the control points of any side (they move in the plane facing you), or drag elsewhere to turn the view. "Show" switches between the Coons patch and its three parts; the readout says how far the surface lies from its sides. The kernel builds the same patch as one control net, below.</figcaption>
</figure>

### From a formula to a control net

A formula is not yet a NURBS. It becomes one thanks to **Greville abscissae**. For a B-spline of degree `$p$` on knots `$t_i$`, the identity

`$$t=\sum_i\xi_i\,N_{i,p}(t),\qquad \xi_i=\frac{t_{i+1}+\dots+t_{i+p}}{p}$$`

expresses *linear precision*: the function `$t$` itself has the `$\xi_i$` as its coefficients. So multiplying by a linear blend is exact on control points: a point on the row `$i,j$` of the net of `$L_c$` is `$(1-\xi_j)b_i+\xi_jt_i$`, where `$b_i,t_i$` are bottom and top control points of the compatible sides and `$\xi_j$` the Greville abscissa across. Then

`$$P_{ij}=\underbrace{(1-\xi_j)b_i+\xi_jt_i}_{\text{across}}+\underbrace{(1-\eta_i)l_j+\eta_ir_j}_{\text{along}}-\underbrace{\text{bilinear of corners at }(\eta_i,\xi_j)}_{\text{corners}}.$$`

That is what `NurbSurface3D::coons` in `core/geop-core-geometry/src/nurb_surface/patch.rs` computes, on homogeneous control points so that rational sides work (the corner weights are brought to one first, by `NurbCurve::with_unit_end_weights`, or two sides meeting at a corner might disagree about it).

**A worked example.** Three straight sides on the unit square, and a bulging fourth, `$d_1(v)=(1+v(1-v),\ v,\ 0)$`. Then `$L_c=(u,v,0)$`, `$B=(u,v,0)$`, and `$L_d=(u+uv(1-v),\ v,\ 0)$`, so `$S(u,v)=(u+uv(1-v),\ v,\ 0)$`. The bulge fades out linearly away from its side: `$S(\tfrac12,\tfrac12)=(\tfrac58,\tfrac12,0)$`.

Note a piece of care which we met in Lecture 26. The net's border rows are taken directly from the sides' control points: "that is what the blend gives there exactly; taking them as they are only leaves out the rounding of adding and taking away the same corners". And where every weight is one, the weight is set back to an exact 1 rather than kept as a computed interval around 1.

## The Gordon surface

The Coons idea scales to a network. Take `$m$` curves in one direction and `$n$` in the other, each crossing every curve of the other family once. Let `$\mathcal P_u$` skin through the `$v$` curves, `$\mathcal P_v$` skin through the `$u$` curves, and `$\mathcal P_u\mathcal P_v$` be the tensor-product surface through the crossing points. The **Gordon surface** `$\mathcal P_u+\mathcal P_v-\mathcal P_u\mathcal P_v$` interpolates all the curves at once. Each of the three runs through the crossings, and along a `$u$` curve the second and third agree, which leaves the first, which is the `$u$` curve. With two curves each way this is the Coons patch again.

There is one obstacle. Each family must cross the others at a common set of parameters, and curves do not naturally do that. In `core/geop-core-geometry/src/nurb_surface/gordon.rs`, `common_parameters` finds where each family crosses, as a fraction of each curve's stretch averaged over the family, and each curve is reparametrized to cross there (`NurbCurve3D::reparametrized`). "Which parameters is a free choice — any increasing ones make the same surface pass through the same curves — and the average keeps each curve's own speed nearly as it was, so the parameters are sharpened": Lecture 5's rule, in the wild. The doc comment on `gordon` is candid about the cost: where curves do not cross at the same fractions, "each curve's speed changes there, and the surface may bend across it".

## Tangent to what is around it

A Coons patch fills a hole, but it only guarantees position along the boundary. In a surface model that isn't enough. Filling a four-sided hole between curved surfaces with a mere position match leaves a visible crease. We want **tangency across each side**: `$G^1$` continuity to the neighbour. So the row of control points next to each side has to be moved to lie in the neighbour's tangent plane.

Over a **flat** neighbour this is exact: the tangent plane is constant, so the row lies in that plane. Exactly.

Over a **curved** neighbour the tangent plane turns along the side, and no NURBS row follows it exactly. Here we meet again what we shall see in the fillet ([Lecture 29](./29-fillets.md)): a surface approximately tangent to another crosses it, as often as not, by a hair, and a boolean then has to cut along each crossing. So we do not aim for tangency at all. We aim for a **tilt**. `NurbSurface3D::tangent_to` leans the row so that the patch leaves the face at `TILT`, one millionth of a radian (`core/geop-core-geometry/src/nurb_surface/patch.rs`), towards the face's normal, then it *proves* the result.

### Proving a tilt: `strays`

Let `$s$` run along the side, let `$\mathrm{into}(s)$` be the patch's derivative across the side, and let `$n(s)$` be the face's normal where the side is at `$s$`. The patch leaves the face on the right side exactly when

`$$f(s)=\mathrm{into}(s)\cdot n(s)>0.$$`

Checking this at the Greville abscissae and halfway between is what `misses` does: that is a sample, and samples prove nothing. `strays` proves it over a whole stretch `$I$` by the **mean value form**:

`$$f(I)\subseteq f(c)+f'(I)\,(I-c),\qquad f'=\mathrm{into}'\cdot n+\mathrm{into}\cdot n'.$$`

The derivatives are enclosed over the box. The hard part is `$n$` and `$n'$`: they depend on the foot point on the face, which moves with `$s$`. `NurbSurface3D::enclose_foot` provides a proven box of `$(u,v)$` containing every foot of the side over `$I$`, by a Krawczyk-style argument (Lecture 24), and the normal and its turn are enclosed over that box. Since the bound's width goes as `$\lvert I\rvert\cdot f'$`, it shrinks with the stretch. AGENTS.md records that a millionth of a radian is shown over stretches of `1e-4`, so thousands per side, and the doc comment on `strays` speaks of stretches "a thousandth long". The cheap look at the abscissae says where to refine while the patch is far off, and only the proof says it is done.

`tangent_to` refines the patch and tries again while it strays, at most `MOST_REFINEMENTS` times, and failing that it reports where. It never claims a tilt that it could not show.

## Filling N sides

Four sides make a patch; a loop of three, or five or more, doesn't have a tensor-product structure. The kernel's answer (`NurbSurface3D::fill`, in `core/geop-core-geometry/src/nurb_surface/fill.rs`) is a flat inner polygon, with a four-sided patch along each side of the loop, each between the side, two *spokes* from its corners to the polygon, and the polygon's own side.

```text
         side k
    ●──────────────●        ● a corner of the loop
     \            /         ─ a side, kept exactly
      \  patch k /          \ a spoke
 spoke \        / spoke
        ○──────○            ○ the inner polygon
```

Now the patches must be tangent to one another across every spoke, and to the polygon. Here "both sides are the kernel's", and that changes things. There is no need for a tilt because the kernel can make them agree exactly: "the two patches meeting along a spoke share its tangent plane at every point of it exactly, by construction". The plane is spanned by the spoke's tangent `$T(v)$` and a field `$V(v)$` across it, both polynomials, and each patch's derivative across the spoke is `$\alpha(v)V(v)+\beta(v)T(v)$`, with `$\alpha,\beta$` linear functions of its own. That derivative is the row next to the spoke, a polynomial too. The kernel's comment credits this form of cross-boundary derivative to SMLib: `$B(v)=p(v)D(v)+q(v)T(v)$`. So two patches built this way are tangent identically.

### The vertex enclosure problem

Something subtle now happens at a corner of the inner polygon, where two patches and the flat polygon meet, each tangent to the other two.

Let `$S_A$` and `$S_B$` be the two patches' shape operators at that corner. (The shape operator maps a tangent direction to the derivative of the unit normal in that direction; it is symmetric.) Patch `$A$` is tangent to the flat polygon all along its polygon side, so its normal is constant along that side: `$S_A a=0$`, with `$a$` the side's direction. Being symmetric with a null vector in two dimensions, `$S_A=\lambda\,bb^{\mathsf T}$`, where `$b\perp a$`. The same goes for `$S_B=\lambda'\,b'b'^{\mathsf T}$`, where `$b'\perp a'$`, and the two polygon sides at a corner have different directions, so `$b\ne\pm b'$`.

Across the spoke, with direction `$s$`, the two patches share a tangent plane along the spoke, so the same normal along it. So `$S_As=S_Bs$`:

`$$\lambda(b\cdot s)\,b=\lambda'(b'\cdot s)\,b'.$$`

The vectors `$b,b'$` are not parallel, so both sides vanish. The spoke leaves the corner into the polygon's exterior, not along a side, so `$b\cdot s\ne0$`, so `$\lambda=0$`. Hence **the corner must be a flat point of both patches**. This is the *vertex enclosure problem*, and in the kernel's history it showed up as a construction that would not converge. As AGENTS.md puts it, "what would not converge before was no numerics". You could refine forever, since no surface exists meeting the conditions unless the data at the corner is flat.

The fix is to meet the condition by construction. The spokes arrive *osculating* the polygon's plane, their last three control points in it; the field `$V$` is constant to first order there; and every control point of the patch within two of that corner lies in the plane (`SPOKE_DEGREE` is 5, so a spoke has six control points: three at the loop's corner, in the tangent plane there, and three at the polygon).

At a corner of the loop there is a companion problem: where the tilt rises from nothing at a corner of a curved face, the corner's twist has to take in how the face's normal turns along the side (`Neighbour::turning` and `twist` in `fill.rs`). AGENTS.md reports that collocation alone left a bump just off the corner that refinement shrank only as `$h^2$`. With the twist right, halving only the spans where the patch strays converges in a few rounds.

<div class="kernel">

**In the kernel.** The surface operation `BoundarySurface` (`ops/geop-ops-surface/src/boundary.rs`) routes on the shape of the loop, not on the surface kinds. Two edges that close no loop give a ruled face. A closed loop gives a flat face if every edge lies in one plane, a Coons patch for four edges, and an N-sided fill otherwise, tangent to neighbours along edges picked as `tangent`.

</div>

<div class="recap">

## Recap

- A loft is first a correspondence problem: winding, matched points, corners, runs, start, and then compatibility. Where a choice cannot be made well, the kernel refuses by name.
- Raising degree and inserting knots make curves compatible without changing them.
- The Coons patch is the Boolean sum `$L_c+L_d-B$`, and Greville abscissae turn each linear blend into control points. The Gordon surface is the same with skinning for blending.
- Where a surface is only approximately tangent to something fixed, tilt away by a little and prove it (`strays`, a mean value form over a proven foot box).
- Where both surfaces are yours, build one polynomial plane field and make them tangent exactly.
- Two patches and a flat polygon, mutually tangent, force a flat point at the corner: the vertex enclosure problem. Build it in.

</div>

## Exercises

**1.** Show the Coons formula's value at `$u=1$` is `$d_1(v)$`.

<details class="solution"><summary>Solution</summary>

At `$u=1$`: `$L_d=d_1(v)$`, `$L_c=(1-v)c_0(1)+vc_1(1)=(1-v)P_{10}+vP_{11}$`, and `$B=(1-v)P_{10}+vP_{11}$`. The last two cancel.

</details>

**2.** Verify the Greville identity `$t=\sum_i\xi_iN_{i,p}(t)$` for `$p=2$` on knots `$[0,0,0,1,1,1]$`, by writing out the Bernstein polynomials.

<details class="solution"><summary>Solution</summary>

`$\xi_0=\tfrac{0+0}{2}=0$`, `$\xi_1=\tfrac{0+1}{2}=\tfrac12$`, `$\xi_2=\tfrac{1+1}{2}=1$`. Then `$\sum\xi_iB_i^2(t)=\tfrac12\cdot2t(1-t)+t^2=t-t^2+t^2=t$`.

</details>

**3.** Make these compatible: a quadratic Bézier over `$[0,1]$`, and a degree-2 curve on knots `$[0,0,0,\tfrac12,1,1,1]$`. How many control points does each end with? What happens to their shapes?

<details class="solution"><summary>Solution</summary>

The Bézier needs the knot `$\tfrac12$` inserted once, giving 4 control points, the same as the other. Their shapes are unchanged.

</details>

**4.** Why does the vertex enclosure argument need the two polygon sides to have different directions? What goes wrong in the argument when the inner polygon has a straight angle at the corner? Does the kernel's construction produce such a thing?

<details class="solution"><summary>Hint</summary>

If `$a\parallel a'$` then `$b\parallel b'$`, and the equation `$\lambda(b\cdot s)b=\lambda'(b'\cdot s)b'$` no longer forces `$\lambda=0$`: it only says the two patches have the same curvature across the spoke, as one cylinder cut in two would. Look at `inner_polygon` in `fill.rs`: the corners sit "halfway to the loop's centre", so a straight corner can arise only if the loop has one.

</details>

**5.** A row next to a side is leaned at `TILT` from the face at its knots, and `$f(s)$` wobbles between. `$f'$` over a stretch `$I$` is bounded by `$10^{-3}$`, and `$f(c)=10^{-6}$`. How long may `$I$` be for the mean value form to prove `$f>0$`?

<details class="solution"><summary>Solution</summary>

`$f(I)\subseteq10^{-6}\pm10^{-3}\lvert I\rvert/2$`, so we need `$\lvert I\rvert<2\times10^{-3}$`. With a side of length one, at least 500 stretches. As the patch follows the face more closely `$f'$` falls, and fewer are needed. This is why AGENTS.md finds "thousands per side", and why refining the patch also makes it cheaper to prove.

</details>

**6.** A designer lofts a D of two corners into a triangle. The kernel refuses. List three ways the designer can tell it what is meant.

<details class="solution"><summary>Hint</summary>

Read the module docs of `lib/geop-lib-sweep/src/loft.rs`: matched points, guide curves, and a third way is to change one section so the corner counts agree. `Section` and `Ends` tell you what else a loft takes.

</details>
