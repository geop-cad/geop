# Lecture 27: Sweeping along a path

<div class="goals">

**After this lecture you can**

- say what a *frame* along a curve is, and why a sweep's result is determined by the choice of one;
- derive the Frenet frame, and explain why it fails at an inflection and on a straight piece;
- define a rotation-minimizing frame by its angular velocity, and carry one along a curve by double reflection;
- compute how far a rotation-minimizing frame turns against the screw frame on a helix, in closed form;
- explain why a closed path may refuse to close, and what the frame's mismatch is geometrically;
- say which spans of a sweep are exact and which are approximated, and why.

</div>

Lecture 26 swept a profile along a straight line and round a circle, and both were exact. What about any other curve: a pipe's route, a spring? We have the grid already. The new question is what the stations' frames should be. That is, as the profile travels, which way should it face and how should it be rolled?

The first part is easy: for a sweep the profile usually stays square to the path. The second is the entire content of this lecture. A plane square to a curve at a point leaves one degree of freedom, a rotation about the tangent, and nothing about the curve alone seems to fix it.

## What a frame is, and what can go wrong

A **frame along a curve** `$C(t)$` is a choice, at every `$t$`, of an orthonormal triple `$(T,U,V)$` with `$T$` the unit tangent. It need not be Frenet's. Sweeping a profile with coordinates `$(x,y)$` gives the surface

`$$S(t,x,y)=C(t)+x\,U(t)+y\,V(t).$$`

The choice of `$U(t)$` matters a great deal. If `$U$` spins quickly about the tangent, a rectangular section winds round the path like a drill bit, even though the path did not ask for that. So we need a rule that chooses `$U$` so that the sweep does not twist more than the path forces it to. Let us look at the obvious candidates.

### The Frenet frame

Differential geometry offers a frame made from the curve alone. Writing `$s$` for arc length:

`$$T=\frac{dC}{ds},\qquad N=\frac{dT/ds}{\lvert dT/ds\rvert},\qquad B=T\times N,$$`

with the Frenet equations

`$$\frac{dT}{ds}=\kappa N,\qquad \frac{dN}{ds}=-\kappa T+\tau B,\qquad \frac{dB}{ds}=-\tau N,$$`

where `$\kappa\ge0$` is the curvature and `$\tau$` the torsion. Equivalently the frame rotates with the *Darboux vector* `$\omega_F=\tau T+\kappa B$`: for each axis `$a$`, `$da/ds=\omega_F\times a$`.

The Frenet frame is canonical, which sounds ideal, and it fails in two ways.

**At an inflection.** Where `$\kappa=0$`, the direction `$N$` is undefined. Take the planar curve `$y=x^3-x$`. On one side of the inflection `$N$` points up and on the other it points down. Passing through, the frame flips by 180°, in one jump. A swept rectangle would be turned through half a turn in zero length.

**On a straight piece.** A line has no `$N$` at all. A path of a line followed by an arc, the most common pipe in the world, has no Frenet frame along its first piece.

**And it twists when it needn't.** Even where it is defined, Frenet follows the curve's torsion into a roll that nothing physical requires.

### Rotation-minimizing frames

Instead of asking what the curve *says* `$U$` should be, specify how `$U$` is allowed to change. Require that `$U$` turn no more about the tangent than it must:

`$$\frac{dU}{ds}\parallel T.$$`

Such a frame's angular velocity has no component along the tangent. It turns `$T$` as required and nothing else, and the angular velocity with those properties is

`$$\omega=T\times\frac{dT}{ds}=\kappa B.$$`

So it is the Darboux vector with the `$\tau T$` term removed. Starting from any `$U_0$` square to `$T(0)$`, this gives a unique frame by solving an ODE. Bishop introduced this frame in 1975 and showed that it exists wherever the curve is `$C^2$`, including at inflections and on straight pieces, where `$\kappa=0$` simply means no rotation. Mechanically it is a pipe of constant section being pushed along without being twisted.

The relation to Frenet comes straight from the two angular velocities. They differ only by `$\tau T$`, so the rotation-minimizing frame turns against Frenet's about the tangent at the rate of the torsion:

`$$\phi(s)=\pm\int_0^s\tau\,ds.$$`

For a planar curve, `$\tau=0$`: so the plane's normal, carried along, remains the plane's normal, and the frame in the plane is simply "the other axis". That is the exact statement in the code: "a rotation-minimizing frame, which for a planar path is the frame that keeps the path's plane normal fixed".

<div class="kernel">

**In the kernel.** The kernel's `reflected` in `lib/geop-lib-sweep/src/path_sweep.rs` computes the angular velocity per unit of the curve's parameter as `$T\times C''/\lvert C'\rvert$`. Check that against `$\omega=T\times dT/ds$`: since `$dT/ds=\bigl(C''-T(T\cdot C'')\bigr)/\lvert C'\rvert^2$`, and `$T\times T=0$`, we get `$\omega=T\times C''/\lvert C'\rvert^2$` per unit of length, and one factor of `$\lvert C'\rvert$` for per unit of parameter. That is the formula in the code.

</div>

## Double reflection

We need to integrate this frame along a curve. The textbook route would be a Runge–Kutta step on `$\dot U=\omega\times U$`, with renormalization. Wang, Jüttler, Zheng and Liu (2008) found something much simpler and better: compute each step as a product of two reflections.

Let `$x_i,t_i$` and `$x_{i+1},t_{i+1}$` be two consecutive points with their unit tangents, and `$r_i$` the frame's vector at `$x_i$`.

1. Reflect in the plane bisecting `$x_i$` and `$x_{i+1}$`, whose normal is `$v_1=x_{i+1}-x_i$`:
   `$$r^L=r_i-\frac{2\,v_1\cdot r_i}{v_1\cdot v_1}\,v_1,\qquad t^L=t_i-\frac{2\,v_1\cdot t_i}{v_1\cdot v_1}\,v_1.$$`
2. Reflect in the plane that takes `$t^L$` onto `$t_{i+1}$`, whose normal is `$v_2=t_{i+1}-t^L$`:
   `$$r_{i+1}=r^L-\frac{2\,v_2\cdot r^L}{v_2\cdot v_2}\,v_2.$$`

Why this is natural: the first reflection carries the point `$x_i$` exactly onto `$x_{i+1}$`. Since the chord `$v_1$` runs nearly along the tangent, that reflection almost reverses the tangent: `$t^L$` is close to `$-t_{i+1}$`, so `$v_2$` is close to `$2t_{i+1}$`. The second reflection, in a plane through `$x_{i+1}$` nearly square to the curve, turns the tangent exactly onto `$t_{i+1}$` and hardly touches anything square to it. On a circular arc, this product is the exact rotation about the arc's axis (exercise 4). Two reflections compose to a rotation, so the frame stays orthonormal to rounding with no need to renormalize, and the method's global error falls as the fourth power of the step. The demo below reports this: halve the steps and the error falls by about sixteen. The test `converges_at_fourth_order` in `book/scripts/demo_tests/helix-frames.test.mjs` measures a ratio in that range.

<div class="kernel">

**In the kernel.** `reflect` in `lib/geop-lib-sweep/src/path_sweep.rs` is one such step applied to a vector, and `reflected` carries a start frame along a curve by repeating it. The doc comment cites the paper. Frames there are computed in plain `f64`, and this is permitted by the "free choice" rule of Lecture 5: along a spline, "which is approximated anyway, the frames [...] are a free choice: the swept body is defined by them, every edge and face built from the same frames, so it is consistent whatever they are exactly".

</div>

## The helix: Frenet, RMF and screw motion

A helix of radius `$r$` rising `$p$` per turn is
`$$C(\theta)=(r\cos\theta,\ r\sin\theta,\ c\,\theta),\qquad c=\frac{p}{2\pi}.$$`
Its speed is `$\lvert C'\rvert=\sqrt{r^2+c^2}$`, and the *pitch angle* `$\alpha$` with `$\tan\alpha=c/r$` is the slope of the tangent over the horizontal. A short calculation gives

`$$\kappa=\frac{r}{r^2+c^2},\qquad \tau=\frac{c}{r^2+c^2}.$$`

Every point of the curve looks the same: the **screw motion** "turn by `$\phi$` about the axis and rise by `$c\phi$`" takes the helix to itself. That gives a third natural frame: carry a frame at the start by the screw motion. Then `$U(\theta)$` can be taken as the radial direction `$(\cos\theta,\sin\theta,0)$`, and since the principal normal of a helix is minus the radial direction, this frame is the Frenet frame up to a fixed rotation.

Which frame is right for a coil spring? They differ. Combine the rate of Frenet against RMF with the closed form: per unit of the turning angle `$\theta$`, the twist between the screw frame and the rotation-minimizing frame is

`$$\frac{d\phi}{d\theta}=\tau\,\lvert C'\rvert=\frac{c}{\sqrt{r^2+c^2}}=\sin\alpha.$$`

So **per full turn, a rotation-minimizing frame rolls `$360°\sin\alpha$` against the screw frame**. Take a spring of radius 10 rising `$20/3$` per turn: `$c=1.061$`, `$\sin\alpha=0.1055$`, so `$38.0°$` per turn. Over three turns the section would have rolled `$114°$` against the coil's own axis.

For a round wire that does not matter: a circle rolling about its centre is the same circle, though the surface parametrization does change. For a square-section coil it is very visible. With an RMF, the square's sides gradually turn away from the axis, and with a screw frame they stay facing it. A designer drawing a square-section spring means the screw frame, whereas a designer routing a rectangular duct round a bend means the rotation-minimizing one.

<figure class="demo" data-demo="helix-frames">
<figcaption>Each frame is computed. The ribbon is what one frame axis sweeps. Choose "rotation-minimizing": the orange axis drifts against the coil, and the plot shows the double-reflection twist (orange) on top of the torsion's integral (green); the readout gives both, and how many steps it took. Lower the steps to watch them part. Choose "Frenet" on the curve with an inflection to see the flip. This is the mathematics of frames; what the kernel does on a helix path is described below and may differ.</figcaption>
</figure>

## Closing a path

For a closed path a new problem appears. Carry a frame all the way round with the rotation-minimizing rule. It need not come back to where it started.

Here is the geometric meaning. The unit tangent `$T(s)$` traces a closed curve on the unit sphere, the *tangent indicatrix*. Carrying `$U$` with no rotation about `$T$` is exactly parallel transport of a tangent vector on that sphere, because `$U$` is perpendicular to the point `$T$` and changes only along `$T$`. By Gauss–Bonnet the parallel transport round a closed spherical curve turns a vector by the enclosed area:

`$$\text{mismatch}\equiv 2\pi-A\pmod{2\pi},$$`

where `$A$` is the area enclosed by the indicatrix. For a planar closed curve with no self-crossings the indicatrix is a great circle, covered once, enclosing `$A=2\pi$`: mismatch zero. That is why a sweep round a planar circle or a planar rounded rectangle closes without any care.

For a nonplanar closed path, a frame computed by the RMF rule generally arrives turned by some angle. A real sweep must then do something: distribute that angle evenly along the path as extra twist, or force an integer number of turns, or refuse. Which of these a kernel does is a design decision.

<div class="kernel">

**In the kernel.** As of this writing, `followed` in `lib/geop-lib-sweep/src/path_sweep.rs` carries the frame along a closed chain and takes the last span's end row from the first station, with nothing done about a mismatch; the closed sweeps the test module of that file builds are planar (a torus round a circle, and a frame round a square). A helix given as a path curve of several spans goes the general way too: `kind` sees no single arc in it, so it is carried as any spline is, by the rotation-minimizing frame. `Control` refuses twist, scale and rails on a closed path ("a closed path sweeps a ring, which a twist, a scale or a rail would not close"). **What the kernel does for a nonplanar closed path is the part of this lecture most likely to change**: the file is being worked on as this is written, so check the source.

</div>

## From frames to a surface

A frame at each of a finite number of stations does not yet give a surface. Consider which spans are exact.

**A line** moves the profile straight on. `Span::Line`, degree 1. Exact.

**A circular arc** turns the profile about the arc's axis. Every profile point moves on a circle, so `Span::arc` gives a rational quadratic row, as in the revolve. It is the same kind of span, and the wall is a piece of torus. Exact.

So a path of lines and arcs yields an entirely exact sweep, and for such a path the rotation-minimizing frame is also exact: on a line nothing turns, and on an arc the frame turns about the arc's axis.

**Any other curve** has no swept surface that is a NURBS in general: the frame's trigonometry is not rational in the curve's parameter. So it is approximated: sections of the profile are placed at several parameters, and the frames between them are interpolated by cubic Hermite interpolation of the frames (`hermite`), giving a `Span::Curve` of degree 3 whose middle rows are frames a third of a step along the derivatives at the sections. Because a row is linear in the profile coordinates (`$p=o+xe_1+ye_2$`), interpolating the frames interpolates every profile point at once with the same degrees and knots, and so the grid of Lecture 26 is unchanged.

That leaves "is this curve a line, an arc or something else?". `kind` in `path_sweep.rs` asks that on the control points: an arc is a rational quadratic Bézier with legs of one length and a middle weight the cosine of half the turn, with those checked by `could_be_equal`, "so a curve that could be one is taken as one".

Note the tension with AGENTS.md's *Everything is a NURBS curve or surface*: isn't `kind` a type dispatch? It does not branch on a stored kind, and everything that leaves it is a NURBS, but it does choose a more exact representation when the control points admit one. This is where the kernel draws the line. The result of a line or an arc span has an exact swept surface available, which no amount of generality in the fitted path could recover, since a fit could only approximate it. The recognition chooses what to *build*, and no downstream algorithm looks at the outcome and takes a special path.

### The screw sweep

There is a second way to sweep along a helix, and it does not use the frames above at all. `screw` in `lib/geop-lib-sweep/src/revolve.rs` sweeps a profile drawn in `$(r,z)$`, a half-plane through the axis. `screw_path` gives stations with `$e_1$` radial and `$e_2$` the axis itself: the profile is not square to the helix but contains the axis, and it moves by the screw motion. That is what a thread is defined by, and thread standards give the tooth's section in an axial plane.

The rows come from `helix_rows` in `core/geop-core-geometry/src/nurb_curve/helix.rs`: a helix is not a rational curve, since its height grows linearly with the angle, and no rational parametrization of a circle turns at a constant rate. So it is approximated, one rational quadratic per quarter turn at most, with the circle part *exact*:

> The curve lies exactly on its cylinder: seen down the axis it is the kernel's own circle. Only the height is approximate. It rises from span end to span end exactly as a helix does, the middle control row sits at the height halfway between [...] and in between the height runs ahead of or behind the true helix by at most 0.53% of the pitch for quarter-turn spans.

The module quotes a measured `$5.3\times10^{-3}$` times the pitch, tested there, and points out that for an ISO metric M6 thread (pitch 1 mm) that is 5 µm, well inside the thread's own tolerance.

`NurbCurve3D::helix` builds the helix as a curve from the same rows, which is why its projection down the axis is exact.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"coil_spring","example":"coil_spring"}'>
<figcaption>A coil of square section, radius 10, swept through three turns, as the kernel meshed it. Its section lies in the zx plane, drawn square to the path only approximately, and the program is a sketch, a 3-D sketch of a helix, and a sweep with orientation <code>follow_path</code>.</figcaption>
</figure>

## Corners

A path need not be smooth. Where two lines meet at an angle, a frame jump gives a problem: the two walls would not meet at the corner. The classical answer is a **mitre**: at the joint, the profile is projected along either line onto the plane bisecting the corner. Then the two straight walls meet there exactly, since each is a translation of that section, and a section projected along the line stays on the wall. Sweeping a square round a square gives a picture frame, with mitred corners. (The test `sweep_round_a_square_is_a_frame` builds exactly that.)

At any other corner, at an arc or a spline, the projected section lies on neither swept wall. So the walls either side are put on the surfaces the rigid profile sweeps along its own curve, and the *body is rebuilt around them*: a vertex moves to where the surfaces meet, and an edge to where they cross. That is the one rebuild of [Lecture 30](./30-one-rebuild.md). Because `geop-lib-sweep` cannot depend on the shell crate (its tests sweep their solids), the rebuild is handed in as a `Rebuild`, and without one such a corner is refused by name.

Tight bends are a separate hazard. If the profile reaches across the axis of an arc, its walls would pass through each other, a self-intersection no boolean should be left to find. `clear_of_bends` refuses this at the outset, naming the curve of the profile and the bend. It decides on the control points of each curve's quarters, which enclose the curve, so that the refusal is proven rather than sampled. This illustrates AGENTS.md's *Refuse what you do not support: early, by name, and say why*.

## What else travels

`Control` in `path_sweep.rs` lets the profile change on the way:

- `Orientation::FixedNormal` keeps the profile facing as drawn: every profile point then travels a *translated copy of the path*, so every span is the path's own degrees, knots and weights, and is exact. It only requires the path to keep passing through the profile's plane, which `translated` checks on control points.
- **Twist** and **scale**, growing evenly with the length travelled.
- **Rails**: where a guide crosses the section is where the profile's point goes. A single rail turns and scales the section uniformly, and two give a linear map of the section's plane.

The example `horn` in `ops/geop-ops-extrude-revolve/src/examples.rs` is a circle swept along a straight path whose size is set by a rail. Each such case is no NURBS in general, so every span is sampled, except a line along which the map changes linearly, and that remains exact. As you would now expect, "exact where exact is possible, enclosed by construction elsewhere" is the rule throughout.

<div class="recap">

## Recap

- A sweep's frame has one free degree of freedom at every point, the roll about the tangent, and the result depends on it.
- The Frenet frame is canonical, but undefined at inflections and on lines, and it follows the torsion.
- A rotation-minimizing frame has angular velocity `$\kappa B$`, is defined everywhere, and rolls against Frenet at the rate of the torsion.
- Double reflection carries it: two reflections, exactly orthonormal, fourth order.
- On a helix an RMF rolls `$360°\sin\alpha$` per turn against the screw frame. A screw motion is what a thread and a square coil usually mean.
- On a closed path, an RMF mismatches by the area its tangent indicatrix encloses.
- Lines and arcs sweep exactly. Other curves are skinned through sections, frames interpolated, and this keeps the grid.

</div>

## Exercises

**1.** Compute the Frenet quantities of the helix `$C(\theta)=(r\cos\theta,r\sin\theta,c\theta)$` and show `$\kappa=r/(r^2+c^2)$` and `$\tau=c/(r^2+c^2)$`.

<details class="solution"><summary>Solution</summary>

`$C'=(-r\sin\theta,r\cos\theta,c)$`, `$\lvert C'\rvert=\sqrt{r^2+c^2}=:v$`. `$T=C'/v$`, so `$dT/ds=C''/v^2=(-r\cos\theta,-r\sin\theta,0)/v^2$`. Hence `$\kappa=r/v^2$` and `$N=(-\cos\theta,-\sin\theta,0)$`. Then `$B=T\times N=(c\sin\theta,-c\cos\theta,r)/v$`, `$dB/ds=(c\cos\theta,c\sin\theta,0)/v^2=-(c/v^2)N$`, and `$dB/ds=-\tau N$` gives `$\tau=c/v^2$`.

</details>

**2.** A planar curve in `$z=0$` with `$r_0=(0,0,1)$`. Show that double reflection leaves `$r$` unchanged at every step, exactly, whatever the step size.

<details class="solution"><summary>Solution</summary>

`$v_1$` and `$t_i$` lie in the plane, so `$t^L$` does, and so does `$v_2=t_{i+1}-t^L$`. Both `$v_1\cdot r_0$` and `$v_2\cdot r^L$` vanish, so neither reflection moves `$r$`. This is the kernel's "exact for a planar curve".

</details>

**3.** A rectangular duct (not a circle) is swept along a helix of radius 40 and pitch 50 by an RMF over two turns. By how much has the section rolled against the coil axis at the end?

<details class="solution"><summary>Solution</summary>

`$c=50/2\pi=7.958$`. `$\sin\alpha=c/\sqrt{r^2+c^2}=7.958/\sqrt{1600+63.3}=7.958/40.78=0.1951$`. Per turn `$360°\cdot0.1951=70.2°$`, so `$140.5°$` in two turns.

</details>

**4.** Show that one step of double reflection on a circular arc, of any size, is exactly the rotation about the arc's axis: compute `$t^L$` and `$v_2$`, and follow the two vectors of the frame.

<details class="solution"><summary>Solution</summary>

Put the arc in its plane, with the chord along `$x$` and the bisector along `$y$`: the point at angle `$\varphi$` is `$R(\sin\varphi,\cos\varphi)$` for `$\varphi\in[-\theta/2,\theta/2]$`, and the tangent is `$(\cos\varphi,-\sin\varphi)$`. So `$t_i=(\cos\tfrac\theta2,\sin\tfrac\theta2)$` and `$t_{i+1}=(\cos\tfrac\theta2,-\sin\tfrac\theta2)$`. The first reflection negates `$x$`, which gives `$t^L=(-\cos\tfrac\theta2,\sin\tfrac\theta2)=-t_{i+1}$`. Hence `$v_2=2t_{i+1}$`. The arc's normal `$z$` is square to `$v_1$` and to `$v_2$`, so it never moves. The radial vector `$(\sin\varphi,\cos\varphi)$` at `$\varphi=-\theta/2$` goes to `$(\sin\tfrac\theta2,\cos\tfrac\theta2)$`, the radial vector at the other end, which is square to `$v_2$`, so the second reflection leaves it. Both results are what the rotation by `$\theta$` about `$z$` gives.

</details>

**5.** What is the mismatch, by the area formula, of a closed path whose tangent indicatrix is a small circle at colatitude 60° covered once? Is such a path possible with this exact indicatrix?

<details class="solution"><summary>Hint</summary>

The cap of colatitude `$\beta$` has area `$2\pi(1-\cos\beta)=\pi$` for `$\beta=60°$`, so the mismatch would be `$2\pi-\pi=\pi$`. But a closed curve has `$\oint T\,ds=0$`, so its indicatrix cannot lie in an open hemisphere, and this small circle does. No closed curve has it. Two half circles sharing a diameter, in planes at an angle, are no real case either: unless they lie in one plane their tangents jump at both ends of the diameter, and with the frame turned the shortest way at each jump, the two turns cancel and the mismatch is zero. For a real case, take a coil closed on itself round a ring, `$((3+\cos5t)\cos t,\ (3+\cos5t)\sin t,\ \sin5t)$`: double reflection measures a mismatch of about `$155°$`, the integral of its torsion modulo a full turn.

</details>

**6.** Find what `Orientation::FixedNormal` checks about the path in `translated` and why it is enough to check control points.

<details class="solution"><summary>Solution</summary>

The heights of the control points above the profile's plane must rise all the way along. A NURBS with positive weights varies no more than its control polygon, so if the polygon's height is monotone then so is the curve's, and the path keeps running through the plane the way it started. So the walls cannot fold over. The doc comment on `translated` gives this argument.

</details>
