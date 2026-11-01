# Lecture 22: From remesh to result

<div class="goals">

**After this lecture you can**

- classify a face against a solid, and say what to do with a face lying on the other solid's boundary;
- derive the keep table for union, intersection, difference and symmetric difference from set algebra and check it against the code;
- assemble the result, and explain why an empty result is an answer;
- use the naming scheme to say what any entity of a result is, and explain how names stay independent of the order of the work;
- explain "up to next", and why a tool must never be stopped on an existing corner;
- describe `split`, and say what grew with every cut in a gear.

</div>

[Lecture 21](./21-remesh.md) ended with two bodies whose faces lie wholly inside the other, wholly outside it, or on its boundary. "The algorithm is deliberately simple," begins `lib/geop-lib-booleans/src/boolean.rs`, "and it is simple *because* remesh has already done the hard part." A boolean becomes a per-face classification followed by a table.

Here is the whole thing, as the module comment gives it:

1. Remesh the two solids against each other.
2. Classify each face, by taking a point strictly inside its trim and asking where it is relative to the other solid.
3. Keep the faces the operator wants, reversing some.
4. Assemble the survivors into a new solid, and discard everything else.

The demo does all four, one dimension down. Choose an operator and drag the vertices: it remeshes, classifies each piece, applies the kernel's table, chains what is kept into loops, and compares the area with the one you get from the areas of the inputs by inclusion and exclusion.

<figure class="demo" data-demo="boolean-2d">
<figcaption>Dashed pieces are dropped, solid ones kept, with arrows the way each runs in the result. Blue is outside the other polygon, red inside, green along its boundary the same way, purple along it the opposite way. Line a side of one polygon up with a side of the other (they snap to a grid) to see a coincident piece, and move one polygon well away, or wholly inside the other, to see the difference empty out and a hole appear. Polygons here are straight, so the "faces" are edges; the classification, the table and the area check are real, with the plain floats on a grid standing in for the kernel's intervals.</figcaption>
</figure>

## Classifying a face

`classify_face` in `lib/geop-lib-booleans/src/boolean.rs` returns a `FaceClassification`: `Inside`, `Outside`, `OnSameNormal` or `OnOppositeNormal`. It works in three steps.

**Find interior points.** `face_interior_point_where` ([Lecture 19](./19-classification.md)) offers points strictly inside the face's trim, in turn, up to two per outer coedge.

**Classify each by `solid_contains`.** `Inside` or `Outside` goes in `decided`. A point *on* the other solid's boundary (`OnFace`, `OnEdge`, `OnVertex`) goes in `on_boundary`. Such a point decides nothing by itself. The face may lie along that boundary over an area, or may merely touch it at a point or along a curve: "a cube face resting on a sphere's pole, say, where the sphere has no normal at all". So it is set aside and the next interior point tried. Any point off the other solid's boundary classifies the whole face, by the remesh's guarantee.

**Check.** If two points of one face are decided differently, then the guarantee failed: "remesh left an intersection curve unimprinted". This is a very useful check. It costs almost nothing, and catches a whole class of remesh bugs where they can still be named. The error prints the face's corners, and what the search makes of the other solid's edges at those corners, to point at the missing curve.

**Only if every interior point lies on the boundary** are the patches coincident, and then normals decide: `$n_a\cdot n_b>0$` is `OnSameNormal`, `$<0$` is `OnOppositeNormal`, and a value that could be zero is an error. The first interior point where both normals exist is used: "a revolve's cap collapses to its pole, where it has none". Note that no epsilon is involved: `definitely_greater` and `definitely_less`, with the third outcome reported.

### Saving rays: sides propagate across untouched faces

A cut leaves most of a gear's faces untouched. It would be a waste to classify each. A face whose trimmed patch is apart from every patch of the other solid (by the boxes of `trimmed_patches`, which uses the box around its pcurves to cut down its surface first: "a gear's cap cut by every tooth is one surface") cannot straddle anything. It "lies on the side of any face it shares an edge with that is inside or outside, all of the edge being apart from the other solid's boundary too." So `combine` classifies only those that could overlap, then floods the classification across edges to the faces that are apart, and classifies only one face of a set that no classified face borders. "What a cut leaves untouched, all but a few faces of a gear once it has many teeth, costs no ray." `faces_classified()` counts these for tests, as in AGENTS.md's *Guard scaling with a count, not a time*.

And each ray that is cast starts from the nearest point already classified (`Classified`, `shell_contains_from`, [Lecture 19](./19-classification.md)).

## The table

Consider what the result of each operator is, as a set, and which pieces of boundary it has.

**Union** `$a\cup b$`. A piece of `$a$`'s boundary is on the result's boundary if it is outside `$b$`: inside `$b$`, it is inside the union. Likewise for `$b$`.

**Intersection** `$a\cap b$`. A piece of `$a$`'s boundary bounds the result if it is inside `$b$`, and likewise for `$b$`.

**Difference** `$a-b=a\cap\bar b$`. The complement `$\bar b$` has the same boundary as `$b$` with the normal reversed. So a piece of `$a$` stays if it is outside `$b$` (inside `$\bar b$`). A piece of `$b$` stays if it is inside `$a$`, and is **reversed**: it now bounds a cavity, and the material is on what used to be its outside.

**Symmetric difference** `$(a-b)\cup(b-a)$`. Each of the two contributes per the above: outside pieces stay and inside pieces stay reversed. So `Outside` is kept, `Inside` is reversed, from either side.

**On the boundary.** Where two faces coincide:

- *Same normal*: the material of both lies on one side of the patch. In a union or an intersection that patch is on the boundary exactly once: keep `$a$`'s copy, by convention, and drop `$b$`'s. In a difference: `$a$`'s material there is removed by `$b$`'s, right up to the patch. It bounds nothing. Drop both.
- *Opposite normals*: the solids meet there from opposite sides. In a union, it is interior: drop both. In an intersection, the intersection has no volume there: drop both. In a difference, `$b$` lay against `$a$` from outside, and `$a$`'s face remains its boundary. Keep `$a$`'s copy, drop `$b$`'s.
- In a symmetric difference, along a coincident patch "the material is on one side of both (in both, so in neither piece) or on either side (in one each, so inside the result): no boundary". Drop.

That gives the following, which is `keeps` in `boolean.rs`, written as a table.

| Face of | Class | Union | Intersection | Difference `$a-b$` | Symmetric |
|---|---|---|---|---|---|
| `$a$` | Outside | keep | drop | keep | keep |
| `$a$` | Inside | drop | keep | drop | **reversed** |
| `$b$` | Outside | keep | drop | drop | keep |
| `$b$` | Inside | drop | keep | **reversed** | **reversed** |
| `$a$` | OnSameNormal | keep | keep | drop | drop |
| `$b$` | OnSameNormal | drop | drop | drop | drop |
| `$a$` | OnOppositeNormal | drop | drop | keep | drop |
| `$b$` | OnOppositeNormal | drop | drop | drop | drop |

`book/scripts/demo_tests/boolean-2d.test.mjs` checks each row of the demo's `keeps` against this table, and the demo's areas against a raster count, for concave shapes. `reverse_face` does the reversing ([Lecture 17](./17-brep.md)): it touches only that face's surface and pcurves, so that the faces stay glued along their shared edges.

### Reading the drilled box's result

Take `box_with_drill_hole` from Lecture 21, a difference. The box is `$a$`, the tool `$b$`.

- The box's five untouched faces: outside the tool, keep.
- The top's ring piece, `extrude(box,end)`: outside. Keep.
- The top's disc: it lies in the plane of the tool's start cap. Both have their normal `$+z$` (each solid's material is below). So `OnSameNormal`: drop.
- The tool's start cap: the same, from `$b$`. Drop.
- The tool's four walls and its floor `extrude(hole,end)`: inside the box. Keep, reversed.

So 5 + 1 + 4 + 1 = 11 faces, as the kernel reports. The floor's outward normal, formerly down (out of the tool, at the tool's bottom), is now up (out of the material, into the hole). Check that in the viewer.

## Assembling

`Model::assemble_solid` (`core/geop-core-topology/src/edit/assemble_solid.rs`) makes one new solid from the faces to keep. Connected sets of those faces become shells: a boolean that cuts a bar in two leaves two lumps, and one that leaves a sphere's room inside a block a lump and a void. Everything not reachable from a kept face is deleted, "by reachability rather than by tracking what was split or re-homed on the way here, so no dangling id can be left behind". Only what the consumed bodies owned can be deleted: other bodies in the model are not touched. Both operands are consumed.

**An empty result is an answer.** `boolean` returns `Ok(None)` when no face is kept: intersecting two solids that do not overlap, or subtracting a solid that wholly contains the first. "Callers that treat 'no solid' as an error would reject the majority of scenes in this crate's own test set." Try this in the demo. This is `Result<Option<SolidId>>` at work: `Err` is "I could not determine this", and `Ok(None)` is "the answer is nothing". They mean opposite things, and conflating them would make a failed classification look like an empty set.

**Closing check.** Before assembling, `check_closed` confirms every edge is used an even number of times by the kept faces. Odd is a hole in the result, "so a face was kept or dropped wrongly, or remesh left a face straddling the other solid". The error lists every face along that edge with its name and classification, "which tells the two apart".

**Merge.** Finally, faces the boolean left in two on one surface are made one (`merge_faces_on_one_surface`, in `ops/geop-ops-model/src/edit.rs`, over `core/geop-core-topology/src/merge.rs`): pieces of one face split along an edge of the other solid that is no edge of the result, and a face of each solid running on into the other's, two blocks standing flush. An edge that parted the same two faces in an operand stays, as the operand had it: a cylinder's quarters, and a face a step divided on purpose. The oldest's name stays and the others' stand for it.

## Names

Later steps refer to what earlier ones built by name, so that a sketch placed on "the top of the box" stays there when an earlier dimension changes ([Lecture 2](./02-programs-of-steps.md)). So booleans have to name everything they create in a way that does not depend on anything accidental. `lib/geop-lib-booleans/src/naming.rs` has the scheme, with `$N$` the boolean's namer:

| Entity | Name |
|---|---|
| the result solid | `$N$` |
| vertex where edges `$E_1<E_2$` cross | `$N(E_1,E_2,i,n)$`: the `$i$`-th of their `$n$` crossings, along `$E_1$` |
| vertex where edge `$E$` pierces face `$F$` | `$N(E,F,i,n)$` |
| vertex inside faces `$F_1<F_2$`, where they touch or a curve they meet in turns back | `$N(F_1,F_2,i,n)$` |
| piece of edge `$E$` starting at vertex `$V$` | `$N(E,V)$`; the piece at `$E$`'s own start keeps `$E$` |
| edge traced along faces `$F_1<F_2$` from `$P$` to `$Q$`, `$P<Q$` | `$N(F_1,F_2,P,Q)$`, with a trailing `,k` where several such edges join them |
| piece of face `$F$` split off along edge `$E$` | `$N(F,E)$`; the other piece keeps `$F$` |

Why these choices?

**Everything is named in terms of what it was before.** Each edge and face has an *origin*: its name before the boolean. A piece of a piece has the same origin as the piece. So a name is a function of the operands, not of the order of splits.

**`<` removes the order of operands.** It is the order of names as strings, so which solid came first, or which pair the search reached first, does not matter. "The one exception is the trailing `,k` of a traced edge: two intersection branches with the same ends on the same faces (an arc and its complement) are told apart only by when they were traced."

**`i` and `n` are not known until the end.** So `BooleanNaming` hands out provisional names (`provisional`), records a `Recipe` of what each entity was made from, and `finish` settles all names at once.

Now read the shaft's vertex name again: `combine(bore,extrude(bore,bore_sketch,c1@3),revolve(shaft,section,c9,a0),0,1)`. The namer is `combine(bore,...)`. The first edge is the bore's seam `c1@3`, and the second is the shaft's vertical seam `c9,a0`. It is the 0th of 1 crossing along the first. Everything you need to find this vertex in the model is in the name, and it would be the same if the kernel found its crossings in another order.

### What grows with every cut: names

AGENTS.md's first item in *Many booleans on one body*. "A piece of a split edge is named after the edge and the vertex it starts at, and that vertex after the edge again, so the name doubled per cut: twenty cuts made names of tens of megabytes." A name's argument longer than `LONGEST_ARGUMENT` (160 characters) is now spelled by its digest (`ops/geop-ops-model/src/names.rs`). A digest does the name's job, which is identity, at a fixed size.

### A trimmed face is still that face

When a face splits, one piece keeps the name `$F$`, and the other is `$N(F,E)$`, "whichever one the splice left the name on". If the boolean then keeps only one of them, that piece *is* the face, trimmed. But it may be the one that lost the name. AGENTS.md: a fillet rolled instead of extruded split a box's bottom along its tool the other way round, and a draft about the bottom no longer found it. So `name_lone_pieces` gives the one remaining piece of an edge or a face the name of what it is a piece of, where nothing has that name any more. Its piece's name stands for it too. Which piece the splice happened to leave the name on is the splicing's choice, not a fact about the result.

## Up to next

An extrude "up to next" should go up to the next face the tool meets. How? You might think: find the first face hit along the direction, and stop there. It is much harder than that, and getting it wrong taught one of the kernel's rules.

### Never place a cut on a corner that already exists

The story from AGENTS.md again ([Lecture 20](./20-validation.md)): a revolve cut "up to next" failed with a vertex too wide to validate. The vertex was where the drilled hole's rim crossed the revolve's *end face*, and the end face was there because a fallback had stopped the tool flat at the first point of contact, which was an existing corner. A choice like that asks the boolean to cross an edge at its own end point, "a degenerate configuration no enclosure stays tight through". The rule: when an operation chooses where a new face goes, a choice that lands it exactly on an existing vertex or edge of the other body is a poor one. Choose positions the geometry actually defines.

### Up to next by pieces

So `boolean_up_to_next` does not choose a stop position at all. It works on a tool long enough to reach past whatever stops it. The remesh cuts the tool along the target's boundary, into pieces that are alternately outside and inside the target. Then:

- For a union, the piece wanted is the first outside the target that the tool's start face reaches, growing until it runs into the target.
- For a difference or an intersection, it is the first inside: cutting, or keeping, from where the tool enters the target until it comes out.

`reach_up_to_next` finds it. Each piece is bounded by tool faces, all on one side of the target, and by target faces inside the tool, each of which bounds two pieces. "Faces sharing an edge bound the same piece if their material lies on the same side of the target", which makes the pieces connected sets of face *sides* (via a union-find), and two pieces neighbours when a target face lies between them. From the piece with the start face, take the start's own piece if it is of the kind wanted, or else the pieces next to it. Drop the rest of the tool.

If the piece also reaches the tool's end face, nothing stopped it: `Reach::RunsOn`, not an error, so the step can go another way, such as all the way round or through all. A symmetric difference has no next face and says so.

The elegance is that the stop surface is never chosen, and the tool's own geometry is cut by the target's faces, wherever they are. It is the same remesh and the same classification. `piece_up_to_next` does it for a new body, cutting the tool with a copy of what it stops at, and `trim_up_to_next` in `lib/geop-lib-booleans/src/trim.rs` for a sheet.

Here you can see a general design pattern. Instead of adding a new operation with its own geometric reasoning, the code reinterprets the result of an operation it already has. AGENTS.md's *Where the topology changes, reuse the boolean* states that as a rule, and [Lecture 30](./30-one-rebuild.md) uses it again.

## Split

`split` in `lib/geop-lib-booleans/src/split.rs` cuts a solid into pieces with a sheet. A copy of the sheet is remeshed with the solid; pieces of it outside the solid go, and those inside are the cuts. Each cut becomes a face of *two* pieces: as it is on the side its normal points away from, and turned around on the other.

Which face of the solid is on which side of a cut is purely topological. At an edge where the sheet crossed the solid's boundary, there is a face of the solid on either side. "Of the two, the one running along the edge the other way than the cut face does is the one on the cut face's side: two faces of a closed, oriented shell always run their shared edge in opposite directions." Elsewhere, faces sharing an edge bound the same piece. So pieces are the connected sets of face *sides*, each built as a solid of its own (`NamedModel::build_body`). Pieces are named `$N(0)$`, `$N(1)$`, ..., ordered by the smallest name of a face they kept. A sheet ending inside the solid, or along which it stays connected, splits nothing and is an error.

Which pieces of the sheet are inside the solid is asked by `classify_face`, as in a boolean. Which face of the solid goes with which cut needs only the orientation of the edges.

## Many booleans on one body

Two items of AGENTS.md's list remain, in addition to rescans and split-edge width ([Lecture 21](./21-remesh.md)), names, and untouched faces (above). *Traced fits* are about the involute flank's curvature changing tenfold, so that the cubic through the march drifted, and the legs are now split finer while the drift is what makes the curve wide ([Lecture 16](./16-tracing-and-pcurves.md)). And *the sketch solver*: a sketch whose arcs' sweeps must be solved for can be sent off to infinity, and a sketch every point of which a formula places is placed in one step ([Lecture 25](./25-sketch-features.md)).

The overall picture: a process that is harmless once may grow without bound when repeated a hundred times. A gear is the test. Standard-part gears are in `lib/geop-lib-stdlib/src/involute.rs`, generated in Rust from a table.

<div class="kernel">

**In the kernel.** `combine_tools` in `lib/geop-lib-booleans/src/combine.rs` is what an extrude, a revolve, a hole, a rib or a pattern does with the tools it built, as the `Combine` of its step says: keep them as a new body, or unite, intersect or subtract each in turn with the target. The operations a program uses directly are in `ops/geop-ops-boolean`: `Boolean`, `Split`, `Imprint`, `Section`. They resolve references and call this library, according to AGENTS.md's *A lib takes geometry; an op resolves references*.

`combine` in `boolean.rs` adds the names of what an error mentions (`names_mentioned`) to both remesh and classification errors. It has the whole shape of this lecture in about 200 lines. It is a good file to read once you've finished.

</div>

<div class="recap">

## Recap

- Classify a face by points strictly inside it. A point on the other's boundary says nothing; if all are, the patches coincide and normals decide. Two points that disagree name a remesh bug.
- The table is set algebra. `$a-b$` keeps `$a$` outside and `$b$` inside, reversed; a coincident patch is kept at most once, always from `$a$`.
- `Ok(None)` is an empty answer, and `Err` means it could not tell.
- Names are functions of the operands' names, sorted to remove order, settled at the end, digested if long, and given back to a lone trimmed piece.
- Up to next chooses no stop position: it keeps the first piece of a tool long enough to pass, so a cut is never placed on an existing corner.
- A split's pieces are connected sets of face sides, decided topologically.

</div>

## Exercises

**1.** Derive `$a\cup b$`'s row for a face of `$a$` from `$a\cup b=\overline{\bar a\cap\bar b}$`, and confirm that it agrees with the table. Then do the same for symmetric difference on a coincident patch.

<details class="solution"><summary>Solution</summary>

`$\bar a$` and `$\bar b$` have reversed boundaries. Under `$\bar a\cap\bar b$`, a piece of `$\partial a$` is kept when it is inside `$\bar b$`, that is, outside `$b$`; complementing the whole reverses every kept face, undoing the first reversal. So it is kept, as is, when outside `$b$`: the table's first row under Union. On a coincident patch with same normal, in `$(a-b)\cup(b-a)$`: both have material on one side, so that side is in both and hence in neither difference; the other side is in neither. No boundary. With opposite normals: one side in `$a$` only, the other in `$b$` only; both in the result, so no boundary. Drop.

</details>

**2.** The closing check counts edges used by kept faces. Construct a mistake in `keeps` that this check catches, and one it does not.

<details class="solution"><summary>Solution</summary>

Caught: keep a piece of `$b$` that should be dropped, with a rim shared by its neighbours. Its edges are then used an odd number of times. Not caught: reversing a face that should stay as is: each edge is still used the same number of times. That is caught later by `check_normals_point_outward`, in validation. Also not caught: using one operator's whole column for another, as that gives a closed result of the wrong set. Hence the area check in the demo.

</details>

**3.** Use the demo with a square `$a$` and a square `$b$` sharing one side, with both on grid points. For each operator, list the classes and verify the areas by hand. Do it again with `$b$` directly over `$a$`.

<details class="solution"><summary>Solution</summary>

With unit squares side by side, the shared side has opposite directions, so `OnOppositeNormal`. Union: outer sides kept, shared side dropped both ways, area 2. Intersection: nothing kept, area 0, empty. Difference: `$a$`'s three outer sides kept, as `Outside`, and its shared side kept by the `OnOppositeNormal` row, `$b$` all dropped; area 1. Symmetric: outer sides of both kept, shared side dropped, area 2. For `$b=a$`, all pieces are `OnSameNormal`: union and intersection keep `$a$`'s copy, area 1 each; difference and symmetric drop all, area 0. `book/scripts/demo_tests/boolean-2d.test.mjs` asserts these.

</details>

**4.** Two rectangular cuts are made, one after another, in a bar, with the second overlapping the first. Using the naming table, what is the name of the piece of an original edge that lies between them, in terms of `$N_1,N_2$`? What is the worst case for the length of its name after `$k$` cuts, without `LONGEST_ARGUMENT`?

<details class="solution"><summary>Solution</summary>

After the first, the piece starting at the crossing vertex `$V_1=N_1(E,F,0,1)$` has name `$N_1(E,V_1)$`, containing `$E$` twice. After the second, a further piece from `$V_2=N_2(N_1(E,V_1),G,0,1)$` is `$N_2(N_1(E,V_1),V_2)$`, containing the previous name twice. Length doubles per cut: `$O(2^k)$`. That is AGENTS.md's "tens of megabytes" at twenty cuts. With a digest of arguments longer than 160, each name stays under a fixed size per argument.

</details>

**5.** Why does `boolean_up_to_next` require a tool "long enough", and how does `Reach::RunsOn` differ from an error? Who decides what happens next?

<details class="solution"><summary>Hint</summary>

The piece is found after the boolean's remesh, so the tool must really pass whatever stops it. If the piece reaches the end face, nothing stopped it. `Reach::RunsOn` is a normal answer carrying a `GeopError` as an explanation: a step may then go another way, through all or all the way round. `Reach::stopped` turns it into an error for callers who want that. Compare AGENTS.md's *Never place a cut on a corner that already exists*: "going up to the next face from where the tool enters the target, or all the way round or through all if part of it never meets the target again".

</details>

**6.** Suppose `classify_face` got one sharp interior point for each face and did not look for a second. Which remesh bugs would go unnoticed, and what would they look like?

<details class="solution"><summary>Solution</summary>

A missed intersection curve leaves a face straddling. With one point, it is classified by whichever side the point falls, and kept or dropped whole. That leaves either a surplus piece of material in the result, or a hole, which the closing check may or may not catch. The code offers more than one point and stops only when all offered agree, to catch it there: `face_interior_point_where` with `accept` returning `false`, so that all candidates are collected. This catches a straddling face whenever two of its boundary coedges' interior points happen to lie on different sides, and that is frequent, though it is no proof either.

</details>
