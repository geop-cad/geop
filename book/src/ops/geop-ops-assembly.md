# geop-ops-assembly

> Brief overview only — full documentation is coming later.

Assemblies: parts placed in a part — another program file's part, built
once by a `Library` and placed by `AddPart` at a pose its program's state
holds — and mated to what is already there. Where every placed part is, is
the program's state, which the editor solves the mates for; dragging a
placed part moves it as far as its mates let it. `PartPattern` places
copies of a placed part in a row or round an axis. Also what reads a part
with the parts placed in it: their solids, each of its own part's material
(`solids`), and an STL file of them all (`Stl`).

## Arguments, and where the part goes

```json
{ "steps": [
    { "id": "pin", "operation": "add_part",
      "args": { "file": "pin.geop",
                "mates": {
                  "m1": { "type": "concentric",
                          "entities": [
                            { "part": "pin", "on": { "face": "extrude(pin,pin_sketch,c1)" } },
                            { "on": { "face": "extrude(hole,hole_sketch,c1)" } } ] } } } } ],
  "state": {
    "pin.pose": { "position": [1.0, 1.0, 0.5], "rotation": [1.0, 0.0, 0.0, 0.0] } } }
```

`file` is relative to the program's own file. A mate holds by `MateEntity`s:
an element `on` of the part placed at `part` — `pin`, or `asm/pin` for one
placed in a placed part — or, with no `part`, of the part itself, which is
ground. `on` is a `Selected` of
[geop-ops-add-datum](./geop-ops-add-datum.md): a vertex, an edge, a face, a
datum (`{"datum": "origin:xy"}`) or a sketch's point or curve. Where the
part goes is not an argument but the program's state `pin.pose` — a
position and a rotation quaternion (see
[geop-ops-parameters](./geop-ops-parameters.md#the-state-state)) — solved
for (see [below](#solving-the-state)). The step places the part there and
adds its mates; it solves nothing. So a later step's mates that move it
move it for every step.

The part placed is an `Instance`, a target of its own (`instance(name)`),
of a `Component`: the part another file builds, built once by the run's
`Library` (the given target `library()`, see
[geop-cad-base](../cad/geop-cad-base.md#the-workspace)) for each set of
parameter values it is placed with, and drawn once however often it is
placed. `placed_parts` lists every placed part, however deep, and `locate`
finds where an element of one is.

## Parameters

`parameters` gives the placed part's own parameters (see
[geop-ops-parameters](./geop-ops-parameters.md#parameters)) other values,
by name: a number, a table's row, the colour (a
`geop_ops_parameters::Settings`). The part is built with them, and its
dialog offers each as what it is: the colour to pick, a number on a slider
over its range, a table's row from a list to search, each showing the value
the part is built with here.

## Mates

A `Mate` is what two entities must be to each other (`MateKind`):

- **constraints** (`Kind`): coincident, concentric, parallel,
  perpendicular, distance and angle, each taking a point, a line or a plane
  of each entity, where its part puts it;
- **joints** (`JointKind`): revolute, slider, cylindrical, fastened — one
  part turning about, or sliding along, the other's axis. A joint is made
  between two frames, one on each part, which a click puts on whatever is
  picked — a face, an edge, a point or a datum — at a corner or side of a
  face or an end of an edge (`MateEntity::frame`, `at`). Its coordinates —
  how far it is turned and slid — are values of the state, named after the
  mate (`joint_parameter`: `add_part(arm2,m1).angle`): set to a value,
  held there, kept within limits;
- **couplings** (`CouplingKind`): two joints tied linearly — a gear pair, a
  rack and its pinion, a screw;
- **fixed** (`{ "type": "fixed" }`): in an `add_part` step and with no
  entity, it holds the part the step places (the first part placed gets
  one).

Every parameter of the program placed becomes one of this program's, named
behind the step's id — `hinge/pin.pose` — and the part is built with those:
its parts are this program's to move, and its mates are solved with this
program's, but not its `fixed` mates: what is fixed in a part placed is
where it is placed. The file is rebuilt incrementally, only from the first
step that reads a value that changed — for a file that only places parts,
only its placements.

## Editing

- The placement fields show a position and angles about x, y and z, and
  pull the part there (`Pull::Pose`).
- Adding a mate arms its entities field. When a mate is added the editor
  solves the program, moving the step's own part first, the others only if
  that is not enough.
- Picks test against the part as the step builds it
  (`OperationInfo::picks_built`), so the placed part's own entities can be
  picked. While a joint's field waits, the hover shows the frame a click
  would put, as three arrows.
- Dragging the placed part — by a gizmo at its origin, or grabbed anywhere
  and dragged in the plane facing the eye — asks for the point grabbed to
  be pulled towards the pointer (`Pull::Point`). The editor solves the
  whole program with that pull, every part that no fixed mate holds free
  to give way: a linkage follows the link dragged. A part no mate may move
  goes where it is pulled.
- A joint's coordinates set in the dialog are held there while the step is
  edited (`Pull::Value`), and the parts move to them.

## Solving the state

What the mates add besides their constraints is said in the part
(`mechanism`): a `fixed` mate declares the state of the part it holds as an
`Unknown` held fixed; a joint declares each of its coordinates as an
`Unknown` that follows its parts — started, where the joint does not hold,
where its parts are, measured from them — and as `solving::Settings`, with
its limits, for the editor to list and set (the joints panel). Its limits
are constraints of their own (`Bound::AtMost`). See
[geop-ops-constraints](./geop-ops-constraints.md) for what those are.

`solving` is what an editor does with a part's state, the same for every
program, on `geop_ops_constraints::solve`: `solve` settles it so every
constraint holds — pulled by a drag (`Pull::Point`), a pose typed in
(`Pull::Pose`), a value set (`Pull::Value`, an angle driven there in
steps) — `set` sets values a user sets, `check` says which constraints
hold, `freedom` how free each placed part is, and `conflicting` which
constraints cannot hold together. Each placed part moves by its pose state;
everything else a solve may move, the steps declare. A solve's `Report`
says which values no true solution is proven for, and why.

## Solids and STL

`solids` lists the solids a part shows, wherever they are placed — its own
and those of the parts placed in it, however deep — each of its own part's
material, with their mass properties: what weighing a part, checking it for
interference, ordering it and simulating it read. `Stl` is the STL exporter
(see [geop-ops-output](./geop-ops-output.md#exporters)): the mesh the
editor draws, the parts placed included, each where it is; its options
choose the solids (`solids`), text or binary (`ascii`) and how finely
curved faces are meshed (`quality`).
