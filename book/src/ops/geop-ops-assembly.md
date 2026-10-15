# geop-ops-assembly

> Brief overview only — full documentation is coming later.

Assemblies: parts placed in a part — another program file's part, loaded
through the runner and placed by `AddPart` at a pose its program's state
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
                            { "on": { "face": { "target": "pin/model", "name": "extrude(pin,pin_sketch,c1)" } } },
                            { "on": { "face": "extrude(hole,hole_sketch,c1)" } } ] } } } } ],
  "state": {
    "pin.pose": { "position": [1.0, 1.0, 0.5], "rotation": [1.0, 0.0, 0.0, 0.0] } } }
```

`file` is relative to the program's own file. A mate holds by `MateEntity`s:
an element `on` — a `Selected` of
[geop-ops-add-datum](./geop-ops-add-datum.md): a vertex, an edge, a face, a
datum (`{"datum": "origin:xy"}`) or a sketch's point or curve — of the part
itself, which is ground, or of a part placed in it, its target behind the
part's path: `pin/model`, or `asm/pin/model` for one placed in a placed
part (see [geop-core-program](../core/geop-core-program.md#instances)). Any
other step's reference takes an element of a placed part the same way: a
datum on a placed part's face, a pattern round a placed part's hole. Where the
part goes is not an argument but the program's state `pin.pose` — a
position and a rotation quaternion (see
[geop-ops-parameters](./geop-ops-parameters.md#the-state-state)) — solved
for (see [below](#solving-the-state)). The step places the part there and
adds its mates; it solves nothing. So a later step's mates that move it
move it for every step.

The part placed is an instance of the part (see
[geop-core-program](../core/geop-core-program.md#instances)): the part
another file's program builds, loaded by the runner of the step from its
workspace (see [geop-cad-base](../cad/geop-cad-base.md#the-workspace)),
its parameters set to the step's `parameters` as its inputs, and run with
the state under its name. The part placing it keeps its part and carries
it over whole when nothing it is given changed. Where it is placed is a
target of its own (`geop_ops_parameters::define_instance`: a `Placement`,
and the file and inputs it was loaded from). `define_placed` loads it,
places it and draws it as a part is drawn (`draw`: its model, datums,
sketches, annotations and the parts placed in it, in its colour; a
`Drawn`, the target `drawn:<name>`), again only when what the drawing read
changed: moving it builds and draws nothing. A drawing's key is a hash of
what it shows, so a viewer is sent a part drawn alike — every screw of a
size — once. A pattern's copy loads again the file and inputs the part it
copies was loaded from, run with its own state. `testing` (feature
`testing`) makes parts in Rust the programs of a workspace, for the tests
of what reads placed parts. `placed_parts` lists every
placed part, however deep, `locate` where one is, and `drawn` how it is
drawn.

## Parameters

`parameters` gives the placed part's own parameters (see
[geop-ops-parameters](./geop-ops-parameters.md#parameters)) other values,
by name: a number, a table's row, the colour (a
`geop_core_program::values::Settings`). The part is built with them, and its
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

Every value of the state of the program placed becomes one of this
program's, named behind the step's id — `hinge/pin.pose` — and the part is
built with those: its parts are this program's to move, and its mates come
up with its constraints, solved with this program's. What it holds fixed
stays fixed in it, which moves as a whole.

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
constraints cannot hold together. Everything a solve may move the steps
declare: `AddPart` the pose state of the part it places, held where it is
as far as its mates let it. A solve's `Report`
says which values no true solution is proven for, and why.

What the crate adds to an editor (`register`, see
[the registry](../core/geop-core-editor.md#the-registry-registry)) is built
on `solving`: the `Settler` the editor settles the state with after every
edit (what the step edited pulls first, else the step's own part first,
then every part not fixed); the joints panel (`JointsPanel`, key `joints`:
the values a user sets, how free each placed part is, measured once an
edit, and the constraints the last settle found conflicting; it takes
`set` to set a joint's coordinate); the drag tool (`DragTool`, key `drag`:
with no step edited, a placed part grabbed is pulled towards the pointer,
a whole drag one edit to undo); and the `AssemblyDrawer`, which draws the
placed parts as instances of their components and lists those placed in
the part itself.

## Solids and STL

`solids` lists the solids a part shows, wherever they are placed — its own
and those of the parts placed in it, however deep — each of its own part's
material, with their mass properties: what weighing a part, checking it for
interference, ordering it and simulating it read. `Stl` is the STL exporter
(see [geop-core-editor](../core/geop-core-editor.md#exporters)): the mesh the
editor draws, the parts placed included, each where it is; its options
choose the solids (`solids`), text or binary (`ascii`) and how finely
curved faces are meshed (`quality`).
