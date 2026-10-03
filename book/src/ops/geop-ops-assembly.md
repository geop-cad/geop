# geop-ops-assembly

> Brief overview only — full documentation is coming later.

Placing parts as an operation: `AddPart` places the part another program
file builds, named by the step's id, and adds mates between its entities
and those of what is already there.

## Arguments, and where the part goes

```json
{ "steps": [
    { "id": "pin", "operation": "add_part",
      "args": { "file": "pin.geop", "fixed": false, "flexible": false,
                "mates": {
                  "m1": { "type": "concentric",
                          "entities": [ { "type": "Face", "name": "pin/extrude(pin,pin_sketch,c1)" },
                                        { "type": "Face", "name": "plate/extrude(hole,hole_sketch,c1)" } ] } } } } ],
  "state": {
    "pin.pose": { "position": [1.0, 1.0, 0.5], "rotation": [1.0, 0.0, 0.0, 0.0] } } }
```

`file` is relative to the program's own file. Entities of a placed part are
named behind its instance's name (`pin/...`); entities of the part itself
are ground. Where the part goes is not an argument but the program's
parameter `pin.pose` — a position and a rotation quaternion — part of its state,
solved for (see [geop-ops](./geop-ops.md)). The step places the part there
and adds its mates; it solves nothing. So a later step's mates that move it
move it for every step.

## Parameters

`parameters` gives the placed part's own parameters (see
[geop-ops](./geop-ops.md#parameters)) other values, by name: a number, a
table's row, the colour. The part is built with them — a file is built once
per set of values it is placed with — and its dialog offers each as what it
is: the colour to pick, a number on a slider over its range, a table's row
from a list to search, each showing the value the part is built with here.

## Rigid and flexible

Placed rigid, a part moves as one body, its own parts where its file puts
them. Placed `flexible`, every parameter of the program placed becomes one
of this program's, named behind the step's id — `hinge/pin.pose` — and the
part is built with those: its parts are this program's to move, and its
mates are solved with this program's. The file is rebuilt incrementally,
only from the first step that reads a parameter that changed — for a file
that only places parts, only its placements.

## Editing

- The placement fields show a position and angles about x, y and z, and
  set the parameter.
- Adding a mate arms its entities field. When a mate is added the editor
  solves the program, moving the step's own part first, the others only if
  that is not enough.
- Picks test against the part as the step builds it (`PICKS_BUILT`), so the
  placed part's own entities can be picked.
- Dragging the placed part asks for the point grabbed to be pulled towards
  the pointer, in the plane through it facing the eye (`Form::drags`). The
  editor solves the whole program with that pull, every part that is not
  fixed free to give way: a linkage follows the link dragged. A fixed part
  goes where it is dragged.
