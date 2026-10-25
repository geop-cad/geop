# geop as an MCP server

`geop mcp` runs the geop editor — the same one the web app runs — as a
[Model Context Protocol](https://modelcontextprotocol.io) server over
stdio, so an agent can model parts: start or load a program, add and change
steps, see where they fail, find the names of faces and edges for the next
step's references, weigh and measure the part, and export or save it.

## Setup

Install the CLI, then register it with your client. For Claude Code:

```sh
cargo install --path cad/geop-cad-cli
claude mcp add geop -- geop mcp
```

Any other MCP client that starts a stdio server works the same way: the
command is `geop`, its argument `mcp`. The transport is MCP's stdio one:
JSON-RPC 2.0, one message per line, newline-delimited, no framing headers.

## Tools

| tool | what it does |
|---|---|
| `new_part` | An empty part. Every part has the datum `origin`, whose planes sketches are drawn on. |
| `load_program` | Replace the part with a `.geop` file (`path`; the other `.geop` files next to it are read too, for the parts it places) or a program's text (`text`). |
| `list_operations` | Every operation: its `kind` (what `add_step` takes), label, group, doc, the arguments a new step starts with (`default_args`) and the editor's form for it (`form`: each field's key, control, label, value; a reference field says which element kinds it accepts). |
| `add_step` | Append a step: `kind`, `args` (the step's saved JSON, as a `.geop` file holds it), optional `id`. Answers with every step's status. |
| `edit_step` | Set some of a step's arguments: the keys of `args` replace those of the step, the rest stay. The program is rebuilt from there. |
| `delete_step` | Remove a step. Steps that referred to what it built fail and say so. |
| `run` | Run the whole program: every step's summary, whether it ran and its error with the kernel's context, and the part's solids. |
| `list_elements` | The part's elements by their stable names: solids, faces (with their solid and the feature that made them), edges (with their faces), vertices (with their positions), sketches, datums, what else the part lists, placed parts. |
| `inspect` | `mass_properties` (volume, area, mass, centre of mass, inertia per solid and in all), `interference` (which solids overlap), or `measure` of one or two `elements` (`{kind, name}`). |
| `export` | Write the part to `path`: `stl`, `step`, `svg`/`dxf` (drawings, flat patterns), `csv` (bill of materials), `zip` (URDF); the format from the extension unless `format` says otherwise; `options` are the exporter's own. |
| `save` | Write the program to `path` as `.geop`, and edit it there from then on. |

Every tool is one transaction on the editor: no step is left open between
calls. A tool that fails answers an error result (`isError: true`) whose
text names what went wrong, with the kernel's context — the step, the
operation, the argument. A step that is added but does not build is not an
error of the tool: it is in the status every tool returns, under its id,
with the kernel's message.

## How a step refers to what came before

Steps refer to what earlier steps built by stable names, never by ids the
kernel assigns. The step `box` of an extrusion makes the solid
`extrude(box)`, its caps `extrude(box,start)` and `extrude(box,end)`, and
its side faces after the sketch's curves. `list_elements` gives these
names; a face is referred to as `{"type": "Face", "name": "extrude(box,end)"}`,
an edge as `{"type": "Edge", "name": ...}`, a sketch as
`{"target": "/:sketch:<id>"}`, and a plane of the origin as
`{"type": "Datum", "name": "origin", "component": {"plane": "z"}}`.

The quickest way to learn an operation's arguments is `list_operations`:
its `default_args` are exactly the JSON `add_step` takes, with the editor's
defaults filled in, and `form` says what each field is. The built-in
examples are another source: `geop examples --out-dir examples` writes every
one as a `.geop` file.

## A session

```jsonc
→ {"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"x","version":"0"}}}
← {"jsonrpc":"2.0","id":1,"result":{"protocolVersion":"2025-06-18","capabilities":{"tools":{}},"serverInfo":{"name":"geop","version":"0.1.0"},"instructions":"..."}}
→ {"jsonrpc":"2.0","method":"notifications/initialized"}
→ {"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"new_part","arguments":{}}}
→ {"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"add_step","arguments":{"id":"outline","kind":"add_sketch","args":{...}}}}
→ {"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"add_step","arguments":{"id":"box","kind":"extrude","args":{"sketch":{"target":"/:sketch:outline"},"extent":{"side1":{"blind":1.0},"symmetric":false,"reversed":false},"output":{"solid":{"mode":"new_body"}}}}}}
← {"jsonrpc":"2.0","id":4,"result":{"content":[{"type":"text","text":"{\"steps\":[...],\"failed\":0,\"solids\":[\"extrude(box)\"]}"}],"isError":false}}
→ {"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"inspect","arguments":{"inspector":"mass_properties"}}}
→ {"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"export","arguments":{"path":"box.step"}}}
```

`cad/geop-cad-cli/tests/mcp.rs` is this session in full: a block with a
hole drilled into it, modelled, inspected, exported as STL and STEP, saved,
changed and loaded again, through the server's pipes.

## If the kernel panics

A panic anywhere in the kernel is a bug (please report it, with the
program). The server catches it, answers the tool call with an error saying
so, and starts a fresh editor with the program as it was before the call,
so the session goes on.
