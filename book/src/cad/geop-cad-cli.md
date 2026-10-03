# geop-cad-cli

> Brief overview only — full documentation is coming later.

The `geop` command-line tool: compiles a part program to STL from the
terminal.

```sh
cargo install --path cad/geop-cad-cli
geop compile part.geop -o out/part.stl
```

## `geop compile`

Builds a program (the JSON the web editor saves, see
[geop-ops](../ops/geop-ops.md#programs)) and writes the part as an STL
mesh. The mesh is the one the editor draws (see
[geop-ops-rasterize](../ops/geop-ops-rasterize.md)), so a compiled file looks
exactly like the part on screen.

| Option              | Meaning                                                            |
| ------------------- | ------------------------------------------------------------------ |
| `<program>`         | the program to build, e.g. `part.geop`                     |
| `-o`, `--output`    | where to write the mesh; defaults to the program's path with `.geop` replaced by `.stl` |
| `-s`, `--solid NAME`| only this solid, by name (e.g. `extrude(hole)`); repeatable; all solids by default |
| `--ascii`           | write ASCII STL instead of binary                                  |
| `-q`, `--quality`   | how finely curved faces are meshed (default 24, as in the web editor); flat faces are exact regardless |

## `geop examples`

Writes every built-in example program as `<name>.geop` together with
its `<name>.stl`, into `--out-dir` (default `examples`). `--only NAME`
restricts it to the named examples, and `--ascii` and `--quality` work as for
`compile`.
