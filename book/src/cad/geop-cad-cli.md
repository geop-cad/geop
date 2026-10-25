# geop-cad-cli

> Brief overview only — full documentation is coming later.

The `geop` command-line tool: compiles a part program to STL — or any
format an exporter of the editor writes — from the terminal.

```sh
cargo install --path cad/geop-cad-cli
geop compile part.geop -o out/part.stl
```

## `geop compile`

Builds a program (the JSON the web editor saves, see
[geop-core-program](../core/geop-core-program.md#operations-and-programs)) and writes the part with the
exporter the editor offers for the output's extension: `.stl` (the mesh the
editor draws, see [geop-core-rasterize](../core/geop-core-rasterize.md)),
`.step`, `.svg`/`.dxf` drawings, `.zip` URDF robots, `.csv` bills of
materials.

| Option                  | Meaning                                                            |
| ----------------------- | ------------------------------------------------------------------ |
| `<program>`             | the program to build, e.g. `part.geop`                             |
| `-o`, `--output`        | where to write the part; defaults to the program's path with `.geop` replaced by `.stl` |
| `-O`, `--option K=V`    | an option of the exporter, the value JSON or else text; repeatable. The STL exporter takes `solids` (`-O 'solids=["extrude(hole)"]'`), `ascii` (`-O ascii=true`) and `quality` (`-O quality=8`, default 24, as in the web editor) |

## `geop examples`

Writes every built-in example program as `<name>.geop` together with
its `<name>.stl`, into `--out-dir` (default `examples`). `--only NAME`
restricts it to the named examples, and `--option` works as for `compile`.

## Tests

The command line's tests are of its own: its arguments, `serve`'s one
line per command, and `examples` over every example the registry offers.
What a part compiles to is tested in the crate of its exporter (STEP in
`geop-ops-step`, STL and URDF in `geop-ops-assembly`, a bill in `geop-ops-bom`, a
drawing in `geop-ops-drawing`, a flat pattern in `geop-ops-sheetmetal`, an
assembly's mesh in `geop-ops-assembly`).
