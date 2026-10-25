# Contributing to geop

Thanks for helping. Three documents say how the code is written; read the
one you need before changing anything:

- [AGENTS.md](AGENTS.md): the rules of the kernel (intervals, no epsilons,
  where things go) and the bugs that made each rule. They bind people and
  agents alike.
- [DEVELOPERS.md](DEVELOPERS.md): the crate table, the scripts and how the
  web app, the VS Code extension and the CLI are built.
- [ROADMAP.md](ROADMAP.md): what is done and what is next.

## Before you open a pull request

```sh
scripts/check-layering.sh                              # the tiers stay apart
cargo fmt --check
timeout 1800 prlimit --as=16000000000 cargo clippy --workspace --all-targets -- -D warnings
scripts/test-affected.sh origin/main                   # the tests your change can affect
cd web && npm run build && npm test                    # if you touched the web app or the protocol
```

Cap every run in memory and time, as above: a search that runs away does not
stop on its own.

Every bug fix starts with a reproducer in the `regression_tests.rs` of the
crate whose operation failed, red before the fix and green after; never a
`.geop` file as a fixture. Anything a user reaches through the editor also
gets a test in that crate's `editor_tests.rs`, driving it the way the front
end does.

## Where to talk

- Questions, ideas, a part that does not build: the
  [Discord](https://discord.gg/btRDgrcwJ5).
- Bugs: a GitHub issue, with the `.geop` program (or the steps to make it)
  and the kernel's error text.

By submitting a contribution you license it under MIT OR Apache-2.0, as the
rest of geop is, with no further terms.
