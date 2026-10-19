# Contributing

Issues and pull requests are welcome.

## Setup

Install Rust with [rustup](https://rustup.rs) and Trackmania 2020, then:

```sh
cargo run --release -p gearbox-cli -- setup
```

## Before you push

```sh
cargo fmt --all
cargo clippy --release --workspace --all-targets -- -D warnings
cargo test --release --workspace
cargo run --release -p gearbox-cli -- golden check
```

CI runs the first three on macOS, Linux and Windows, plus `cargo deny`. It
can't run `golden check`: that needs the game's packs, so run it yourself
when you touch the engine or the world builder. Every case must stay
`EXACT`.

## Rules

- The engine must stay exact. Don't reorder float operations, change
  iteration order or swap in "equivalent" math: results must match bit for
  bit.
- Names explain the code; the codebase carries no comments.
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org):
  `feat:`, `fix:`, `perf:`, `refactor:`, `chore:`, `ci:`. Release notes are
  generated from them.

## Releases

Maintainers release from Actions → **Release** → Run workflow, with a
version like `0.2.0` (or `0.2.0-rc.1` for a pre-release). The workflow tests,
tags, builds every platform and publishes the release.
