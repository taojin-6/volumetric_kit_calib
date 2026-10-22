# Contributing

[AGENTS.md](AGENTS.md) is the canonical shared guide for contributors, Codex,
and Claude Code. It owns the working rules, validation commands, and task map;
`CLAUDE.md` imports it. [DECISIONS.md](DECISIONS.md) keeps the recorded choices.

## One-time setup

Install [`pre-commit`](https://pre-commit.com) (`pipx install pre-commit`, or
`pip install --user pre-commit`), then, from a fresh clone, install the git hook:

```sh
pre-commit install
```

This wires the hooks in [`.pre-commit-config.yaml`](.pre-commit-config.yaml)
(clang-format, cmake-format, trailing-whitespace, end-of-file-fixer, and a
check of CI's Homebrew key wiring) into `.git/hooks/`. They run on staged files
at `git commit` and abort the commit if anything is reformatted or a check
fails, so style is fixed locally instead of in CI. The hook lives in `.git/`
and is **not** tracked, so each clone must run this once.

## Build and test

Out-of-source CMake build (needs GLM and OpenCV 5 — see the README
prerequisites):

```sh
calib_root="$(git rev-parse --show-toplevel)"
cmake -S "$calib_root" -B "$calib_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$calib_root/build" --parallel
```

Then run the tests as [AGENTS.md](AGENTS.md#build-and-validation) does.

The `core` tier is pure CPU, so the suite runs on any machine — no GPU or device
required.

## Working guidance

Follow [Working with Git](AGENTS.md#working-with-git) for worktrees and PRs.
`DESIGN.md` remains gitignored; from a standard `.worktrees/<branch>/`, read
the primary checkout's `../../DESIGN.md` only when the task needs it. Do not
publish it with a documentation change.

## Formatting and CI

- clang-format is pinned (see `.pre-commit-config.yaml`) so local and CI
  formatting are byte-identical; don't reformat with a different version.
- Mark deferred/future work inline with a greppable `// TODO:` (or `# TODO:` in
  CMake), rather than tracking it only in prose or commits.

## Commits

Follow Conventional Commits, e.g. `feat(core): …`, `build: …`,
`refactor(core): …`.
