# Contributing

## One-time setup

Install [`pre-commit`](https://pre-commit.com) (`pipx install pre-commit`, or
`pip install --user pre-commit`), then, from a fresh clone, install the git hook:

```sh
pre-commit install
```

This wires the hooks in [`.pre-commit-config.yaml`](.pre-commit-config.yaml)
(clang-format, cmake-format, trailing-whitespace, end-of-file-fixer) into
`.git/hooks/`. They run on staged files at `git commit` and abort the commit if
anything is reformatted, so style is fixed locally instead of in CI. The hook
lives in `.git/` and is **not** tracked, so each clone must run this once.

## Build and test

Out-of-source CMake build (needs Eigen — see the README prerequisites):

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

The `core` tier is pure CPU, so the suite runs on any machine — no GPU or device
required.

## Work in a git worktree

Implement each change in a dedicated **git worktree**, not by `git checkout`-ing
a branch in the primary checkout. Create it locally under `.worktrees/` (e.g.
`git worktree add .worktrees/<branch> -b <branch>`) — never as a sibling in the
parent folder — and remove it with `git worktree remove` once the PR merges.

## Formatting and CI

- clang-format is pinned (see `.pre-commit-config.yaml`) so local and CI
  formatting are byte-identical; don't reformat with a different version.
- Mark deferred/future work inline with a greppable `// TODO:` (or `# TODO:` in
  CMake), rather than tracking it only in prose or commits.

## Commits

Follow Conventional Commits, e.g. `feat(core): …`, `build: …`,
`refactor(core): …`.
