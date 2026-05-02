# Contributing

## Cloning

This repo uses git submodules. Always clone with `--recursive`, or initialise after the fact:

```bash
git clone --recursive https://github.com/youtalk/kachaka_autoware_bridge.git
# or, after a non-recursive clone:
git submodule update --init --recursive
```

If you prefer SSH for the submodule, add to `~/.gitconfig`:

```
[url "git@github.com:"]
    insteadOf = https://github.com/
```

## Local checks before pushing

Install [pre-commit](https://pre-commit.com/) once and run it on every commit:

```bash
pip install pre-commit  # or: pipx install pre-commit
pre-commit install
pre-commit run --all-files
```

CI runs the same hooks. If pre-commit is green locally, the lint job will be green on GitHub.

## Build and test

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

The CI workflow runs the same three commands on Ubuntu 24.04 with ROS 2 Jazzy.

## Bumping the kachaka-api submodule

```bash
cd kachaka-api
git fetch --tags
git checkout <new-tag>
cd ..
git add kachaka-api
git commit -m "chore(submodule): bump kachaka-api to <new-tag>"
```

## Code style

- C++ formatted by `clang-format` using the root `.clang-format`.
- CMake formatted by `cmake-format` using the root `.cmake-format.yaml`.
- Apache-2.0 copyright header on new source files (`Copyright YYYY Yutaka Kondo`).

## Reporting issues

Open an issue on this repo with a minimal reproduction. For bugs that are actually upstream `kachaka-api` issues, file them at <https://github.com/pf-robotics/kachaka-api/issues>.
