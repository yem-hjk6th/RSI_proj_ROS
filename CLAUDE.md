# Rules for agents working in this repository

## Scope
- Implement exactly the GitHub issue you were given. Read `docs/DESIGN.md` first.
- Work on a feature branch and open a PR to `main` that names the issue.
- Do not push to `main`.
- Touch only the packages and files your issue lists. If you need a change elsewhere, say so in the PR description instead of making it.
  - This matters most for `src/krc_interfaces`.

## Language and identity
- Everything committed is in English: code, comments, docs, commit messages.
- Commit identity: `git config user.name yem-hjk6th` and `git config user.email 183082151+yem-hjk6th@users.noreply.github.com`.
- Add no `Co-Authored-By` or other attribution trailers.
- No personal names, e-mail addresses, IP addresses, host names or absolute paths of anyone's machine.
- Network values come from parameters, with placeholders in the examples.

## Platform
- ROS 2 Jazzy on Ubuntu 24.04, Python ≥ 3.12, `rclpy` and `ament_python`.
- Message packages use `ament_cmake`.
- Core logic is plain Python with no ROS imports, tested with `pytest`.
- ROS nodes are thin wrappers around that core.
- CI (`.github/workflows/ci.yml`) must pass. If ROS 2 cannot be installed in your session, run the pure-Python tests locally and rely on CI for the colcon build.

## Safety invariants (never weaken these; tests must cover them)
1. RSI replies always carry zero position corrections (RKorr = 0).
2. The commanded override is always clamped to `[ov_min, ov_max]` with the configured speed floor.
3. If the controller's heartbeat is stale, the bridge ramps linearly to `ov_default` over `fallback_ramp_s`.
4. The 4 ms reply loop never blocks on ROS callbacks, logging or other processes. It runs in its own process or real-time thread.
5. No code in this phase streams joint or Cartesian motion targets.
6. Controller-side files (KRL, sps.sub, EKI/RSI XML) are templates marked `NOT RUN` in their header until tested on hardware.

## Tests
- **Golden tests:** run the corresponding module in `legacy/` on the inputs in `data/`, store the outputs as fixtures, and assert that the port reproduces them.
  - Exact equality for integer outputs such as override values.
  - A stated tolerance for floats.
- Keep fixtures small. Do not commit generated files over 5 MB.
