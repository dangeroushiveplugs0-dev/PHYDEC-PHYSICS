# PHYDEC-PHYSICS Roadmap

## M0 — Buildable foundation
- Portable C++20 core with no renderer dependency.
- CMake build and automated tests in GitHub Actions.
- Deterministic fixed-step update entry points.
- Keep public API small and testable.

## M1 — Secondary motion
- Bone-chain settings and animation-pose input.
- Per-point mass, damping, stiffness, gravity and external forces.
- Analytic sphere, capsule, and one-sided plane contacts are now applied during constraint iterations.
- Next: improve contact stability, add collision-response tuning, and validate with automated builds.
- Android NDK integration spike after the desktop core is green.

## M2 — Collision foundation
- Initial analytic contact queries: sphere, capsule, and one-sided plane.
- Mesh collision queries and proxy acceleration structures behind a common interface.
- Explicit tolerances, broad-phase/narrow-phase tests, and performance benchmarks.

## M3 — Deformation and authoring
- Soft-body prototype and attachment/weight maps.
- Runtime API plus minimal mobile-friendly authoring controls.
- Cache/replay hooks for reproducible simulations.

## M4 — Viscous liquid research prototype
- Evaluate particle-based simulation and smooth surface reconstruction.
- Profile on representative Android hardware before expanding scope.

## Engineering rules
- Renderer-independent core; platform and rendering adapters stay separate.
- Every feature lands with tests and documented limitations.
- Do not claim Android performance until measured on-device.
- KawaiiPhysics is a feature/architecture reference, not code to copy blindly; preserve attribution and license obligations if code is reused.
