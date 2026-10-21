# PHYDEC-PHYSICS Roadmap

## M0 — Buildable foundation
- Portable C++20 core with no renderer dependency.
- CMake build and automated tests in GitHub Actions.
- Deterministic fixed-step update entry points.
- Keep public API small and testable.

## M1 — Secondary motion
- Bone-chain settings and animation-pose input.
- Per-point mass, damping, stiffness, gravity and external forces.
- Sphere, capsule, and one-sided plane contacts applied during constraint iterations.
- Triangle-mesh surface contacts integrated into the same solver.
- Next: improve contact stability, add collision-response tuning, and validate with automated builds.
- Android NDK integration spike after the desktop core is green.

## M2 — Collision foundation
- Implemented first narrow-phase set: sphere, capsule, one-sided plane, and triangle-mesh surface contacts.
- Added optional median-split BVH acceleration with nearest-point AABB pruning. Added BVH refitting for vertex deformation when triangle count/order remain unchanged; rebuild when topology or triangle ordering changes. Brute-force fallback remains available when no valid BVH exists.
- Added conservative-advancement swept-sphere collision against static triangle surfaces in secondary motion, plus relative-motion CCD for rigidly translating triangle meshes using the existing BVH. Rotation and topology changes during a sweep remain open work. Deforming-mesh CCD now supports linearly interpolated vertex motion with stable triangle count/order and conservative per-node BVH displacement bounds. Add robust watertight-mesh validation and broader deformation/CCD performance tests.
- Establish broad-phase/narrow-phase interfaces and explicit tolerances. Opt-in nearest-point and deforming-CCD benchmarks report timing; deforming CCD also asserts deterministic candidate reduction against brute force.

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


## Collision response follow-up

- [x] Add configurable friction and restitution to secondary-motion contact response.
- [x] Add swept mesh collision query and fast-crossing regression tests.
- [x] Add BVH acceleration for large triangle meshes.
- [x] Add BVH refit support and regression tests for vertex-deformed meshes.
- [x] Add an odd/even ray-based point-in-closed-mesh query and tetrahedron regression tests. Caller must supply a closed, watertight mesh; boundary points are treated as outside and manifoldness is not validated.
- [x] Integrate rigidly translating-mesh CCD into secondary motion.
- [x] Add deforming-mesh swept-sphere query and integrate it into secondary motion for stable topology/linear vertex motion.
- [x] Add deforming-CCD candidate counters, brute-force equivalence tests, and a candidate-reduction benchmark.
- [ ] Add rotation-aware CCD, mesh topology validation, and broader refit/swept-collision profiling.


## Release readiness

### v0.1.0 — Core preview

Publish when all of these are true:
- [ ] The latest `main` commit passes the full C++ Core workflow.
- [ ] Public headers, installed CMake package, and independent consumer are verified.
- [ ] README clearly identifies the supported features and unsupported cases.
- [ ] A version tag and release notes describe the API as experimental and explain lifetime/topology assumptions.
- [ ] At least one example demonstrates secondary motion and mesh collision from a consuming project.

This preview is a reusable collision/secondary-motion library, not the full planned engine. Soft-body simulation, attachment-weight authoring, and viscous-liquid simulation are not prerequisites for v0.1.0, but are prerequisites for claiming those features exist.

### v1.0.0 — Stable core

Before calling the engine stable, require documented API/ABI policy, stronger invalid-geometry and topology handling, broader CCD including rotational motion, deterministic regression coverage for edge cases, performance baselines, and a clear platform support matrix. Soft-body and liquid milestones remain separate feature releases unless they are explicitly added to the v1.0 scope.
