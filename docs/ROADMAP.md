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
- Added conservative-advancement swept-sphere collision against triangle surfaces in secondary motion; validate limits and performance with more cases. Add watertight closed-mesh containment and moving/deforming mesh CCD/update strategies.
- Establish broad-phase/narrow-phase interfaces and explicit tolerances. An opt-in nearest-point benchmark now exists; extend it to BVH refit and swept collision workloads.

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
- [ ] Integrate containment response into particle/secondary-motion collision and add accelerated ray traversal plus moving-mesh CCD.
