# PHYDEC-PHYSICS

A renderer-independent, C++20 physics engine in development, designed for reusable simulation APIs and an Android-friendly workflow.

## Current status

**Secondary-motion and collision prototype.** The solver supports sphere, capsule, one-sided plane, and triangle-mesh surface contacts. Mesh collision uses the actual triangles supplied by the caller rather than approximating the mesh with a capsule or box. Mesh queries can use an explicitly built median-split BVH to prune triangles during nearest-surface queries. After vertex deformation, `refit_triangle_mesh_bvh(mesh)` updates bounds without rebuilding the split structure when triangle count/order are unchanged; rebuild with `build_triangle_mesh_bvh(mesh)` after topology or triangle-order changes. If refitting fails, stale acceleration data is cleared and queries fall back to the brute-force reference path. Mesh collision also has a conservative-advancement swept-sphere query integrated into secondary motion to reduce tunneling through thin static triangle surfaces. An odd/even ray query can classify points against caller-supplied closed, watertight meshes; it does not validate mesh closure and is not yet integrated into solver response. Moving/deforming-mesh continuous collision remains unimplemented. GitHub Actions must be green on the latest commit before this code is considered verified.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Direction

- Secondary motion for animated chains
- Sphere, capsule, plane, and true triangle-mesh surface collision
- Optional BVH acceleration for detailed triangle-mesh queries
- Soft-body deformation and attachment weights
- Viscous-liquid research with smooth surface reconstruction
- Renderer-independent core with future Android integration

See [the roadmap](docs/ROADMAP.md) for milestones. KawaiiPhysics is a reference for feature expectations, not a claim of compatibility or a source-code port.
