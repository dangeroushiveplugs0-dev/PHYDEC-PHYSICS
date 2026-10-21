# PHYDEC-PHYSICS

A renderer-independent, C++20 physics engine in development, designed for reusable simulation APIs and an Android-friendly workflow.

## Current status

**Secondary-motion and collision prototype.** The solver supports sphere, capsule, one-sided plane, and triangle-mesh surface contacts. Mesh collision uses the actual triangles supplied by the caller rather than approximating the mesh with a capsule or box. The current mesh query is brute-force and resolves nearby surface contacts; it does not yet provide watertight solid containment, continuous collision detection, or a BVH for large meshes. GitHub Actions must be green on the latest commit before this code is considered verified.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Direction

- Secondary motion for animated chains
- Sphere, capsule, plane, and true triangle-mesh surface collision
- Future BVH acceleration and detailed mesh queries for large meshes
- Soft-body deformation and attachment weights
- Viscous-liquid research with smooth surface reconstruction
- Renderer-independent core with future Android integration

See [the roadmap](docs/ROADMAP.md) for milestones. KawaiiPhysics is a reference for feature expectations, not a claim of compatibility or a source-code port.
