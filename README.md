# PHYDEC-PHYSICS

A renderer-independent, C++20 physics engine in development, designed for reusable simulation APIs and an Android-friendly workflow.

## Current status

**Foundation + analytic collision prototype.** The repository contains a secondary-motion chain prototype, CMake build, automated tests, and GitHub Actions configuration. Sphere, capsule, and one-sided plane contact queries are implemented with edge-case tests. Build/test status must be verified in Actions before treating this milestone as green.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Direction

- Secondary motion for animated chains
- Analytic collision contacts, then detailed mesh queries and proxies
- Soft-body deformation and attachment weights
- Viscous-liquid research with smooth surface reconstruction
- Renderer-independent core with future Android integration

See [the roadmap](docs/ROADMAP.md) for milestones. KawaiiPhysics is a reference for feature expectations, not a claim of compatibility or a source-code port.
