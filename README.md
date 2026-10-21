# PHYDEC-PHYSICS

A renderer-independent, C++20 physics engine in development, designed for reusable simulation APIs and an Android-friendly workflow.

## Current status

**Secondary-motion and analytic-collision prototype.** The core has a bone-chain solver plus sphere, capsule, and one-sided plane contacts. The solver now applies configured contacts during its constraint iterations, with regression tests for basic contact cases. GitHub Actions build/test status still needs confirmation before treating the code as verified.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Direction

- Secondary motion for animated chains
- Analytic collision response, then detailed mesh queries and proxies
- Soft-body deformation and attachment weights
- Viscous-liquid research with smooth surface reconstruction
- Renderer-independent core with future Android integration

See [the roadmap](docs/ROADMAP.md) for milestones. KawaiiPhysics is a reference for feature expectations, not a claim of compatibility or a source-code port.
