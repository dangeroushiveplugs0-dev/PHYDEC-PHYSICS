# PHYDEC-PHYSICS

A renderer-independent, C++20 physics engine in development, designed for reusable simulation APIs and an Android-friendly workflow.

## Current status

**Secondary-motion and collision prototype.** The solver supports sphere, capsule, one-sided plane, and triangle-mesh surface contacts. Mesh collision uses the actual triangles supplied by the caller rather than approximating the mesh with a capsule or box. Mesh queries can use an explicitly built median-split BVH to prune triangles during nearest-surface queries. After vertex deformation, `refit_triangle_mesh_bvh(mesh)` updates bounds without rebuilding the split structure when triangle count/order are unchanged; rebuild with `build_triangle_mesh_bvh(mesh)` after topology or triangle-order changes. If refitting fails, stale acceleration data is cleared and queries fall back to the brute-force reference path. Mesh collision also has a conservative-advancement swept-sphere query integrated into secondary motion to reduce tunneling through thin static triangle surfaces. `sweep_sphere_against_translating_mesh(...)` additionally handles a collider undergoing rigid translation by solving in relative motion and reusing its BVH; collider rotation and per-vertex deformation during a sweep are not supported yet. Secondary-motion settings expose translating colliders through non-owning mesh pointers to avoid duplicating large triangle/BVH arrays; the referenced mesh must outlive the settings and simulation steps. Closed-mesh containment can be enabled per collider with `closed_mesh_solid = true`; this is opt-in and assumes a closed, non-self-intersecting mesh, which the engine does not validate. Closed-solid response is deliberately opt-in and requires a closed, watertight, non-self-intersecting mesh; topology validation is not implemented. Deforming-mesh CCD is available for linearly interpolated vertex motion when previous/current meshes have identical triangle count and ordering; the current mesh BVH is expanded conservatively for candidate selection. Rotation and topology changes during a sweep remain unsupported. GitHub Actions must be green on the latest commit before a change is considered verified.

## Integrate into another C++ project

PHYDEC is intended to be reusable as a core engine/library, not tied to one app. It has no renderer dependency and exposes a CMake target named `PHYDEC::Core` plus a convenience header:

```cpp
#include <phydec/phydec.hpp>
```

To build and install the library from this repository:

```sh
cmake -S . -B build -DPHYDEC_BUILD_TESTS=OFF -DPHYDEC_INSTALL=ON
cmake --build build --parallel
cmake --install build --prefix ./install
```

A consuming CMake project can then use the installed package with `find_package(PHYDEC CONFIG REQUIRED)` and link `PHYDEC::Core` (set `CMAKE_PREFIX_PATH` to the install prefix). The install/export workflow is verified in CI by installing PHYDEC and compiling/running a separate consumer project that uses `find_package(PHYDEC CONFIG REQUIRED)`. Android NDK packaging and ABI-specific artifacts remain future work.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

See [the testing guide](docs/TESTING.md) for how to verify changes from GitHub Actions, including collision regressions and performance checks.

## Optional performance benchmark

The deterministic nearest-point and deforming-mesh CCD benchmarks compare accelerated results with brute-force references. Deforming CCD also reports BVH candidate counts and checks candidate reduction. Timings are informational and must be compared on the same machine/build; CI does not enforce a machine-dependent speed threshold.

```sh
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DPHYDEC_BUILD_BENCHMARKS=ON -DPHYDEC_BUILD_TESTS=OFF
cmake --build build-bench --target phydec_mesh_bvh_benchmark phydec_deforming_ccd_benchmark --parallel
./build-bench/benchmarks/phydec_mesh_bvh_benchmark
./build-bench/benchmarks/phydec_deforming_ccd_benchmark
```

## Direction

- Secondary motion for animated chains
- Sphere, capsule, plane, and true triangle-mesh surface collision
- Optional BVH acceleration for detailed triangle-mesh queries
- Soft-body deformation and attachment weights
- Viscous-liquid research with smooth surface reconstruction
- Renderer-independent core with future Android integration

See [the roadmap](docs/ROADMAP.md) for milestones. KawaiiPhysics is a reference for feature expectations, not a claim of compatibility or a source-code port.
