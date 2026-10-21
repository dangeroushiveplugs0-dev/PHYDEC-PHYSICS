# Testing PHYDEC-PHYSICS

This project is developed through GitHub, so you can verify changes from an Android phone without Android Studio.

## 1. Check the automated build

1. Open the [PHYDEC-PHYSICS Actions page](https://github.com/dangeroushiveplugs0-dev/PHYDEC-PHYSICS/actions).
2. Open the newest **C++ Core** workflow run.
3. Wait until it says **Success**. If it says **Failure**, open the run and inspect the first failed step and its log.
4. Confirm the run belongs to the latest commit on `main`. An older green run does not verify newer code.

The workflow configures and builds the C++20 core, runs CTest, installs the library, builds and runs an independent consumer, then builds and runs the BVH benchmark's correctness guard.

## 2. What the automated tests cover

- **Core motion:** deterministic stepping, anchors, and basic collision response.
- **Mesh collision:** closest-point queries, BVH vs. brute-force agreement, refitting, containment, and fast-crossing CCD.
- **Moving mesh CCD:** a translating mesh hit and miss.
- **Deforming mesh CCD:** a triangle moving through a stationary sphere and a separated miss.
- **Packaging:** a separate project uses `find_package(PHYDEC CONFIG REQUIRED)` and links `PHYDEC::Core`.
- **Performance guard:** the benchmark compares BVH and brute-force query results. Timings are informational; CI does not enforce a machine-dependent speedup threshold.

## 3. Run tests locally (desktop/Linux/macOS/Windows with CMake)

From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

To run the optional benchmark:

```sh
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DPHYDEC_BUILD_BENCHMARKS=ON -DPHYDEC_BUILD_TESTS=OFF
cmake --build build-bench --target phydec_mesh_bvh_benchmark --parallel 2
./build-bench/benchmarks/phydec_mesh_bvh_benchmark
```

## 4. What these tests do not prove yet

A green workflow proves the current automated cases compile and pass; it does not prove production readiness or Android performance. The deforming-mesh CCD path assumes previous/current meshes have identical triangle count and ordering and linearly interpolated vertex positions. Rotation during the sweep, topology changes, automatic watertightness/manifold validation, and on-device performance tests remain future work. Do not enable closed-solid containment on an open or non-manifold mesh.
