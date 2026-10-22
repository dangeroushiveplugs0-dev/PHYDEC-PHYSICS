# Solver comparison experiment: PHYDEC vs jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics (BSD-3-Clause).

## Status

**Experiment started; no production solver decision has been made.** The C++20 prototype in
`engine/include/phydec/experimental/exact_spring.hpp` expresses the reference's documented
closed-form damped oscillator mathematically. It does not copy the JavaScript source. The
reference project is credited here; if source code is copied in a future change, preserve its
BSD-3-Clause notice and required disclaimers.

## Category scorecard

| Category | PHYDEC baseline | Reference approach | Current verdict |
|---|---|---|---|
| Secondary-motion integration | Verlet-style update plus spring-to-anchor force and segment-length constraints | Exact closed-form damped oscillator for a constant-coefficient spring | **Testing**: compare stability, timestep sensitivity, behavior, and cost |
| Gravity / parent acceleration | Gravity and external acceleration; parent motion represented through animated anchors | Gravity sag plus explicit parent-acceleration input | **Different API semantics**: test under matched forcing before deciding |
| Bone chains | Coupled point chain with distance constraints | Sequential independent oscillators; upstream acceleration propagates down chain | **Different behavior**: neither wins without use-case tests |
| Collision | Sphere/capsule/plane, detailed triangle meshes, BVH, closed-mesh solid option, translating and deforming mesh CCD | Plane/sphere/capsule point projection extension | **PHYDEC covers more required collision cases**; still validate robustness and performance |
| Weight-painted deformation | Not yet part of the core solver API | Per-region weight map and weighted bone offsets in the reference demos | **Reference has a feature PHYDEC has not implemented**; evaluate when deformation API is in scope |
| Numerical stability | Depends on Verlet step size, constraint iterations, and settings | Closed-form step for constant coefficients and constant forcing over each interval | **Prototype tests added**; broader comparison pending |
| Runtime cost | Current chain includes constraints and collision loops | Exact independent oscillator uses transcendental functions per step | Benchmark is informational; workloads are not identical |
| Complexity / integration | Current API already integrates collision and chain constraints | Compact oscillator model, separate extensions | Decide after measured tradeoffs and feature requirements |

## Test coverage added

- Compare one 1-second step with 100 steps of 0.01 seconds under constant forcing.
- Exercise under-damped, critical-nearby, and over-damped regimes.
- Check long-running gravity sag against its analytic equilibrium.
- Check invalid zero timestep does not change state.
- Add a Release-oriented throughput microbenchmark. It explicitly labels the workloads as non-equivalent and is not a CI timing threshold.

## Next comparison passes

1. Run CI and inspect the first prototype results.
2. Add matched response scenarios: rest, gravity sag, sudden parent acceleration, damping sweep, variable frame deltas, and long-run error/drift.
3. Measure cost on identical compiler/build configuration; report setup cost and repeated-update cost separately.
4. Test collision approaches as a separate category, including fast motion and actual triangle meshes. Do not use the reference's simple point colliders as a substitute for PHYDEC's detailed-mesh collision requirement.
5. Decide category by category. Adopt only the parts that win on correctness/stability and justify integration cost. Do not add duplicate production solvers unless distinct semantics are genuinely useful.
