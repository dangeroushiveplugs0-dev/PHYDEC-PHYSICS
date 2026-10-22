# Solver comparison experiment: PHYDEC vs jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics (BSD-3-Clause).

## Status

**Experiment started; no production solver decision has been made.** The C++20 prototype in
`engine/include/phydec/experimental/exact_spring.hpp` expresses the reference's documented
closed-form damped oscillator mathematically. It does not copy the JavaScript source. The
reference project is credited here; if source code is copied in a future change, preserve its
BSD-3-Clause notice and required disclaimers.

CI run [#102](https://github.com/dangeroushiveplugs0-dev/PHYDEC-PHYSICS/actions/runs/39777740656)
passed: the full core test suite, installable package, external consumer, existing collision
benchmarks, exact-spring tests, and solver benchmark smoke test all completed successfully.

## Category scorecard

| Category | PHYDEC baseline | Reference approach | Current verdict |
|---|---|---|---|
| Secondary-motion integration | Verlet-style update plus spring-to-anchor force and segment-length constraints | Exact closed-form damped oscillator for a constant-coefficient spring | **Testing**: exact prototype passes its initial analytic and timestep-partition checks; production comparison still needed |
| Gravity / parent acceleration | Gravity and external acceleration; parent motion represented through animated anchors | Gravity sag plus explicit parent-acceleration input | **Different API semantics**: test under matched forcing before deciding |
| Bone chains | Coupled point chain with distance constraints | Sequential independent oscillators; upstream acceleration propagates down chain | **Different behavior**: neither wins without use-case tests |
| Collision | Sphere/capsule/plane, detailed triangle meshes, BVH, closed-mesh solid option, translating and deforming mesh CCD | Plane/sphere/capsule point projection extension | **PHYDEC covers more required collision cases**; still validate robustness and performance |
| Weight-painted deformation | Not yet part of the core solver API | Per-region weight map and weighted bone offsets in the reference demos | **Reference has a feature PHYDEC has not implemented**; evaluate when deformation API is in scope |
| Numerical stability | Depends on Verlet step size, constraint iterations, and settings | Closed-form step for constant coefficients and constant forcing over each interval | **Early advantage for exact integration on timestep-partition invariance**; wider stress tests pending |
| Runtime cost | Current chain includes constraints and collision loops | Exact independent oscillator uses transcendental functions per step | See initial microbenchmark below; it is not an apples-to-apples speed claim |
| Complexity / integration | Current API already integrates collision and chain constraints | Compact oscillator model, separate extensions | Decide after matched behavior tests and integration-cost review |

## Initial test results

The exact spring prototype passes tests for:

- One 1-second step versus 100 steps of 0.01 seconds under constant forcing, across under-damped, near-critical, and over-damped cases.
- Long-running gravity sag converging to the analytic equilibrium.
- Zero timestep preserving the current state.
- The entire existing CI suite and installable external-consumer check.

These results validate the prototype's basic mathematics; they do **not** yet prove it is the best production solver for constrained bone chains.

### Initial CI throughput microbenchmark

Run #102 used a GitHub-hosted Ubuntu Release build, 20,000 states/chains, and 600 frames at 1/60 s:

| Workload | Measured time per update |
|---|---:|
| Exact independent oscillator | 5.27 ns |
| PHYDEC two-point constrained chain | 18.57 ns |

This is only an initial reference point. These workloads are **not equivalent**: PHYDEC's chain performs constraint work, while the oscillator integrates an independent state and uses a different model. The result does not justify claiming a 3.5x engine-wide speedup or changing the production solver. A fairer comparison must isolate equivalent equations and report setup cost separately from repeated updates.

## Next comparison passes

1. Add matched response scenarios: rest, gravity sag, sudden parent acceleration, damping sweep, variable frame deltas, and long-run error/drift.
2. Compare exact integration against a minimal Verlet-style integrator for the same unconstrainted oscillator equation, then separately compare chain constraints and collision integration.
3. Measure cost on identical compiler/build configuration; report setup cost and repeated-update cost separately.
4. Test collision approaches as a separate category, including fast motion and actual triangle meshes. Do not use the reference's simple point colliders as a substitute for PHYDEC's detailed-mesh collision requirement.
5. Decide category by category. Adopt only the parts that win on correctness/stability and justify integration cost. Do not add duplicate production solvers unless distinct semantics are genuinely useful.
