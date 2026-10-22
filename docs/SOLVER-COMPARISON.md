# Solver comparison experiment: PHYDEC vs jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics (BSD-3-Clause).

## Status

**Experiment ongoing; no production solver decision has been made.** The C++20 prototype in
`engine/include/phydec/experimental/exact_spring.hpp` expresses the reference's documented
closed-form damped oscillator mathematically. It does not copy the JavaScript source. The
reference project is credited here; if source code is copied in a future change, preserve its
BSD-3-Clause notice and required disclaimers.

CI run [#108](https://github.com/dangeroushiveplugs0-dev/PHYDEC-PHYSICS/actions/runs/39838201618)
passed after adding non-finite-settings guards and expanded stress tests. The full workflow
includes the core test suite, installed CMake package, external consumer, and optional benchmark
correctness/smoke checks.

## Category scorecard

| Category | PHYDEC baseline | Reference approach | Current verdict |
|---|---|---|---|
| Secondary-motion integration | Verlet-style update plus spring-to-anchor force and segment-length constraints | Exact closed-form damped oscillator for a constant-coefficient spring | **Trade-off found**: exact integration is timestep-partition invariant for constant forcing; the approximate integrator costs less per update |
| Gravity / parent acceleration | Gravity and external acceleration; parent motion represented through animated anchors | Gravity sag plus explicit parent-acceleration input | **Different API semantics**: constant parent acceleration is now covered by partition-invariance tests; abrupt changes still need a dedicated behavior comparison |
| Bone chains | Coupled point chain with distance constraints | Sequential independent oscillators; upstream acceleration propagates down chain | **Different behavior**: neither wins without use-case tests |
| Collision | Sphere/capsule/plane, detailed triangle meshes, BVH, closed-mesh solid option, translating and deforming mesh CCD | Plane/sphere/capsule point projection extension | **PHYDEC covers more required collision cases**; still validate robustness and performance |
| Weight-painted deformation | Not yet part of the core solver API | Per-region weight map and weighted bone offsets in the reference demos | **Reference has a feature PHYDEC has not implemented**; evaluate when deformation API is in scope |
| Numerical stability | Depends on Verlet step size, constraint iterations, and settings | Closed-form step for constant coefficients and constant forcing over each interval | **Early advantage for exact integration** on timestep-partition invariance; stress tests pass for selected damping and timestep ranges, but this is not a proof of universal stability |
| Runtime cost | Current chain includes constraints and collision loops | Exact independent oscillator uses transcendental functions per step | Matched-equation benchmark favors semi-implicit Euler throughput; see below |
| Complexity / integration | Current API already integrates collision and chain constraints | Compact oscillator model, separate extensions | Decide after matched behavior tests and integration-cost review |

## Matched-equation and stress tests

The test suite compares the exact solution with semi-implicit Euler integrating the **same**
spring equation, with the same initial state and constant forcing. The regression test verifies
that approximate integration error decreases as the timestep shrinks from 1/30 to 1/60 to
1/120 seconds.

The experimental solver also tests under-damped, near-critical, and over-damped cases; exact
one-step versus 100-step partitioning; constant parent acceleration; analytic gravity sag;
zero timestep; and rejection of non-finite settings without state mutation. A long-run sweep
runs 12,000 steps for each of six damping ratios at each of three timestep sizes and checks that
the state remains finite. These are bounded regression scenarios, not a proof that all parameter
combinations are stable or physically appropriate.

This is a fairer integration-method comparison than timing the reference oscillator against a
constrained chain. It still does not mean semi-implicit Euler is the exact algorithm used by
every part of PHYDEC's current chain solver.

## Initial CI throughput benchmark

Run #104 used a GitHub-hosted Ubuntu Release build, 20,000 states, and 600 frames at 1/60 s:

| Same spring equation, independent states | Measured time per update |
|---|---:|
| Exact closed-form oscillator | 4.16 ns |
| Semi-implicit Euler approximation | 2.67 ns |

On that runner, the exact solution cost about 1.56 times as much per update (roughly 56% more).
That cost buys a useful property: under constant coefficients and forcing, one long step and
many smaller steps give the same analytical result apart from floating-point rounding. The
approximate method's error decreased at finer timesteps in our regression test.

For context only, the existing PHYDEC two-point constrained chain measured 12.53 ns/update in
the same run. It performs different work, so this is **not** a direct speed ratio and must not be
used to claim the exact solver makes the full engine faster or slower. Timings are informational,
not CI thresholds.

## Next comparison passes

1. Compare a sudden parent-acceleration change and variable frame-delta sequences with an explicit expected response; constant-forcing partition invariance alone does not cover changing input.
2. Compare bone-chain semantics: independent spring offsets versus distance-constrained points, including upstream-to-downstream acceleration propagation.
3. Test collision approaches separately, including fast motion and actual triangle meshes. Do not use simple point colliders as a substitute for PHYDEC's detailed-mesh collision requirement.
4. Evaluate weight-painted deformation as a feature-coverage category when the core deformation API is in scope.
5. Decide category by category. Adopt only the parts that win on correctness/stability and justify integration cost. Do not add duplicate production solvers unless distinct semantics are useful.
