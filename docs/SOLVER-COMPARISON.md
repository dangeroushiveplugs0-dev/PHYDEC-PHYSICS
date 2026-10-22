# Solver comparison experiment: PHYDEC vs jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics (BSD-3-Clause).

## Status

**Experiment started; no production solver decision has been made.** The C++20 prototype in
`engine/include/phydec/experimental/exact_spring.hpp` expresses the reference's documented
closed-form damped oscillator mathematically. It does not copy the JavaScript source. The
reference project is credited here; if source code is copied in a future change, preserve its
BSD-3-Clause notice and required disclaimers.

CI run [#104](https://github.com/dangeroushiveplugs0-dev/PHYDEC-PHYSICS/actions/runs/39778105835)
passed the core tests, installable package, external consumer, collision benchmark guards,
exact-spring experiments, and solver benchmark smoke test.

## Category scorecard

| Category | PHYDEC baseline | Reference approach | Current verdict |
|---|---|---|---|
| Secondary-motion integration | Verlet-style update plus spring-to-anchor force and segment-length constraints | Exact closed-form damped oscillator for a constant-coefficient spring | **Trade-off found**: exact integration is timestep-partition invariant for constant forcing; the approximate integrator costs less per update |
| Gravity / parent acceleration | Gravity and external acceleration; parent motion represented through animated anchors | Gravity sag plus explicit parent-acceleration input | **Different API semantics**: test under matched forcing before deciding |
| Bone chains | Coupled point chain with distance constraints | Sequential independent oscillators; upstream acceleration propagates down chain | **Different behavior**: neither wins without use-case tests |
| Collision | Sphere/capsule/plane, detailed triangle meshes, BVH, closed-mesh solid option, translating and deforming mesh CCD | Plane/sphere/capsule point projection extension | **PHYDEC covers more required collision cases**; still validate robustness and performance |
| Weight-painted deformation | Not yet part of the core solver API | Per-region weight map and weighted bone offsets in the reference demos | **Reference has a feature PHYDEC has not implemented**; evaluate when deformation API is in scope |
| Numerical stability | Depends on Verlet step size, constraint iterations, and settings | Closed-form step for constant coefficients and constant forcing over each interval | **Early advantage for exact integration** on timestep-partition invariance; broader stress tests pending |
| Runtime cost | Current chain includes constraints and collision loops | Exact independent oscillator uses transcendental functions per step | Matched-equation benchmark favors semi-implicit Euler throughput; see below |
| Complexity / integration | Current API already integrates collision and chain constraints | Compact oscillator model, separate extensions | Decide after matched behavior tests and integration-cost review |

## Matched-equation test

The test suite compares the exact solution with semi-implicit Euler integrating the **same**
spring equation, with the same initial state and constant forcing. The regression test verifies
that approximate integration error decreases as the timestep shrinks from 1/30 to 1/60 to
1/120 seconds. It also checks under-damped, near-critical, and over-damped cases, exact
one-step versus 100-step partitioning, analytic gravity sag, and invalid zero timestep.

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

1. Test sudden parent acceleration, damping sweeps, variable frame deltas, and long-run drift.
2. Compare bone-chain semantics: independent spring offsets versus distance-constrained points.
3. Test collision approaches separately, including fast motion and actual triangle meshes. Do not
   use simple point colliders as a substitute for PHYDEC's detailed-mesh collision requirement.
4. Evaluate weight-painted deformation as a feature-coverage category when the core deformation
   API is in scope.
5. Decide category by category. Adopt only the parts that win on correctness/stability and justify
   integration cost. Do not add duplicate production solvers unless distinct semantics are useful.
