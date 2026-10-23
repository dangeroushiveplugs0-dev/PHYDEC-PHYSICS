# Solver comparison experiment: PHYDEC vs jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics (BSD-3-Clause).

## Status

**Experimental exact-spring bone wrapper and isolated hybrid chain prototype added; the existing production chain solver remains unchanged.** The independent wrapper is not collision-aware or length-constrained. The hybrid prototype adds positional segment-length projection, but is still experimental and does not yet handle collision, rotation-aware local frames, or per-bone settings. The C++20 prototype in
`engine/include/phydec/experimental/exact_spring.hpp` expresses the reference's documented
closed-form damped oscillator mathematically. It does not copy the JavaScript source. The
reference project is credited here; if source code is copied in a future change, preserve its
BSD-3-Clause notice and required disclaimers.

CI run [#108](https://github.com/dangeroushiveplugs0-dev/PHYDEC-PHYSICS/actions/runs/39838201618)
passed after adding non-finite-settings guards and expanded stress tests. The variable-delta, parent-acceleration, and linked-chain fixture CI runs have passed. Isolated tests cover the bone wrapper and a new hybrid chain fixture that runs 1,200 animated frames, checks finite output, pinned root, segment-length error, measurable secondary motion, invalid-input handling, and reset behavior. CI has not yet verified the hybrid branch.

## Category scorecard

| Category | PHYDEC baseline | Reference approach | Current verdict |
|---|---|---|---|
| Secondary-motion integration | Verlet-style update plus spring-to-anchor force and segment-length constraints | Exact closed-form damped oscillator for a constant-coefficient spring | **Trade-off found**: exact integration is timestep-partition invariant for constant forcing; the approximate integrator costs less per update |
| Gravity / parent acceleration | Gravity and external acceleration; parent motion represented through animated anchors | Gravity sag plus explicit parent-acceleration input | **Different API semantics**: constant forcing and piecewise-constant parent-acceleration changes are now tested; rapidly varying animation input still needs realistic pose-driven comparisons |
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
one-step versus 100-step partitioning; constant parent acceleration; piecewise-constant parent
acceleration changes with variable timestep partitions; an observable response to a sudden
parent-acceleration change; analytic gravity sag; zero timestep; and rejection of non-finite
settings without state mutation. A long-run sweep runs 12,000 steps for each of six damping
ratios at each of three timestep sizes and checks that the state remains finite. These are
bounded regression scenarios, not a proof that all parameter combinations are stable or
physically appropriate.

The piecewise forcing test keeps acceleration-change boundaries fixed and varies only the
subdivision inside each constant-input interval. It does not model a real animated parent track
or establish that one solver produces more pleasing motion on a character.

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
the same run. It performs different work, so this is **not** a direct speed ratio and must not
be used to claim the exact solver makes the full engine faster or slower. Timings are informational,
not CI thresholds.

## Next comparison passes

1. Compare linked-chain behavior under parent translation/rotation and realistic animation acceleration, including upstream-to-downstream response.
2. Prototype a hybrid exact-spring chain that feeds spring offsets through distance constraints. The new isolated header is `engine/include/phydec/experimental/hybrid_exact_spring_chain.hpp`; it is not part of the default solver API. Compare segment-length error and motion response before deciding whether any behavior belongs in production.
3. Add collision integration to the hybrid prototype only after its constraint behavior passes CI and comparison tests. Then test collision approaches separately, including fast motion and actual triangle meshes. Do not use simple point colliders as a substitute for PHYDEC's detailed-mesh collision requirement.
4. Evaluate weight-painted deformation as a feature-coverage category when the core deformation API is in scope.
5. After solver comparison, start the explicitly planar 2D math/secondary-motion prototype and compare it against a constrained 3D reference.
6. Decide category by category. Adopt only the parts that win on correctness/stability and justify integration cost. Do not add duplicate production solvers unless distinct semantics are useful.
