# Engineering Decision: Compare Before Replacing

## Language policy

- The engine runtime and public core remain C++20.
- Do not add JavaScript or another runtime language because an external reference uses it.
- Python may be considered later for tooling, offline analysis, test-data generation, or scripts, but only when it solves a concrete problem. It is not a required runtime dependency.
- When an external algorithm is useful, translate the algorithm into idiomatic C++ and preserve the source project's required copyright and license notices when code is reused.

## Evaluation policy

Finding an existing implementation triggers an evaluation, not an automatic rewrite.

For each substantial subsystem, compare:
1. **Correctness:** equivalent inputs should produce results within stated tolerances; include edge cases and regression cases.
2. **Stability:** vary frame delta, acceleration, stiffness/frequency, damping, and simulation duration. Look for divergence, NaNs, excessive drift, and sensitivity to frame rate.
3. **Feature coverage:** compare what the subsystem actually supports, not only what the README claims.
4. **Complexity:** assess algorithmic complexity, allocations, code size, maintenance burden, and integration cost.
5. **Performance:** benchmark representative workloads in the same C++ build configuration and on the same machine. Report setup/preparation cost separately from repeated-query cost.
6. **Portability:** consider C++20 compatibility, Android/NDK constraints, deterministic behavior, and public API fit.
7. **Licensing:** verify the license and retain required notices before porting source code.

Prefer the simplest approach that satisfies all required behavior. Keep a more complicated implementation only when tests or measurements demonstrate a meaningful benefit. A benchmark win does not override correctness or stability regressions.

## Current case: xloveee/jiggle-physics

Reference: https://github.com/xloveee/jiggle-physics

The reference is JavaScript, dependency-free, and licensed BSD 3-Clause. Its core uses an exact closed-form damped-oscillator step with parent acceleration and gravity sag; extensions provide linked spring bones and point-vs-plane/sphere/capsule collision. It explicitly does not claim to be a full soft-body FEM or cloth/self-collision engine.

PHYDEC currently uses a Verlet-style point-chain update with gravity, external acceleration, a spring-to-anchor term, segment-length constraints, and integrated collision handling. Gravity is already implemented. The solvers have different state models and semantics, so a comparison must use matched scenarios and carefully defined output metrics rather than simply swapping formulas.

### Proposed experiment

Build an isolated C++20 prototype of the reference oscillator. Do not replace the production solver initially.

- Use deterministic initial conditions and parameter sets.
- Compare rest behavior, gravity sag, parent-acceleration response, damping regimes, variable frame deltas, and long-running numerical behavior.
- Measure runtime for equivalent numbers of points/bones, reporting compiler/build mode and workload.
- Compare output trajectories and quantify differences; use tolerances and expected behavior, not a claim that two different models must produce identical trajectories.
- Include collision behavior separately because the reference's collider extension is point projection, while PHYDEC has mesh collision and CCD.
- Record results and choose whether to keep the current solver, replace its integration model, or expose distinct solver modes. Do not add two production paths unless their different behavior is a real user-facing requirement.

No performance or quality conclusion is assumed in advance. The result of the experiment decides the next step.
