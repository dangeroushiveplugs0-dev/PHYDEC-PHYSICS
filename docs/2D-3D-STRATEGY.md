# 2D and 3D support strategy

## Decision

PHYDEC is **3D-first, dimension-aware, and intended to support 2D as a first-class use case**. This is an architecture direction, not a claim that a complete 2D solver exists today.

The first 2D support target is planar secondary motion and collision. Later 2D soft-body and deformation work should reuse the common foundation where the physics and requirements genuinely match.

## Design principles

1. **One core, dimension-specific behavior where necessary.** Reuse scalar math, spring integration, timestep handling, material parameters, diagnostics, and testing patterns. Do not force all 2D behavior through expensive or confusing 3D-only abstractions.
2. **Explicit 2D plane semantics.** A 2D simulation has two in-plane coordinates and a declared plane/orientation when embedded in a 3D scene. For a simple XY-plane mode, out-of-plane movement is constrained rather than left to drift.
3. **Do not equate 2D with a thin 3D world.** A 2D collider should have clear planar intersection and contact semantics. A 3D circle/sphere or triangle mesh is not automatically the correct 2D shape.
4. **Keep the renderer out of the core.** The same 2D simulation should work for sprites, skeletal animation, UI-driven motion, or a renderer that embeds the plane in a 3D scene.
5. **No duplicate physics implementations without evidence.** Compare shared scalar/vector formulations against separate 2D-specialized routines for correctness, complexity, and measured performance.
6. **Preserve 3D quality.** 2D support must not weaken the existing 3D collision, CCD, BVH, or secondary-motion behavior.

## Proposed dimensional model

- **2D vector:** two components and planar operations, with no implicit third-axis behavior.
- **2D rotation:** one scalar angle for planar rotation; avoid unnecessary 3D quaternion machinery in 2D-only APIs.
- **2D colliders:** start with circle, segment/capsule-like shape, and line/plane contact, then polygon/edge geometry as requirements demand.
- **2D mesh collision:** use edges/polygons and a suitable acceleration structure; do not pass 2D polygons into 3D triangle-mesh APIs and pretend semantics are identical.
- **2D secondary motion:** first validate a planar point-chain/spring model with gravity, damping, stiffness, and contacts.
- **3D integration:** provide an adapter/embedding convention to map planar results to a chosen 3D plane when users need a 2D rig inside a 3D application.
- **Soft bodies:** later decide whether 2D area/edge constraints and 3D volume/shape constraints share an interface or require distinct solvers. Do not assume a single constraint model is ideal for both.

## Compatibility goals

Potential use cases include 2D skeletal/sprite animation, side-scrolling games, planar character secondary motion, UI spring motion, and 2D soft-body experiments. These are target use cases, not features claimed as already implemented.

The API should let a consumer explicitly choose a dimensional mode, rather than infer 2D from all Z values happening to be zero. That makes the intended constraints and collision semantics unambiguous.

## Test and benchmark plan

### 2D correctness
- Planar invariance: a simulation configured for the XY plane never produces unintended Z displacement or velocity.
- Gravity and spring response along both in-plane axes.
- Rest, damping, stiffness/frequency sweeps, and long-running behavior.
- Circle/edge and capsule-like contacts, including grazing and degenerate geometry.
- Consistent results when the same simulation is embedded in different world planes.
- Boundary and tolerance cases, including zero-length edges and non-finite input handling.

### Cross-dimensional comparisons
- Equivalent 2D scenario against a deliberately constrained 3D reference, comparing projected positions and contact outcomes within tolerance.
- Confirm that any mismatch is intentional and documented, not a hidden coordinate convention bug.
- Compare common-code and specialized-code implementations before optimizing.

### Performance
- Benchmark repeated 2D updates at increasing point/bone counts.
- Measure 2D broad/narrow-phase collision separately from spring integration.
- Compare against constrained 3D only as context, not as proof of a universal speedup.
- Do not claim Android performance until measured on target Android hardware.

## Implementation order

1. Finish the current solver comparison experiment without changing the production solver based on assumptions.
2. Specify a small dimensional math/collision API, including explicit plane conventions.
3. Add tested 2D vector/rotation primitives and a minimal planar secondary-motion prototype.
4. Add 2D collision primitives and regression tests.
5. Benchmark and compare the 2D implementation with a constrained 3D reference.
6. Extend to 2D soft-body/deformation only after the initial 2D foundation proves useful.

## Current status

Planning decision recorded. No 2D runtime API or solver has been implemented by this document alone. The engine remains C++20 and renderer-independent.
