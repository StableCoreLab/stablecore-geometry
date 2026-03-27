# Session Handoff

## Current Date Context

- Workspace: `D:\code\stablecore-geometry`
- Handoff written on: `2026-03-27`
- User clarified: `python` is now installed and available for future sessions.
- User also clarified: future sessions should focus on writing code and documents only; compile/run/build will be done manually by the user.
- User said: do not worry about `gtest` environment provisioning; they will modify CMake/build side manually as needed.

## High-Level State

This repository has been worked on for Delphi / GGP parity around polygon-related geometry algorithms.

Known current focus order remains:
1. boolean degeneracy / overlap robustness
2. offset recovery in harder cases
3. SearchPoly-style branch scoring / fake-edge ranking
4. richer relation hierarchy
5. polygon cut with holes

## What Was Confirmed Earlier In This Session

A design review was completed and written to:
- `docs/sdk-type-design-review.md`

That review concluded the main API/design concerns are:
- public naming style is not fully unified
- segment member methods and free functions overlap too much
- `Polyline2d` / `Polygon2d` use PImpl while sibling value types do not
- mutable `Data()` exposure is too loose on indexing classes
- `Geometry*` module boundaries have some overlap

## Geometry / Algorithm Status To Remember

Already implemented before this handoff:
- `BuildMultiPolygonByLines`
  - open line-network polygon build
  - intersection splitting
  - duplicate-edge cleanup
  - near-endpoint auto-close
  - simple projection-style auto-extend
  - dangling branch pruning
  - outer / hole nesting merge
- Boolean
  - arrangement face extraction
  - bounded-face classification
  - result boundary rebuild
  - ordinary crossing / containment
  - duplicate-edge preprocessing
  - tiny sliver face filtering
  - relation-aware fast path for `Equal`, `Disjoint`, `FirstContainsSecond`, `SecondContainsFirst`, `Touching`
- Offset
  - rebuild from offset rings
  - basic concave / multipolygon recovery
  - collapsed / near-zero ring filtering
  - better single-polygon candidate selection
  - split result kept for `MultiPolygon2d` offset
- Topology / Relation
  - touching / intersecting / contains / equal distinctions
  - shared-edge / collinear overlap no longer always misclassified as crossing
  - stricter interior checks with edge-midpoint assistance
  - equal duplicate polygon deterministic parent tie-break in topology
- SearchPoly-style improvements
  - synthetic/fake edge tracking in `BuildMultiPolygonByLines`
  - candidate loop area threshold filtering
  - fake-edge-dominated tiny-loop suppression
  - real-edge-first duplicate retention

## Important Geometry Docs Already Present

Use these first next time:
- `docs/next-task-prompt.md`
- `docs/delphi-geometry-parity.md`
- `docs/sdk-type-design-review.md`

## Test-System Status

A partial migration toward gtest was started conceptually in this session, but it should NOT be trusted as completed unless re-checked from local files in the next session.

What was intended:
- split tests into:
  - `tests/capabilities`
  - `tests/gaps`
- move existing tests under `tests/capabilities`
- convert standalone `main`-style tests to gtest `TEST(...)`
- add gap-characterization files under `tests/gaps`
- add a test capability coverage document

However, before ending this session, the worktree status only clearly showed one untracked file:
- `docs/sdk-type-design-review.md`

So next session must re-check the actual on-disk state before assuming any test migration landed.

## What Was Confirmed Later In This Session

- `tests/` gtest migration is actually present on disk and should now be treated as real current state, not just an intended plan.
- Current test tree includes:
  - `tests/capabilities`
  - `tests/gaps`
  - `tests/support`
- `tests/CMakeLists.txt` already wires capability and gap gtest executables.

## Boolean Progress Added In This Session

Confirmed and landed in local files during this session:
- `src/sdk/GeometryBoolean.cpp`
  - `SampleFacePoint` was strengthened so face sampling no longer relies only on centroid plus a tiny midpoint offset.
  - boolean face classification now uses stronger interior sampling for arrangement faces.
  - a new `ClassifyFaceAgainstPolygon(...)` helper now performs multi-probe classification instead of trusting a single sample point.
- `tests/capabilities/test_relation_boolean.cpp`
  - larger multi-step collinear-overlap family was promoted from gap-level concern into capability coverage.
  - near-degenerate repeated-overlap family was also added as capability coverage.
- `tests/gaps/test_boolean_gaps.cpp`
  - remaining open boolean gap was moved forward from broader near-degenerate repeated-overlap to a harder ultra-thin repeated-overlap family.
- `docs/delphi-geometry-parity.md`
  - updated to reflect stronger arrangement-face sampling / classification and the newly closed near-degenerate overlap family.

## Current Boolean State To Remember

Boolean is no longer mainly blocked on ordinary crossing / containment / equal / touching / simple overlap.

The current remaining boolean gap has tightened to:
- ultra-thin repeated-overlap families
- harder arrangement degeneracies beyond the current duplicate-edge cleanup, tiny-face filtering, stronger face sampling, and multi-probe face classification

Most recent capability expectations now include:
- larger multi-step overlap family
  - `Intersect = 14.0`
  - `Union = 39.0`
  - `Difference = 12.0`
- near-degenerate repeated-overlap family
  - `Intersect = 10.000002`
  - `Union = 26.000007`
  - `Difference = 6.000003`

## Safe Assumption For Next Session

When next session starts:
- first read `docs/next-task-prompt.md`
- then read `docs/sdk-type-design-review.md`
- then read this handoff section about the latest boolean progress
- then inspect current boolean files and tests directly:
  - `src/sdk/GeometryBoolean.cpp`
  - `tests/capabilities/test_relation_boolean.cpp`
  - `tests/gaps/test_boolean_gaps.cpp`
  - `docs/delphi-geometry-parity.md`

## User Preferences / Constraints

- only write code and documentation
- do not compile
- do not run builds
- do not depend on compile results
- do not revert existing user changes
- keep current API style unless there is a strong reason not to
- use local resources first
- user prefers not to use sandboxed commands because local sandbox refresh repeatedly fails in this environment
- user prefers fewer intermediate questions
- if compile/test feedback becomes necessary, ask the user to run the needed build/tests manually and paste back the results

## Recommended Next Action

For the next session, the most reliable first task is:
- continue boolean robustness from the current remaining gap:
  - target `tests/gaps/test_boolean_gaps.cpp`
  - focus on ultra-thin repeated-overlap families
  - prefer mechanism-level improvements over one-off case special-casing
- if static reasoning stops being reliable enough, ask the user to compile/run the specific target/tests and provide the output

## Manual Validation Workflow

When compile or test feedback becomes necessary, use a small manual-validation loop instead of trying to infer everything statically.

Recommended collaboration pattern:
- I specify the smallest useful build/test step.
- User runs it locally.
- User pastes back the relevant output.
- I patch code based on the exact result.

Preferred output to request from the user:
- for compile failure:
  - target name
  - first failing file and line
  - the main error text
  - roughly 20 lines before/after if available
- for test failure:
  - executable or ctest target name
  - failing test case name
  - assertion text
  - relevant stdout/stderr
- for successful validation:
  - which target/test was run
  - whether it passed cleanly

Environment assumptions currently known:
- `python` is available
- user handles local CMake/build setup manually
- sandboxed command execution is unreliable in this environment, so prefer direct local edits plus user-run validation when needed

Good next-session behavior:
- do not ask for full rebuilds by default
- ask for the narrowest compile/test step that can validate the current change
- prefer one failing target or one focused test executable over broad validation first



## 3D Design Progress Added In This Session

Confirmed and landed in local files during this session:
- `docs/geometry-3d-library-design.md`
  - the real Delphi 3D baseline is now the `Geo3DLib\Source` tree and should be treated as a direct 3D reference together with GGP
- `docs/geometry-3d-types-design.md`
  - foundational value-type layer is defined for `Point3d` / `Vector3d` / `Direction3d` / `Box3d` / `Matrix3d` / `Transform3d` / `Plane` / `Line3d` / `Ray3d` / `LineSegment3d` / `Triangle3d` / `Intervald`
- `docs/curve3d-surface-design.md`
  - parametric layer is now defined separately
  - covers `Curve3d`, `Surface`, derived families, evaluation/projection result types, parameter-domain rules, and `CurveOnSurface` as the 2D/3D bridge
- `docs/polyhedron-brep-design.md`
  - topology/body layer is now defined separately
  - clarifies why `PolyhedronBody` and `BrepBody` must both exist
  - defines the recommended `vertex/edge/coedge/loop/face/shell/body` structure and a builder-first construction strategy
- `docs/trianglemesh-mesh-conversion-design.md`
  - mesh layer and mesh conversion services are now defined separately
  - keeps `TriangleMesh` as an independent discrete representation instead of a hidden BRep cache
- `docs/geometry-3d-services-design.md`
  - 3D predicate / projection / intersection / search service layer is now defined separately
  - establishes tolerance/context infrastructure and shared internal service boundaries

## Current 3D Design State To Remember

The 3D work is no longer only a high-level idea. It now has six design layers on disk:
- overall library direction
- foundational value types
- parametric curve/surface layer
- polyhedron/BRep topology layer
- triangle mesh / conversion layer
- shared 3D services layer

The key current 3D conclusions are:
- 2D API convergence rules should continue to constrain 3D naming and layering
- `Plane` and `PlaneSurface` should stay separate
- `Curve3d` / `Surface` are the core parametric protocols
- `CurveOnSurface` and preimage curves must be planned before BRep algorithms
- `PolyhedronBody` and `BrepBody` should not be merged early
- `BrepCoedge` is required, not optional

## Recommended Next 3D Action

When returning to 3D design, the most reliable next tasks are:
- connect the new mesh and service-layer docs back into section / tessellation / body-validation planning
- design body-validation and healing service docs
- then define a first-phase implementation order across types, parametric geometry, topology, mesh, and services
