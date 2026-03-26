# Delphi Geometry Parity

## 1. Purpose

This document records the comparison baseline between the current C++ repository and the Delphi implementation under:

- `D:\code\GFY2.0\Controls\GAEAResource\GCL\Geo2DLib\Source`

It is used as the stable reference for future capability tracking and implementation parity work.

## 2. Comparison Scope

Only geometry algorithm capability is in scope.

Included:

- 2D geometry types
- geometric predicates and tolerance rules
- distance, projection, sampling, and metrics
- segment intersection and relation logic
- polyline and polygon algorithms
- offset, boolean, topology, and polygon-building logic
- geometry-oriented spatial indexing and search

Explicitly excluded:

- `GGLCoordTrans.pas`
- `GGLDrawFunc2d.pas`

Notes:

- Future comparisons should continue to exclude coordinate transform and drawing/rendering code.
- This document evaluates source capability only.
- Do not rely on compilation or build output for this comparison baseline.

## 3. Current Overall Conclusion

The current C++ repository now covers a larger portion of the Delphi geometry algorithm capability for polygon and face workflows, but it still has not reached full Delphi parity.

Current status summary:

- basic geometry kernel: largely covered
- line-network polygon reconstruction with splitting, duplicate-edge cleanup, nearby auto-close, simple auto-extend, and branch pruning: substantially improved
- polygon boolean on ordinary crossing, containment, exact-equal, and basic collinear-overlap cases: partially covered
- polygon relation and topology classification with equal/shared-edge recovery: partially covered
- polygon offset rebuild, split recovery, and basic cleanup: partially covered
- Delphi-level polygon search recovery on heavily ambiguous dirty input and deepest offset/boolean robustness: still below Delphi level

## 4. Capability Classification

### 4.1 Reached Or Basically Reached

These areas already have clear C++ types, public APIs, and implementation support.

- basic point/vector/box/segment abstractions
- line segment and arc segment metrics, projection, tangent, normal, and point-at-parameter operations
- basic polyline and polygon area, perimeter, centroid, orientation, and bounds operations
- point containment for line, ring, and polygon
- translate, rotate, mirror, and stretch transforms for core geometry types
- basic KD-tree and box-tree style query capability
- basic segment search by distance and nearest point

Representative C++ references:

- `include/sdk/GeometryProjection.h`
- `include/sdk/GeometryMetrics.h`
- `include/sdk/GeometryShapeOps.h`
- `include/sdk/GeometryRelation.h`
- `include/sdk/GeometryTransform.h`
- `src/sdk/GeometryKDTree.cpp`
- `src/sdk/GeometryBoxTree.cpp`

Representative verified tests:

- `tests/test_sdk_algorithms.cpp`
- `tests/test_transform_sampling.cpp`
- `tests/test_topology_indexing.cpp`

### 4.2 Partially Reached

These areas exist in the current C++ repository, and recent work improved them substantially, but implementation depth or robustness is still below Delphi.

- polygon boolean operations
- polygon offset
- polygon topology hierarchy
- polygon cut by line
- build multipolygon from line input
- polygon relation aggregation
- geometry search infrastructure for downstream polygon workflows

Reasons for partial status:

- the APIs exist and now cover more crossing, nested, and branched cases
- line-network polygon reconstruction now performs duplicate cleanup, intersection splitting, nearby endpoint auto-close, simple projection-based auto-extend, dangling-branch pruning, and candidate filtering that down-ranks fake-edge-dominated tiny loops before face extraction
- boolean now works through relation-aware fast paths plus arrangement face classification and boundary rebuild, with duplicate-edge preprocessing and tiny-face filtering to reduce overlap/degenerated slivers
- offset now rebuilds polygon results from generated offset rings, filters collapsed rings before rebuild, and better preserves the semantically correct result when rebuild returns multiple candidates
- relation/topology now distinguishes equal/shared-edge overlap more reliably instead of collapsing them into generic crossing cases
- complex topology cleanup, heavily ambiguous branch handling, stronger fake-edge heuristics, and difficult recovery logic are still not yet equivalent to Delphi

Representative C++ references:

- `src/sdk/GeometryBoolean.cpp`
- `src/sdk/GeometryOffset.cpp`
- `src/sdk/GeometryPathOps.cpp`
- `src/sdk/GeometryTopology.cpp`
- `src/sdk/GeometrySegmentSearch.cpp`

Representative Delphi references:

- `GGLOffsetFunc2d.pas`
- `GGLPolyRelation.pas`
- `GGLSearchPolyFunc2d.pas`

### 4.3 Not Yet Reached

These Delphi geometry algorithm capabilities do not yet have equivalent depth in the current C++ repository.

- branch scoring, fake-edge strategies, and richer search heuristics comparable to Delphi `SearchPoly`
- stronger auto-close and auto-extend logic for heavily ambiguous or multi-candidate branched networks beyond the current synthetic-edge filtering stage
- complex offset cleanup for reverse edges, invalid circles, multi-failure recovery, and more difficult self-intersection cases
- deeper boolean robustness on larger overlap families and harder near-degenerate arrangements beyond the current duplicate/sliver cleanup
- Delphi-level end-to-end polygon search/build workflows around `SearchPoly`

Representative Delphi references:

- `GGLSearchPolyFunc2d.pas`
- `GGLOffsetFunc2d.pas`
- `GGLPolyRelation.pas`

## 5. Current Gap List

### 5.1 Must Close

These are the remaining gaps most likely to block Delphi-level polygon capability.

- branch-aware polygon search for heavily ambiguous line networks, not only already-repairable linework
- broader boolean robustness on larger collinear-overlap families and harder near-degenerate intersections; repeated-edge cleanup and tiny-sliver suppression are now partially closed
- stronger offset recovery for the remaining self-intersection and reverse-edge cases; split recovery and collapsed-ring filtering are now partially closed
- broader preprocessing parity across boolean and offset; duplicate-edge cleanup, tiny-face filtering, and collapsed-ring filtering now extend beyond line-network polygon building
- more Delphi-like fake-segment and candidate-ranking strategies when multiple closure paths are possible; basic fake-edge-dominated tiny-loop suppression is now present but still well below Delphi smart search

### 5.2 Should Close

These gaps do not block basic polygon workflows anymore, but they still matter for production-hard parity.

- richer polygon relation hierarchy comparable to Delphi `GGLPolyRelation`
- better polygon cut and split workflows for holes, multi-cut input, and line-network-assisted partitioning
- more explicit topology recovery between line search, face extraction, boolean, and offset pipelines
- stronger tolerance propagation so the same `eps` policy behaves consistently through splitting, ring extraction, containment, and rebuild
- broader search helpers for downstream polygon workflows instead of leaving most cleanup to individual algorithms

### 5.3 Can Be Deferred

These areas are still differences, but they are lower priority than the items above.

- performance tuning for large arrangements and repeated rebuild steps
- longer-lived spatial index reuse instead of rebuilding local structures inside each workflow
- more exhaustive test matrices for difficult geometry families after the remaining algorithm gaps are closed
- API reshaping to expose more intermediate topology products if needed by higher layers

## 6. Key Evidence For The Current Gap

### 6.1 Boolean Has Moved Beyond Simple Cell Cases But Is Still Not Delphi-Level

The C++ boolean implementation is no longer just simple scan cells. It now uses relation-aware short-circuit paths for equal/disjoint/containment/touching cases, then rebuilds a segment arrangement when needed, removes repeated undirected edges before arrangement, and filters very small bounded faces before result rebuild. This closes more of the previous gap for crossing, contained, equal, touching, and simple overlap cases.

Current C++ evidence:

- `src/sdk/GeometryBoolean.cpp`
- `tests/test_relation_boolean.cpp`

Still missing compared with Delphi:

- deeper robustness around harder degeneracies and large overlap families
- broader cleanup and recovery strategies on more difficult polygon graphs beyond duplicate-edge and tiny-face filtering
- stronger integration with heavier polygon search workflows

### 6.2 BuildMultiPolygonByLines Now Includes Repair Before Face Reconstruction

The C++ implementation now reconstructs polygons from open and branched line networks after duplicate cleanup, intersection splitting, nearby endpoint auto-close, simple projection-based auto-extend, dangling-branch pruning, and rejection of fake-edge-dominated tiny candidate loops. This closes a meaningful part of the earlier `SearchPoly` gap for ordinary dirty linework and some ambiguous closure leftovers.

Current C++ evidence:

- `src/sdk/GeometryPathOps.cpp`
- `tests/test_shapes_pathops.cpp`

Still missing compared with Delphi:

- branch-oriented scoring and search strategies
- richer fake-edge insertion policies
- stronger recovery from heavily ambiguous line networks
- fuller equivalence to Delphi smart search and closure workflows

Representative Delphi references:

- `GGLSearchPolyFunc2d.pas:1614`
- `GGLSearchPolyFunc2d.pas:1779`
- `GGLSearchPolyFunc2d.pas:3018`
- `GGLSearchPolyFunc2d.pas:3761`
- `GGLSearchPolyFunc2d.pas:3905`

### 6.3 Offset Now Rebuilds From Generated Rings But Is Still Not Delphi Offset Depth

The current C++ offset logic now feeds generated offset rings back into polygon reconstruction, filters collapsed/near-zero rings before rebuild, and chooses a better semantic survivor when a single-polygon offset rebuild returns multiple candidates. This is stronger than rejecting the result immediately when an intermediate ring becomes invalid.

Current C++ evidence:

- `src/sdk/GeometryOffset.cpp`
- `tests/test_offset.cpp`

Still missing compared with Delphi:

- dedicated reverse-edge cleanup
- invalid-circle cleanup and deeper loop filtering than the current collapsed-ring filter
- more complete recovery when offset output becomes more ambiguous than the current split/narrow-bridge cases
- better handling of the remaining narrow-channel, loop-collapse, and hole-inversion edge cases

Representative Delphi references:

- `GGLOffsetFunc2d.pas:549`
- `GGLOffsetFunc2d.pas:636`
- `GGLOffsetFunc2d.pas:751`
- `GGLOffsetFunc2d.pas:875`

### 6.4 Relation And Topology Are Stronger Than Before But Still Simpler Than Delphi PolyRelation

The C++ relation and topology logic now checks polygon boundary contact together with containment, avoids treating shared collinear edges as automatic crossing, and correctly recovers equal polygons in the basic coincident case. Touching and intersecting cases are therefore stronger than the earlier vertex-only containment logic.

Current C++ evidence:

- `src/sdk/GeometryTopology.cpp`
- `tests/test_topology_indexing.cpp`

Still missing compared with Delphi:

- richer multi-level relation management comparable to `GGLPolyRelation`
- more complete integration with complex search/build workflows
- more systematic handling of degenerate and nearly coincident relation inputs

## 7. Current Comparison Baseline

Until this document is updated, future parity review should use the following rule:

- if a C++ feature works only for straightforward branching and ordinary crossing cases, it still does not count as Delphi parity for the full `SearchPoly` class of capability
- if a C++ feature lacks explicit ambiguity resolution, fake-edge ranking, or heavier cleanup phases, it still remains below Delphi for production-hard polygon workflows
- if a C++ feature handles ordinary offset but does not recover well from self-intersection, split output, or collapse, it still remains below Delphi offset capability
- coordinate transform and drawing modules remain out of scope even if they exist in Delphi

## 8. Suggested Tracking Method

Future implementation comparison should continue to categorize each algorithm area into:

- reached
- partially reached
- not yet reached

Recommended next tracking focus:

- stronger overlap and degeneracy handling in polygon boolean
- stronger offset cleanup and recovery
- branch scoring and ambiguity resolution in polygon search
- richer polygon relation hierarchy behavior
- unified geometry preprocessing before all face operations
