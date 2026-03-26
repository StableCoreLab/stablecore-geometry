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

The current C++ repository now covers a larger portion of the Delphi geometry algorithm capability, but it still has not reached full Delphi parity.

Current status summary:

- basic geometry kernel: largely covered
- open and branched line-network polygon reconstruction: partially covered
- polygon boolean on crossing and contained polygon cases: partially covered
- polygon relation and topology classification: partially covered
- polygon offset cleanup through rebuild: partially covered
- Delphi-level search/build recovery, branch pruning, and deep offset robustness: still below Delphi level

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

These areas exist in the current C++ repository, and this round improved them substantially, but implementation depth or robustness is still below Delphi.

- polygon boolean operations
- polygon offset
- polygon topology hierarchy
- polygon cut by line
- build multipolygon from line input
- polygon relation aggregation
- geometry search infrastructure for downstream polygon workflows

Reasons for partial status:

- the APIs exist and now cover more crossing, nested, and branched cases
- line-network polygon reconstruction now performs intersection splitting before face extraction
- boolean now works through arrangement face classification and boundary rebuild
- offset now rebuilds polygon results from generated offset rings instead of rejecting invalid intermediate rings immediately
- complex topology cleanup, ambiguous branch handling, auto-close, auto-extend, and difficult recovery logic are still not yet equivalent to Delphi

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

- automatic closing and extension for search-based polygon extraction in complex branched networks
- branch removal and duplicate-segment cleanup strategies comparable to Delphi `SearchPoly`
- richer search heuristics and recovery for ambiguous polygon reconstruction paths
- complex offset cleanup for reverse edges, invalid circles, multi-failure recovery, and more difficult self-intersection cases
- Delphi-level end-to-end polygon search/build workflows around `SearchPoly`

Representative Delphi references:

- `GGLSearchPolyFunc2d.pas`
- `GGLOffsetFunc2d.pas`
- `GGLPolyRelation.pas`

## 5. Key Evidence For The Current Gap

### 5.1 Boolean Has Moved Beyond Simple Cell Cases But Is Still Not Delphi-Level

The C++ boolean implementation is no longer just simple scan cells. It now rebuilds a segment arrangement, classifies bounded faces, and reconstructs the selected result boundary. This closes part of the previous gap for crossing and contained polygons.

Current C++ evidence:

- `src/sdk/GeometryBoolean.cpp`
- `tests/test_relation_boolean.cpp`

Still missing compared with Delphi:

- broader recovery strategies on more difficult polygon graphs
- Delphi-level polygon search workflow integration
- deeper robustness around difficult degeneracies and overlapping cases

### 5.2 BuildMultiPolygonByLines Has Improved To Intersection-Splitting Face Reconstruction

The C++ implementation now reconstructs polygons from open and branched line networks after splitting line intersections. This is stronger than the earlier closed-polyline-only behavior.

Current C++ evidence:

- `src/sdk/GeometryPathOps.cpp`
- `tests/test_shapes_pathops.cpp`

Still missing compared with Delphi:

- branch-oriented search strategies
- auto-close and auto-extend workflows
- branch pruning and more aggressive topology cleanup
- richer recovery from difficult or ambiguous line networks

Representative Delphi references:

- `GGLSearchPolyFunc2d.pas:1614`
- `GGLSearchPolyFunc2d.pas:1779`
- `GGLSearchPolyFunc2d.pas:2876`
- `GGLSearchPolyFunc2d.pas:3761`

### 5.3 Offset Now Rebuilds From Generated Rings But Is Still Not Delphi Offset Depth

The current C++ offset logic now feeds generated offset rings back into polygon reconstruction, which is stronger than rejecting the result immediately when an intermediate ring becomes invalid.

Current C++ evidence:

- `src/sdk/GeometryOffset.cpp`
- `tests/test_offset.cpp`

Still missing compared with Delphi:

- dedicated reverse-edge cleanup
- invalid-circle cleanup and deeper loop filtering
- more complete recovery when offset output splits or becomes ambiguous

Representative Delphi references:

- `GGLOffsetFunc2d.pas:549`
- `GGLOffsetFunc2d.pas:636`
- `GGLOffsetFunc2d.pas:751`
- `GGLOffsetFunc2d.pas:875`

### 5.4 Relation And Topology Are Stronger Than Before But Still Simpler Than Delphi PolyRelation

The C++ relation and topology logic now checks polygon boundary contact together with containment, so touching and intersecting cases are stronger than the earlier vertex-only containment logic.

Current C++ evidence:

- `src/sdk/GeometryTopology.cpp`
- `tests/test_topology_indexing.cpp`

Still missing compared with Delphi:

- richer multi-level relation management comparable to `GGLPolyRelation`
- more complete integration with complex search/build workflows

## 6. Current Comparison Baseline

Until this document is updated, future parity review should use the following rule:

- if a C++ feature works only for straightforward branching and ordinary crossing cases, it still does not count as Delphi parity for the full `SearchPoly` class of capability
- if a C++ feature lacks explicit branch recovery, auto-close, auto-extend, or heavy cleanup phases, it still remains below Delphi for production-hard polygon workflows
- coordinate transform and drawing modules remain out of scope even if they exist in Delphi

## 7. Suggested Tracking Method

Future implementation comparison should continue to categorize each algorithm area into:

- reached
- partially reached
- not yet reached

Recommended next tracking focus:

- complex branched polygon-search workflows
- stronger offset cleanup and recovery
- broader degeneracy handling in polygon boolean
- richer polygon relation hierarchy behavior
