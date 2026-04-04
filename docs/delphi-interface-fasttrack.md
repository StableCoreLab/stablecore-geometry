# Delphi Interface Fast-Track

## Goal

This file fixes the fast-track replacement target for the Delphi geometry stack:

- expose the full interface surface early
- back each public entry with contract or capability tests
- let product development start against stable C++ SDK names
- keep algorithm depth work behind those interfaces

## Delphi-To-C++ Interface Inventory

| Delphi capability family | Delphi evidence | C++ target surface | Current status |
| --- | --- | --- | --- |
| `SearchPoly` / fake-edge / auto-close / branch cleanup | `GGLSearchPolyFunc2d.pas` | `GeometrySearchPoly.h` + `GeometryPathOps.h` | branch-scored subset landed |
| polygon relation tree / contains hierarchy | `GGLPolyRelation.pas` | `GeometryRelation.h` + `GeometryTopology.h` | usable subset |
| offset with rebuilt polygon recovery | `GGLOffsetFunc2d.pas` | `GeometryOffset.h` | usable subset |
| 2D boolean | GGJ + Geo2DLib product use | `GeometryBoolean.h` | usable subset |
| segment search / box-tree acceleration | Geo2DLib | `GeometrySegmentSearch.h` + `GeometryBoxTree.h` | usable subset |
| section / projected contour rebuild | `GGJSumpCommon.pas`, `GGL3DCommon.pas` | `GeometrySection.h` | usable subset |
| polyhedron / brep conversion | GGJ conversion and rebuild path | `GeometryBrepConversion.h` | usable subset with open repair gaps |
| brep healing / trim backfill | GGJ healing path | `GeometryHealing.h` | usable subset with open aggressive-policy gaps |
| body / shell boolean | `GGL.pas` | `GeometryBodyBoolean.h` | first overlap subset landed |
| brep to mesh / mesh to body support chain | GGJ export + conversion use | `GeometryMeshConversion.h`, `GeometryMeshRepair.h`, `GeometryMeshOps.h` | usable subset with open fidelity gaps |

## Fast-Track Rules

1. Product should code only against SDK headers in `include/sdk`.
2. New Delphi-parity work should prefer adding stable entry points before deepening internals.
3. Every new interface must land with either:
   - a capability test proving a supported subset, or
   - a gap test explicitly documenting the missing algorithmic depth.
4. "Algorithm library done" means the agreed interface inventory has gone green for its committed tests, not that every theoretical geometry case is solved.

## First Fast-Track Batch

- `GeometrySearchPoly.h`
  - formalizes Delphi-style polygon search as a first-class SDK entry instead of leaving product to call `BuildMultiPolygonByLines(...)` ad hoc
  - current stable subset covers invalid-input contract, candidate ranking, branch scoring, candidate-level fake-edge diagnostics, and smallest-containing candidate lookup
- `GeometryBodyBoolean.h`
  - reserves Delphi-style body/shell boolean SDK names so product can wire against stable APIs now
  - current stable subset covers invalid-input contract, identical/disjoint closed-body subsets, and axis-aligned single-box overlap subsets whose result remains one closed box
- accompanying tests
  - `tests/capabilities/test_searchpoly_sdk.cpp`
  - `tests/capabilities/test_3d_body_boolean_sdk.cpp`
  - `tests/gaps/test_3d_body_boolean_gaps.cpp`

## Next Batches

- deepen `GeometrySearchPoly` from the current branch-scored + candidate fake-edge diagnostic subset toward richer fake-edge explanation and Delphi-grade ambiguous recovery
- deepen `GeometryBodyBoolean` from identical/disjoint + axis-aligned single-box overlap subsets toward non-box overlap, touching/shell-policy, and healing-integrated cases
- continue shrinking gap tests only when corresponding capability tests turn green
