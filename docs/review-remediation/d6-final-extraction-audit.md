# D6 final extraction audit (2026-09-02)

## Shared, source-neutral helpers

- `DirectedEdgeFans2d.h`: outgoing-fan ordering and next-face traversal.
- `SegmentParameters2d.h`: parameter clamping and sorted clustering.
- `SegmentSubdivision2d.h`: intersection scanning and split endpoints, with source indices returned to the caller.
- `ArrangementVertices2d.h`: tolerance-based vertex indexing and order-independent edge keys.
- `RingVertices2d.h`: consecutive duplicate removal and closing-point normalization only.
- `PolygonNesting2d.h`: smallest containing polygon parent selection.

Each helper is private to `Source/Detail`; none exposes a public API or embeds a module-specific epsilon policy.

## Deliberately retained implementations

| Routine | Why it remains local |
| --- | --- |
| `RemoveDuplicateSegments` | PathOps replaces a synthetic segment with a real segment; Boolean keeps first occurrence after its scale-aware vertex tolerance. |
| `AppendSplitEdges` | PathOps stores edge length and synthetic/visited state; Boolean has no synthetic input state and uses an independently computed vertex tolerance. |
| `SimplifyRingVertices` | Only duplicate normalization is shared. Boolean additionally removes nearly collinear vertices to stabilize Boolean faces; PathOps preserves those vertices for repair scoring and diagnostics. |
| Face/ring extraction and assembly | PathOps performs repair-aware candidate scoring and nesting depth; Boolean builds bounded faces then classifies them against operands. |

## Regression evidence

`TestRingTraversalRegression.cpp` directly covers directed-fan ties, parameter clustering, source-indexed crossing/overlap subdivision, vertex merging and edge keys, ring normalization, and nested-parent selection. Existing crossed-edge, overlap, narrow-gap, SearchPoly, and Boolean regressions verify the public results remain aligned.
