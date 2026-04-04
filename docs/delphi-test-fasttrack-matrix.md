# Delphi Test Fast-Track Matrix

## Purpose

This matrix turns Delphi-replacement work into an interface-and-tests program.

## Contract And Capability Matrix

| SDK surface | Minimum tests required | Current state |
| --- | --- | --- |
| `GeometrySearchPoly.h` | invalid-input contract, closed-loop representative capability, candidate ranking, repair diagnostics, point-containing candidate lookup | second batch landed |
| `GeometryOffset.h` | line/arc/polyline/polygon offset contracts, rebuilt output capability, split-output capability | covered |
| `GeometryRelation.h` | point location, containment ordering, representative nested/hole topology capability | covered subset |
| `GeometryBoolean.h` | invalid operands, crossing/containment/equal/touching representative capabilities | covered subset |
| `GeometrySection.h` | section contract, deterministic contour/segment counts, topology/components, Brep path parity | covered subset |
| `GeometryBrepConversion.h` | invalid-input contract, representative conversion capabilities, explicit open-gap coverage | covered subset |
| `GeometryHealing.h` | conservative trim-backfill contract, representative aggressive-policy capabilities, explicit open-gap coverage | covered subset |
| `GeometryBodyBoolean.h` | invalid-input contract, identical/disjoint closed-body representative capability, explicit gap coverage beyond that subset | first deterministic subset landed |
| `GeometryMeshConversion.h` | representative Brep/Polyhedron mesh conversion capability, fidelity gaps | covered subset |

## Definition Of Done For Fast-Track

The algorithm library is considered ready for product development when:

1. the agreed Delphi-facing SDK interfaces exist;
2. each interface has contract tests;
3. each implemented subset has capability tests;
4. each unimplemented area is called out by a gap test;
5. the committed test matrix is green.

## Immediate Work Queue

1. deepen `GeometryBodyBoolean` beyond identical/disjoint closed-body subsets.
2. expand `GeometrySearchPoly` with branch scoring and fake-edge explanation.
3. keep syncing `docs/test-capability-coverage.md` and gap docs as soon as a subset changes state.
