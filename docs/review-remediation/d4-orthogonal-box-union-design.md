# D4: face-connected axis-aligned box union

## Scope

Support the previously unsupported union of two recognized axis-aligned closed boxes when their occupied volumes form one face-connected orthogonal solid. This includes partial face contact and non-rectangular L-shaped results. It applies uniformly to `PolyhedronBody` and `SCBrepBody` through the existing conversion boundary.

It does not claim general 3D CSG: rotated boxes, non-box bodies, edge-only contact, vertex-only contact, disconnected unions, and general difference remain outside this capability.

## Algorithm

1. Reuse the existing axis-aligned-box recognizer for both inputs.
2. Form sorted, tolerance-deduplicated coordinate planes from the two boxes' min/max coordinates.
3. Mark each positive-volume grid cell occupied when its center belongs to either input box.
4. Require all occupied cells to be connected through faces. This preserves the existing multi-body policy for edge/vertex-only contact.
5. Emit only faces between an occupied and an unoccupied cell, with outward orientation; convert the resulting orthogonal `PolyhedronBody` using the normal B-rep conversion path.
6. Return success only when conversion produces a valid single closed shell. Otherwise preserve `UnsupportedOperation` and do not return a partial result.

## Contracts and invariants

- Result issue is `None`, `bodies` is empty, and one result body is present.
- The result body is valid, has exactly one closed shell, and volume equals the inclusion-exclusion volume of the two input boxes within tolerance.
- Input order does not change the occupied cells or output bounds.
- The fast paths for identical, contained, disjoint, single-box union, edge touching, and vertex touching remain unchanged.
