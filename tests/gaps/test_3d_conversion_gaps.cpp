#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::BrepBody;
using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::PolyhedronBody;

TEST(Conversion3dGapTest, HighFidelityBrepToMeshFeaturePreservationRemainsOpen)
{
    const BrepBody body({});
    const auto converted = ConvertToTriangleMesh(body);
    (void)converted;
    GTEST_SKIP() << "Known 3D gap: high-fidelity feature-preserving Brep->mesh conversion beyond planar holed+multi-face area-preserving subset, shared-edge global vertex-reuse subset, and disconnected closed-shell component-preserving subset is still open.";
}

TEST(Conversion3dGapTest, GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen)
{
    const PolyhedronBody body;
    const auto mesh = ConvertToTriangleMesh(body);
    (void)mesh;
    GTEST_SKIP() << "Known 3D gap: robust non-planar polyhedron->Brep repair beyond affine-planar + support-plane-refit (including closed-shell shared-topology subset via triangular faces [closed tetrahedron], support-plane-mismatch quad shared-edge-chain shared-topology subset, shared-chain mixed-content full-composition subset, dual outer/hole duplicate-normalization composition subset, deformed-cube multi-face non-planar subsets via single-displaced-vertex and dual-displaced-vertices, near-equal shared-edge representative-average vertex-placement subset, support-mismatch near-equal shared-edge representative-average subset, support-mismatch near-equal shared-apex fan representative-average subset, support-mismatch near-equal shared-edge-chain representative-average subset, support-mismatch near-equal shared-corner fan representative-average subset, support-mismatch near-equal closed-tetra representative-average subset, support-mismatch near-equal shared-edge-chain with duplicate-normalization representative-average subset, support-mismatch near-equal shared-chain mixed-content duplicate-hole representative-average subset, support-mismatch near-equal shared-chain full-composition representative-average subset, and support-mismatch near-equal shared-chain dual-duplicate full-composition representative-average subset) + scale-aware tiny-loop normal fallback + mild/tiny-scale outer/hole/multi-face/mixed-content/shared-edge loop-projection + duplicate/collinear-leading normalization composition subset is still open.";
}
