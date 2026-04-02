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
    GTEST_SKIP() << "Known 3D gap: high-fidelity feature-preserving Brep->mesh conversion is still open.";
}

TEST(Conversion3dGapTest, GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen)
{
    const PolyhedronBody body;
    const auto mesh = ConvertToTriangleMesh(body);
    (void)mesh;
    GTEST_SKIP() << "Known 3D gap: robust non-planar polyhedron->Brep repair beyond affine-planar + support-plane-refit subset is still open.";
}
