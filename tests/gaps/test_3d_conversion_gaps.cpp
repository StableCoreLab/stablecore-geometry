#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::BrepBody;
using geometry::sdk::ConvertToBrepBody;
using geometry::sdk::ConvertToTriangleMesh;
using geometry::sdk::Plane;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::Vector3d;

namespace
{
PolyhedronBody BuildQuadSharedEdgeChainWithSupportMismatchAndDuplicateBody()
{
    // Three quads in a strip: A-B-C.
    // A and B share edge (b,c); B and C share edge (e,f).
    // The support planes are intentionally mismatched and each face has tiny
    // non-planar drift. The middle face also contains a duplicate leading
    // shared vertex, so repair normalization can change loop cardinality and
    // break one-to-one source representative mapping used by current reuse.
    const double s = 1e-5;
    const Point3d a{0.0, 0.0, 0.0};
    const Point3d b{s, 0.0, 0.0};
    const Point3d c{s, s, 0.0};
    const Point3d d{0.0, s, 1.5e-6};
    const Point3d e{2.0 * s, 0.0, 1.1e-6};
    const Point3d f{2.0 * s, s, 0.0};
    const Point3d g{3.0 * s, 0.0, 1.3e-6};
    const Point3d h{3.0 * s, s, 0.0};

    const Plane mismatchedPlane =
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 4e-6}, Vector3d{0.0, 0.0, 1.0});

    return PolyhedronBody({
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({a, b, c, d})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({b, b, e, f, c})),
        PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}))});
}
} // namespace

TEST(Conversion3dGapTest, HighFidelityBrepToMeshFeaturePreservationRemainsOpen)
{
    const BrepBody body({});
    const auto converted = ConvertToTriangleMesh(body);
    (void)converted;
    GTEST_SKIP() << "Known 3D gap: high-fidelity feature-preserving Brep->mesh conversion beyond planar holed+multi-face area-preserving subset and shared-edge global vertex-reuse subset is still open.";
}

TEST(Conversion3dGapTest, GeneralNonPlanarPolyhedronToBrepRepairRemainsOpen)
{
    const PolyhedronBody body;
    const auto mesh = ConvertToTriangleMesh(body);
    (void)mesh;
    GTEST_SKIP() << "Known 3D gap: robust non-planar polyhedron->Brep repair beyond affine-planar + support-plane-refit (including closed-shell shared-topology subset via triangular faces [closed tetrahedron], shared-chain mixed-content full-composition subset, dual outer/hole duplicate-normalization composition subset) + scale-aware tiny-loop normal fallback + mild/tiny-scale outer/hole/multi-face/mixed-content/shared-edge loop-projection + duplicate/collinear-leading normalization composition subset is still open. Specifically: shared-edge vertex consistency constraint (quad faces whose shared vertices are NOT defining vertices of both adjacent face refit-planes get independently projected, yielding O(z_drift)-level divergence that exceeds the default merge tolerance) is the next open frontier.";
}

TEST(Conversion3dGapTest, QuadSharedEdgeChainWithNormalizationVertexConsistencyRemainsOpen)
{
    const PolyhedronBody body = BuildQuadSharedEdgeChainWithSupportMismatchAndDuplicateBody();
    const auto result = ConvertToBrepBody(body);
    (void)result;

    GTEST_SKIP() << "Known 3D gap: quad-face shared-edge chain with support-plane mismatch plus duplicate-loop normalization still lacks robust cross-face vertex consistency closure. Representative-id reuse helps when loop cardinality is preserved, but normalization-induced cardinality changes can still require explicit snapping/constraint reprojection.";
}
