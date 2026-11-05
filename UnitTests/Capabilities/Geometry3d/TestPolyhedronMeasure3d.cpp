#include <gtest/gtest.h>

#include "Geometry.h"

using namespace Geometry;

TEST(Geometry3dTest, PolyhedronFaceAreaUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-15;
    const PolyhedronFace3d face(
        SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}),
        PolyhedronLoop3d({{0, 0, 0}, {side, 0, 0}, {side, side, 0}, {0, side, 0}}));
    ASSERT_TRUE(face.IsValid(tolerance.distanceEpsilon));
    const auto projection = ProjectFaceToPolygon2d(face, tolerance);
    ASSERT_TRUE(projection.success);
    ASSERT_TRUE(projection.polygon.IsValid(tolerance.distanceEpsilon));
    ASSERT_FALSE(projection.polygon.IsValid());
    EXPECT_NEAR(Area(face, tolerance), side * side, 1e-30);
}

TEST(Geometry3dTest, PolyhedronBodyVolumeUsesCallerTolerance)
{
    constexpr double side = 1e-10;
    const SCPoint3d a{0, 0, 0};
    const SCPoint3d b{side, 0, 0};
    const SCPoint3d c{0, side, 0};
    const SCPoint3d d{0, 0, side};
    const PolyhedronBody body({
        PolyhedronFace3d(SCPlane::FromPointAndNormal(a, {0, 0, -1}), PolyhedronLoop3d({a, c, b})),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(a, {0, -1, 0}), PolyhedronLoop3d({a, b, d})),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(b, {1, 1, 1}), PolyhedronLoop3d({b, c, d})),
        PolyhedronFace3d(SCPlane::FromPointAndNormal(a, {-1, 0, 0}), PolyhedronLoop3d({c, a, d})),
    });
    SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-15;
    ASSERT_TRUE(body.IsValid(tolerance));
    const auto conversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
    ASSERT_TRUE(conversion.success);
    ASSERT_TRUE(conversion.mesh.IsValid(tolerance.distanceEpsilon));
    ASSERT_FALSE(conversion.mesh.IsValid());
    EXPECT_FALSE(IsClosedTriangleMesh(conversion.mesh));
    EXPECT_TRUE(IsClosedTriangleMesh(conversion.mesh, tolerance.distanceEpsilon));
    EXPECT_TRUE(ExtractBoundaryEdges(conversion.mesh, tolerance.distanceEpsilon).empty());
    EXPECT_NEAR(Volume(conversion.mesh, tolerance.distanceEpsilon), side * side * side / 6.0, 1e-40);
    EXPECT_NEAR(Volume(body, tolerance), side * side * side / 6.0, 1e-40);
}
