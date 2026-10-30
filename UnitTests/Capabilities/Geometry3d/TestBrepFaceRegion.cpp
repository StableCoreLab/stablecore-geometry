#include <gtest/gtest.h>

#include <memory>

#include "../../../Source/Core/Detail/BrepFaceRegion.h"

TEST(BrepFaceRegionTest, SmallUvTrimRespectsCallerParameterTolerance)
{
    constexpr double width = 1e-10;
    Geometry::SCGeometryTolerance3d tolerance{};
    tolerance.distanceEpsilon = 1e-15;
    tolerance.parameterEpsilon = 1e-15;

    const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlaneSurface::FromPlane(
            Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
    const Geometry::SCPolyline2d uvRing(
        {{0.0, 0.0}, {width, 0.0}, {width, width}, {0.0, width}},
        Geometry::SCPolylineClosure::Closed);
    ASSERT_FALSE(uvRing.IsValid());
    ASSERT_TRUE(uvRing.IsValid(tolerance.parameterEpsilon));

    const Geometry::SCCurveOnSurface trim(surface, uvRing);
    ASSERT_TRUE(trim.IsValid(tolerance));
    const Geometry::SCBrepFace face(surface, Geometry::SCBrepLoop{}, {}, trim);
    Geometry::SCPolygon2d polygon;
    ASSERT_TRUE(Geometry::Detail::TryBuildPolygonFromBrepFaceTrims(face, tolerance, polygon));
    EXPECT_TRUE(polygon.IsValid(tolerance.parameterEpsilon));
    EXPECT_NEAR(polygon.Area(tolerance.parameterEpsilon), width * width, 1e-25);

    const Geometry::SCBox3d bounds = face.Bounds();
    ASSERT_TRUE(bounds.IsValid());
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, tolerance.distanceEpsilon);
    EXPECT_NEAR(bounds.MaxPoint().x, width, tolerance.distanceEpsilon);
}
