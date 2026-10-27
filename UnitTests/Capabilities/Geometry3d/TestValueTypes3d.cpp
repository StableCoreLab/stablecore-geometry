#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCPlane.h"
#include "Types/Geometry3d/SCRay3d.h"
#include "Types/Geometry3d/SCTriangle3d.h"

namespace
{
    static_assert(Geometry::SCIntervald{1.0, 2.0}.Contains(1.5, 0.0));

    TEST(PointVector3dCapabilityTest, AlmostEqualsRejectsInvalidToleranceAndCoordinates)
    {
        const Geometry::SCPoint3d point{1.0, 2.0, 3.0};
        const Geometry::SCVector3d vector{1.0, 2.0, 3.0};
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_TRUE(point.AlmostEquals(vector, 0.0));
        EXPECT_FALSE(point.AlmostEquals(vector, -1.0));
        EXPECT_FALSE(point.AlmostEquals(vector, infinity));
        EXPECT_FALSE(point.AlmostEquals(Geometry::SCPoint3d{infinity, 2.0, 3.0}, infinity));
    }

    TEST(PointVector3dCapabilityTest, LengthAndDirectionRemainStableAtExtremeScales)
    {
        const Geometry::SCVector3d tiny{1e-200, 0.0, 0.0};
        EXPECT_NEAR(tiny.Length(), 1e-200, 1e-210);
        EXPECT_EQ(tiny.Normalized(1e-210), (Geometry::SCVector3d{1.0, 0.0, 0.0}));
        EXPECT_TRUE(Geometry::SCDirection3d::FromVector(tiny, 1e-210).IsValid());

        const Geometry::SCVector3d large{1e200, 0.0, 0.0};
        EXPECT_NEAR(large.Length(), 1e200, 1e190);
        EXPECT_EQ(large.Normalized(), (Geometry::SCVector3d{1.0, 0.0, 0.0}));

        const double infinity = std::numeric_limits<double>::infinity();
        EXPECT_EQ(Geometry::SCVector3d{}.Normalized(-1.0), Geometry::SCVector3d{});
        EXPECT_EQ(tiny.Normalized(infinity), Geometry::SCVector3d{});
        EXPECT_EQ((Geometry::SCVector3d{infinity, 0.0, 0.0}.Normalized(0.0)), Geometry::SCVector3d{});
    }

    TEST(Box3dCapabilityTest, AlmostEqualsRejectsInvalidToleranceForEmptyBoxes)
    {
        const Geometry::SCBox3d empty;
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_TRUE(empty.AlmostEquals(empty, 0.0));
        EXPECT_FALSE(empty.AlmostEquals(empty, -1.0));
        EXPECT_FALSE(empty.AlmostEquals(empty, infinity));
    }

    TEST(Direction3dCapabilityTest, AlmostEqualsUsesAngularToleranceAtSmallAngles)
    {
        const auto reference = Geometry::SCDirection3d::FromVector({1.0, 0.0, 0.0});
        const auto near = Geometry::SCDirection3d::FromVector({1.0, 5e-10, 0.0});
        const auto far = Geometry::SCDirection3d::FromVector({1.0, 2e-9, 0.0});

        EXPECT_TRUE(reference.AlmostEquals(near, 1e-9));
        EXPECT_FALSE(reference.AlmostEquals(far, 1e-9));
        EXPECT_FALSE(reference.AlmostEquals(reference, -1.0));
        EXPECT_FALSE(reference.AlmostEquals(reference, std::numeric_limits<double>::quiet_NaN()));
        EXPECT_TRUE(reference.AlmostEquals(reference.Reverse(), 4.0));
    }

    TEST(Direction3dCapabilityTest, RejectsInvalidDistanceTolerance)
    {
        const Geometry::SCDirection3d empty;
        EXPECT_FALSE(empty.IsValid(-1.0));
        EXPECT_FALSE(empty.IsValid(std::numeric_limits<double>::infinity()));
        EXPECT_FALSE(Geometry::SCDirection3d::FromVector({}, -1.0).IsValid(-1.0));

        const auto unit = Geometry::SCDirection3d::FromVector({1.0, 0.0, 0.0});
        EXPECT_FALSE(unit.IsValid(-1.0));
        EXPECT_FALSE(unit.IsValid(std::numeric_limits<double>::quiet_NaN()));
    }

    TEST(Ray3dCapabilityTest, ValidityRequiresFiniteNonnegativeTolerance)
    {
        const Geometry::SCRay3d empty;
        EXPECT_FALSE(empty.IsValid(-1.0));

        const auto unit = Geometry::SCDirection3d::FromVector({1.0, 0.0, 0.0});
        const auto ray = Geometry::SCRay3d::FromOriginAndDirection({1.0, 2.0, 3.0}, unit);
        EXPECT_TRUE(ray.IsValid(0.0));
        EXPECT_EQ(ray.PointAt(2.0), (Geometry::SCPoint3d{3.0, 2.0, 3.0}));
        EXPECT_FALSE(ray.IsValid(-1.0));
        EXPECT_FALSE(ray.IsValid(std::numeric_limits<double>::infinity()));
        EXPECT_FALSE(ray.IsValid(std::numeric_limits<double>::quiet_NaN()));
        const Geometry::SCRay3d invalidOrigin{{std::numeric_limits<double>::infinity(), 0.0, 0.0}, unit};
        EXPECT_FALSE(invalidOrigin.IsValid());
    }

    TEST(Interval3dCapabilityTest, InvalidIntervalsCannotContainOrMatch)
    {
        const Geometry::SCIntervald interval{1.0, 2.0};
        EXPECT_TRUE(interval.Contains(1.0, 0.0));
        EXPECT_TRUE(interval.Contains(2.0, 0.0));
        EXPECT_FALSE(interval.Contains(0.5, 0.0));
        EXPECT_TRUE(interval.Contains(0.5, 0.5));
        EXPECT_FALSE(interval.Contains(1.5, -1.0));
        EXPECT_FALSE(interval.Contains(1.5, std::numeric_limits<double>::infinity()));
        EXPECT_FALSE(interval.Contains(std::numeric_limits<double>::infinity(), 0.0));
        const Geometry::SCIntervald pointInterval{1.0, 1.0};
        const Geometry::SCIntervald nonFinite{1.0, std::numeric_limits<double>::infinity()};
        EXPECT_TRUE(pointInterval.Contains(1.0, 0.0));
        EXPECT_FALSE(nonFinite.IsValid());

        const Geometry::SCIntervald reversed{2.0, 1.0};
        EXPECT_FALSE(reversed.IsValid());
        EXPECT_FALSE(reversed.Contains(1.5, 1.0));
        EXPECT_FALSE(reversed.AlmostEquals(reversed, 0.0));
        EXPECT_FALSE(interval.AlmostEquals(interval, -1.0));
        EXPECT_FALSE(interval.AlmostEquals(interval, std::numeric_limits<double>::quiet_NaN()));
        EXPECT_TRUE(interval.AlmostEquals({1.0, 2.0 + 1e-10}, 1e-9));
    }

    TEST(Primitive3dCapabilityTest, InvalidToleranceCannotValidateDegenerateGeometry)
    {
        const Geometry::SCLine3d line;
        const Geometry::SCLineSegment3d segment;
        const Geometry::SCPlane plane;
        const Geometry::SCTriangle3d collinear{{0.0, 0.0, 0.0},
                                                {1.0, 0.0, 0.0},
                                                {2.0, 0.0, 0.0}};
        const Geometry::TriangleMesh mesh({collinear.a, collinear.b, collinear.c}, {{{0, 1, 2}}});

        EXPECT_FALSE(line.IsValid(-1.0));
        EXPECT_FALSE(segment.IsValid(-1.0));
        EXPECT_FALSE(plane.IsValid(-1.0));
        EXPECT_TRUE(collinear.IsDegenerate(-1.0));
        EXPECT_FALSE(collinear.IsValid(-1.0));
        EXPECT_FALSE(mesh.IsValid(-1.0));

        const Geometry::SCTriangle3d valid{{0.0, 0.0, 0.0},
                                            {1.0, 0.0, 0.0},
                                            {0.0, 1.0, 0.0}};
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const Geometry::TriangleMesh validMesh({valid.a, valid.b, valid.c}, {{{0, 1, 2}}});
        EXPECT_TRUE(valid.IsValid(0.0));
        EXPECT_FALSE(valid.IsValid(nan));
        EXPECT_FALSE(validMesh.IsValid(nan));
        EXPECT_FALSE(Geometry::TriangleMesh{}.IsValid(nan));
    }

    TEST(Triangle3dCapabilityTest, DegeneracyUsesLinearHeightToleranceAtSmallScale)
    {
        const Geometry::SCTriangle3d small{{0.0, 0.0, 0.0},
                                           {1e-5, 0.0, 0.0},
                                           {0.0, 1e-5, 0.0}};
        EXPECT_FALSE(small.IsDegenerate());
        EXPECT_TRUE(small.IsValid());
        EXPECT_TRUE(Geometry::TriangleMesh({small.a, small.b, small.c}, {{{0, 1, 2}}}).IsValid());

        const Geometry::SCTriangle3d thin{{0.0, 0.0, 0.0},
                                          {1.0, 0.0, 0.0},
                                          {0.5, 5e-10, 0.0}};
        EXPECT_TRUE(thin.IsDegenerate());
        EXPECT_FALSE(thin.IsDegenerate(1e-12));
    }

    TEST(PlaneSurface3dCapabilityTest, AxisOrthogonalityUsesAngularToleranceInRadians)
    {
        const auto plane = Geometry::SCPlane::FromPointAndNormal({}, {0.0, 0.0, 1.0});
        const Geometry::SCPlaneSurface surface(
            plane, {1.0, 0.0, 0.0}, {std::sin(1.0), std::cos(1.0), 0.0},
            {0.0, 1.0}, {0.0, 1.0});
        Geometry::SCGeometryTolerance3d tolerance{};
        tolerance.angleEpsilon = 0.9;
        EXPECT_FALSE(surface.IsValid(tolerance));
        tolerance.angleEpsilon = 1.1;
        EXPECT_TRUE(surface.IsValid(tolerance));
    }
}
