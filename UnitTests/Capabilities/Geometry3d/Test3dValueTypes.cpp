#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCTriangle3d.h"

namespace
{
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

    TEST(PlaneSurfaceCapabilityTest, AxisOrthogonalityUsesAngularToleranceInRadians)
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
