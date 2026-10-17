#include <gtest/gtest.h>

#include <limits>

#include "Types/Geometry3d/SCDirection3d.h"

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
}
