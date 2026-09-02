#include <gtest/gtest.h>

#include "Support/Epsilon.h"

#ifndef GEOMETRY_USE_DLL
#error "Consumers of SCGeometry must compile with GEOMETRY_USE_DLL."
#endif

namespace
{
    TEST(DllExportTest, ImportsExportedEpsilonData)
    {
        EXPECT_NEAR(Geometry::kPi, 3.14159265358979323846, 1e-15);
        ASSERT_EQ(Geometry::kArcBoundsCriticalAngles.size(), 4U);
        EXPECT_NEAR(Geometry::kArcBoundsCriticalAngles[0], 0.0, 1e-15);
    }
}  // namespace
