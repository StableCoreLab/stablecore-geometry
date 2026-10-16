#include <gtest/gtest.h>

#include "Support/Epsilon.h"

#if defined(SCGEOMETRY_EXPECT_SHARED) != defined(GEOMETRY_USE_DLL)
#error "SCGeometry consumer DLL import definition does not match the library type."
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
