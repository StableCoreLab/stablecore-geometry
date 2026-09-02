#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "Support/GeometryTestSupport.h"

TEST(GeometryTestSupportTest, ReportsCustomNearMismatchThroughGTest)
{
    EXPECT_NONFATAL_FAILURE(
        Geometry::Test::AssertNear(1.0, 2.0, 0.0, "actual", "expected", __FILE__, __LINE__), "value mismatch");
}
