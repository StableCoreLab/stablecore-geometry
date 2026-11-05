#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "Support/GeometryTestSupport.h"

TEST(GeometryTestSupportTest, ReportsCustomNearMismatchThroughGTest)
{
    EXPECT_NONFATAL_FAILURE(
        Geometry::Test::AssertNear(1.0, 2.0, 0.0, "actual", "expected", __FILE__, __LINE__), "value mismatch");
}

TEST(GeometryTestSupportTest, CustomMacrosReportEachFailureAndContinue)
{
    ::testing::TestPartResultArray failures;
    bool continued = false;
    int firstLine = 0;
    int secondLine = 0;
    {
        ::testing::ScopedFakeTestPartResultReporter reporter(
            ::testing::ScopedFakeTestPartResultReporter::INTERCEPT_ONLY_CURRENT_THREAD, &failures);
        firstLine = __LINE__ + 1;
        GEOMETRY_TEST_ASSERT_NEAR(1.0, 2.0, 0.0);
        secondLine = __LINE__ + 1;
        GEOMETRY_TEST_ASSERT_NEAR(3.0, 4.0, 0.0);
        continued = true;
    }
    EXPECT_TRUE(continued);
    ASSERT_EQ(failures.size(), 2);
    for (int i = 0; i < failures.size(); ++i)
    {
        const auto& failure = failures.GetTestPartResult(i);
        EXPECT_EQ(failure.type(), ::testing::TestPartResult::kNonFatalFailure);
        EXPECT_STREQ(failure.file_name(), __FILE__);
        EXPECT_EQ(failure.line_number(), i == 0 ? firstLine : secondLine);
    }
}
