#include <gtest/gtest.h>

#include <limits>

#include "Core/Relation.h"
#include "Support/StableNumerics.h"
#include "Support/Geometry2d/Predicate2.h"

namespace
{
    TEST(StableNumericsTest, StableParametersAndDimensionalThresholdsKeepExtremeScales)
    {
        using namespace Geometry::Detail;
        const double maximum=std::numeric_limits<double>::max();
        EXPECT_FALSE(StableSpan(-maximum,maximum));
        ASSERT_TRUE(StableMidpoint(maximum,maximum));
        EXPECT_DOUBLE_EQ(*StableMidpoint(maximum,maximum),maximum);
        EXPECT_DOUBLE_EQ(*StableLerp(-maximum,maximum,0.5),0);
        EXPECT_DOUBLE_EQ(*StableNormalizedParameter(-maximum,maximum,0),0.5);
        EXPECT_DOUBLE_EQ(*StableNormalizedParameter(2,2,2),0);
        EXPECT_FALSE(StableAdvance(maximum,maximum));
        EXPECT_FALSE(StableContains(0,1,std::numeric_limits<double>::infinity(),maximum));
        EXPECT_FALSE(*MagnitudeWithinPower(1e-300,1e-200,2));
        EXPECT_TRUE(*MagnitudeWithinPower(1e300,1e200,2));
        EXPECT_FALSE(*MagnitudeWithinPower(1e-300,0,3));
        EXPECT_TRUE(*MagnitudeWithinPower(0,0,3));
        EXPECT_FALSE(Geometry::IsZero(Geometry::SCVector2d{1e-200,0},0));
        EXPECT_FALSE(Geometry::IsZero(Geometry::SCVector2d{1e200,1e200},1e200));
        EXPECT_TRUE(Geometry::IsZero(Geometry::SCVector2d{1e-200,0},1e-200));
        EXPECT_TRUE(Geometry::IsParallel({1e200,0,0},{-1e200,0,0}));
        EXPECT_FALSE(Geometry::IsParallel({1e200,0,0},{0,1e200,0}));
        EXPECT_TRUE(Geometry::IsPerpendicular({1e-200,0,0},{0,1e-200,0}));
        EXPECT_FALSE(Geometry::IsPerpendicular({1e200,0,0},{1e200,0,0}));
    }
}
