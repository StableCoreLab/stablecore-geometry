#include <gtest/gtest.h>

#include <limits>

#include "Geometry2d/SCSegmentSearch2d.h"
#include "Geometry3d/SCSegmentSearch3d.h"

namespace
{
    template <typename Hit>
    void CheckSegmentSearchHitPayload()
    {
        Hit hit{};
        hit.distanceSquared = 4.0;
        hit.parameter = 0.5;
        ASSERT_TRUE(hit.IsValid());
        const double infinity = std::numeric_limits<double>::infinity();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (const double invalid : {-1.0, infinity, -infinity, nan})
        {
            hit.distanceSquared = invalid;
            EXPECT_FALSE(hit.IsValid());
        }
        hit.distanceSquared = 0.0;
        for (const double invalid : {infinity, -infinity, nan})
        {
            hit.parameter = invalid;
            EXPECT_FALSE(hit.IsValid());
        }
        // 参数域由具体曲线决定，有效性不强制限制到 [0, 1]。
        hit.parameter = -2.0;
        EXPECT_TRUE(hit.IsValid());
        hit.parameter = 5.0;
        EXPECT_TRUE(hit.IsValid());
        hit.point.x = nan;
        EXPECT_FALSE(hit.IsValid());
    }
}

TEST(IndexHitContractTest, SegmentSearch2dRejectsNonFinitePayloads)
{
    CheckSegmentSearchHitPayload<Geometry::SCSegmentSearchHit2d>();
}

TEST(IndexHitContractTest, SegmentSearch3dRejectsNonFinitePayloads)
{
    CheckSegmentSearchHitPayload<Geometry::SCSegmentSearchHit3d>();
}
