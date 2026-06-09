#include <gtest/gtest.h>
#include <cmath>

#include "Support/GeometryTestSupport.h"
#include "Types/Geometry2d/SCBox2.h"

using Geometry::SCBox2d;
using Geometry::SCBox2i;
using Geometry::SCPoint2d;
using Geometry::SCPoint2i;

TEST(BoxTest, CoversCurrentCapabilities)
{
    SCBox2i emptyBox;
    ASSERT_FALSE(emptyBox.IsValid());

    const SCBox2i invalidBox(SCPoint2i(3, 4), SCPoint2i(1, 2));
    ASSERT_FALSE(invalidBox.IsValid());

    const SCBox2i pointBox(SCPoint2i(2, 3), SCPoint2i(2, 3));
    ASSERT_TRUE(pointBox.IsValid());
    ASSERT_EQ(SCBox2i::FromMinMax(SCPoint2i(2, 3), SCPoint2i(2, 3)), pointBox);
    ASSERT_EQ(pointBox.Width(), 0.0);
    ASSERT_EQ(pointBox.Height(), 0.0);
    ASSERT_EQ(pointBox.Area(), 0.0);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(pointBox.Center(), SCPoint2d(2.0, 3.0), 1e-12);

    const SCBox2i boxA(SCPoint2i(1, 2), SCPoint2i(4, 6));
    ASSERT_TRUE(boxA.IsValid());
    ASSERT_EQ(boxA.Width(), 3.0);
    ASSERT_EQ(boxA.Height(), 4.0);
    ASSERT_EQ(boxA.Area(), 12.0);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), SCPoint2d(2.5, 4.0), 1e-12);

    SCBox2i expanded;
    expanded.ExpandToInclude(SCPoint2i(5, 7));
    ASSERT_TRUE(expanded.IsValid());
    ASSERT_EQ(expanded.MinPoint(), SCPoint2i(5, 7));
    ASSERT_EQ(expanded.MaxPoint(), SCPoint2i(5, 7));

    expanded.ExpandToInclude(SCPoint2i(2, 9));
    ASSERT_EQ(expanded.MinPoint(), SCPoint2i(2, 7));
    ASSERT_EQ(expanded.MaxPoint(), SCPoint2i(5, 9));

    expanded.ExpandToInclude(SCBox2i(SCPoint2i(0, 1), SCPoint2i(3, 8)));
    ASSERT_EQ(expanded.MinPoint(), SCPoint2i(0, 1));
    ASSERT_EQ(expanded.MaxPoint(), SCPoint2i(5, 9));

    const SCBox2i beforeIgnored = expanded;
    expanded.ExpandToInclude(SCBox2i(SCPoint2i(10, 10), SCPoint2i(4, 12)));
    ASSERT_EQ(expanded, beforeIgnored);

    SCBox2d floatingBox;
    floatingBox.ExpandToInclude(SCPoint2d(1.5, 2.5));
    floatingBox.ExpandToInclude(SCPoint2d(-0.5, 3.0));
    ASSERT_TRUE(floatingBox.IsValid());
    ASSERT_LT(std::abs(floatingBox.Width() - 2.0), 1e-12);
    ASSERT_LT(std::abs(floatingBox.Height() - 0.5), 1e-12);
    ASSERT_LT(std::abs(floatingBox.Area() - 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(floatingBox.Center(), SCPoint2d(0.5, 2.75), 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(floatingBox, SCBox2d::FromMinMax(SCPoint2d(-0.5, 2.5), SCPoint2d(1.5, 3.0)), 1e-12);

    const SCBox2i sameA(SCPoint2i(1, 2), SCPoint2i(3, 4));
    const SCBox2i sameB(SCPoint2i(1, 2), SCPoint2i(3, 4));
    const SCBox2i different(SCPoint2i(1, 2), SCPoint2i(3, 5));
    ASSERT_EQ(sameA, sameB);
    ASSERT_NE(sameA, different);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(sameA, sameB, 0.0);
}
