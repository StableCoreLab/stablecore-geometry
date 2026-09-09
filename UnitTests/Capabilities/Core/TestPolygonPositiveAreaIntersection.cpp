#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry.h"

using Geometry::kDefaultEpsilon;
using Geometry::QueryPolygonPositiveAreaIntersection;
using Geometry::SCArcSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolygonPositiveAreaIntersectionFailure2d;
using Geometry::SCPolygonPositiveAreaIntersectionResult2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ISCSegment2d;
using Geometry::kPi;

using namespace Geometry;

namespace
{
    [[nodiscard]] SCPolygon2d MakeSquare(double minX, double minY, double maxX, double maxY)
    {
        return SCPolygon2d(SCPolyline2d({{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}},
                                        SCPolylineClosure::Closed));
    }

    [[nodiscard]] SCPolygon2d MakeCircle(const SCPoint2d& center, double radius)
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.push_back(std::make_shared<SCArcSegment2d>(center, radius, 0.0, kPi));
        segments.push_back(std::make_shared<SCArcSegment2d>(center, radius, kPi, kPi));
        return SCPolygon2d(SCPolyline2d(std::move(segments), SCPolylineClosure::Closed));
    }

    // 所有成功结果断言 failure == None；所有失败结果断言
    // hasPositiveAreaIntersection == false && failure != None。
    void AssertSuccessFalse(const SCPolygonPositiveAreaIntersectionResult2d& result)
    {
        ASSERT_TRUE(result.success);
        EXPECT_FALSE(result.hasPositiveAreaIntersection);
        EXPECT_EQ(result.failure, SCPolygonPositiveAreaIntersectionFailure2d::None);
    }

    void AssertSuccessTrue(const SCPolygonPositiveAreaIntersectionResult2d& result)
    {
        ASSERT_TRUE(result.success);
        EXPECT_TRUE(result.hasPositiveAreaIntersection);
        EXPECT_EQ(result.failure, SCPolygonPositiveAreaIntersectionFailure2d::None);
    }

    void AssertFailure(const SCPolygonPositiveAreaIntersectionResult2d& result,
                       SCPolygonPositiveAreaIntersectionFailure2d expected)
    {
        EXPECT_FALSE(result.success);
        EXPECT_FALSE(result.hasPositiveAreaIntersection);
        EXPECT_EQ(result.failure, expected);
    }
}  // namespace

// 不相交：两个完全分离的正方形面积集合无公共内部区域，成功且为 false。
TEST(PolygonPositiveAreaIntersectionTest, DisjointSquaresReturnFalse)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(20.0, 20.0, 30.0, 30.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessFalse(result);
}

// 角接触：两正方形仅在一个角点接触，无正面积公共区域，成功且为 false。
TEST(PolygonPositiveAreaIntersectionTest, CornerTouchReturnsFalse)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(10.0, 10.0, 20.0, 20.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessFalse(result);
}

// 共边接触：两正方形沿一条完整边接触，无正面积公共区域，成功且为 false。
TEST(PolygonPositiveAreaIntersectionTest, EdgeTouchReturnsFalse)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(10.0, 0.0, 20.0, 10.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessFalse(result);
}

// 部分重叠：两正方形部分面积重叠，存在正面积公共区域，成功且为 true。
TEST(PolygonPositiveAreaIntersectionTest, PartialOverlapReturnsTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(5.0, 5.0, 15.0, 15.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 双向包含：first 严格包含 second，存在正面积公共区域，成功且为 true。
TEST(PolygonPositiveAreaIntersectionTest, FirstContainsSecondReturnsTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 20.0, 20.0);
    const auto second = MakeSquare(2.0, 2.0, 8.0, 8.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 双向包含：second 严格包含 first，存在正面积公共区域，成功且为 true。
TEST(PolygonPositiveAreaIntersectionTest, SecondContainsFirstReturnsTrue)
{
    const auto first = MakeSquare(2.0, 2.0, 8.0, 8.0);
    const auto second = MakeSquare(0.0, 0.0, 20.0, 20.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 非严格包含且接触：first 包含 second 且 second 部分边界与 first 边界重合，
// 仍存在正面积公共区域，成功且为 true。
TEST(PolygonPositiveAreaIntersectionTest, NonStrictContainmentWithTouchReturnsTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    // second 与 first 在左侧和底部边重合，但仍有正面积公共区域。
    const auto second = MakeSquare(0.0, 0.0, 5.0, 5.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 完全相等：两正方形完全相同，填充集合完全相等，成功且为 true。
TEST(PolygonPositiveAreaIntersectionTest, EqualPolygonsReturnTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 交换对称性：交换 first 和 second 后 success、failure 与 hasPositiveAreaIntersection 均不变。
TEST(PolygonPositiveAreaIntersectionTest, SymmetricUnderSwap)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(5.0, 5.0, 15.0, 15.0);
    const auto ab = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    const auto ba = QueryPolygonPositiveAreaIntersection(second, first, kDefaultEpsilon);
    EXPECT_EQ(ab.success, ba.success);
    EXPECT_EQ(ab.hasPositiveAreaIntersection, ba.hasPositiveAreaIntersection);
    EXPECT_EQ(ab.failure, ba.failure);
}

// 孔洞负空间：first 在与 second 重叠区域处有孔洞，孔洞内不构成正面积公共区域。
// 当 second 完全落在 first 的孔洞内时，填充集合不相交，成功且为 false。
TEST(PolygonPositiveAreaIntersectionTest, HoleExcludesPositiveArea)
{
    // first 为带孔洞的正方形，外环 [0,20]×[0,20]，洞 [5,15]×[5,15]。
    const auto first = SCPolygon2d(
        MakeSquare(0.0, 0.0, 20.0, 20.0).OuterRing(),
        {MakeSquare(5.0, 5.0, 15.0, 15.0).OuterRing()});
    // second 完全位于 first 的洞内，填充集合不相交。
    const auto second = MakeSquare(7.0, 7.0, 13.0, 13.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessFalse(result);
}

// 孔洞与外环部分重叠：second 部分位于孔洞、部分位于填充区域，仍存在正面积公共区域。
TEST(PolygonPositiveAreaIntersectionTest, HolePartialOverlapReturnsTrue)
{
    // first 外环 [0,20]×[0,20]，洞 [5,15]×[5,15]。
    const auto first = SCPolygon2d(
        MakeSquare(0.0, 0.0, 20.0, 20.0).OuterRing(),
        {MakeSquare(5.0, 5.0, 15.0, 15.0).OuterRing()});
    // second 横跨洞与填充区域。
    const auto second = MakeSquare(2.0, 2.0, 8.0, 8.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 小于 eps 的狭窄正面积交集必须返回 true 或 NumericalIndeterminate，不得成功返回 false。
// 重叠区域宽 0.25 (=2.5*eps)、高 1.0，面积 0.25 > eps^2=0.01。
// 由于面宽接近 eps，代表点 inward offset 可能越出面外，
// 返回 true、NumericalIndeterminate 或 FaceClassificationFailure 均合法。
TEST(PolygonPositiveAreaIntersectionTest, NarrowIntersectionReturnsTrueOrIndeterminate)
{
    const auto first = MakeSquare(0.0, 0.0, 1.0, 1.0);
    const auto second = MakeSquare(0.75, 0.0, 1.75, 1.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, 0.1);
    if (result.success)
    {
        EXPECT_TRUE(result.hasPositiveAreaIntersection);
        EXPECT_EQ(result.failure, SCPolygonPositiveAreaIntersectionFailure2d::None);
    }
    else
    {
        EXPECT_FALSE(result.hasPositiveAreaIntersection);
        EXPECT_TRUE(result.failure == SCPolygonPositiveAreaIntersectionFailure2d::NumericalIndeterminate ||
                    result.failure == SCPolygonPositiveAreaIntersectionFailure2d::FaceClassificationFailure);
    }
}

// 无效输入：无效多边形返回 InvalidInput。
TEST(PolygonPositiveAreaIntersectionTest, InvalidPolygonReturnsInvalidInput)
{
    const SCPolygon2d invalid;
    const auto valid = MakeSquare(0.0, 0.0, 10.0, 10.0);
    AssertFailure(QueryPolygonPositiveAreaIntersection(invalid, valid, kDefaultEpsilon),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
    AssertFailure(QueryPolygonPositiveAreaIntersection(valid, invalid, kDefaultEpsilon),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
}

// 无效容差：eps 非有限或非正返回 InvalidInput。
TEST(PolygonPositiveAreaIntersectionTest, InvalidToleranceReturnsInvalidInput)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = MakeSquare(5.0, 5.0, 15.0, 15.0);
    AssertFailure(QueryPolygonPositiveAreaIntersection(first, second, 0.0),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
    AssertFailure(QueryPolygonPositiveAreaIntersection(first, second, -1e-9),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
    AssertFailure(QueryPolygonPositiveAreaIntersection(first, second,
                                                       std::numeric_limits<double>::quiet_NaN()),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
    AssertFailure(QueryPolygonPositiveAreaIntersection(first, second,
                                                       std::numeric_limits<double>::infinity()),
                  SCPolygonPositiveAreaIntersectionFailure2d::InvalidInput);
}

// 外环起始段轮换：同一填充集合外环从不同顶点开始，仍返回 true。
TEST(PolygonPositiveAreaIntersectionTest, RotatedOuterRingStartReturnsTrue)
{
    // 原始正方形 [0,10]×[0,10]，从 (0,0) 起始。
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    // 轮换起始顶点：从 (10,0) 起始。
    const auto second = SCPolygon2d(SCPolyline2d({{10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}},
                                                 SCPolylineClosure::Closed));
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 整体反向：外环方向反向，规范化后仍表示同一填充集合，返回 true。
TEST(PolygonPositiveAreaIntersectionTest, ReversedOuterRingReturnsTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    // 反向外环：从 (0,0)→(0,10)→(10,10)→(10,0)。
    const auto second = SCPolygon2d(SCPolyline2d({{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}},
                                                 SCPolylineClosure::Closed));
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 等价分段不同：同一几何边界但用不同分段方式（两段折线 vs 四段折线），仍返回 true。
TEST(PolygonPositiveAreaIntersectionTest, EquivalentSubdivisionReturnsTrue)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    // 第二个多边形用 8 个顶点表示同一正方形边界（每边插入一个中点）。
    const auto second = SCPolygon2d(SCPolyline2d({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {10.0, 5.0},
                                                   {10.0, 10.0}, {5.0, 10.0}, {0.0, 10.0}, {0.0, 5.0}},
                                                  SCPolylineClosure::Closed));
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 圆弧边界多边形部分重叠：两圆形多边形部分重叠，返回 true。
TEST(PolygonPositiveAreaIntersectionTest, CircularPolygonsPartialOverlapReturnTrue)
{
    const auto first = MakeCircle({0.0, 0.0}, 5.0);
    const auto second = MakeCircle({3.0, 0.0}, 5.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 圆弧边界多边形不相交：两圆形多边形完全分离，返回 false。
TEST(PolygonPositiveAreaIntersectionTest, CircularPolygonsDisjointReturnFalse)
{
    const auto first = MakeCircle({0.0, 0.0}, 2.0);
    const auto second = MakeCircle({10.0, 0.0}, 2.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessFalse(result);
}

// 圆弧边界包围盒重叠但圆周严格包含：段对无边界交点时仍必须完成正面积判断。
TEST(PolygonPositiveAreaIntersectionTest, CircularPolygonContainmentReturnsTrue)
{
    const auto first = MakeCircle({0.0, 0.0}, 5.0);
    const auto second = MakeCircle({0.0, 0.0}, 2.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

TEST(PolygonPositiveAreaIntersectionTest, EqualCircularPolygonsReturnTrue)
{
    const auto first = MakeCircle({0.0, 0.0}, 5.0);
    const auto second = MakeCircle({0.0, 0.0}, 5.0);
    AssertSuccessTrue(QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon));
}

TEST(PolygonPositiveAreaIntersectionTest, TangentCircularPolygonsReturnFalse)
{
    const auto first = MakeCircle({0.0, 0.0}, 5.0);
    const auto second = MakeCircle({10.0, 0.0}, 5.0);
    AssertSuccessFalse(QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon));
}

// 两孔洞多边形重叠：两个带洞的正方形外环部分重叠，但洞不重叠，返回 true。
TEST(PolygonPositiveAreaIntersectionTest, TwoHoledPolygonsOverlapReturnTrue)
{
    const auto first = SCPolygon2d(
        MakeSquare(0.0, 0.0, 20.0, 20.0).OuterRing(),
        {MakeSquare(2.0, 2.0, 5.0, 5.0).OuterRing()});
    const auto second = SCPolygon2d(
        MakeSquare(10.0, 0.0, 30.0, 20.0).OuterRing(),
        {MakeSquare(25.0, 2.0, 28.0, 5.0).OuterRing()});
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}

// 一方完全在另一方孔洞内，但外环仍重叠：存在正面积公共区域返回 true。
TEST(PolygonPositiveAreaIntersectionTest, OverlappingRingsWithHolesReturnTrue)
{
    // first 外环 [0,20]，洞 [8,12]。
    const auto first = SCPolygon2d(
        MakeSquare(0.0, 0.0, 20.0, 20.0).OuterRing(),
        {MakeSquare(8.0, 8.0, 12.0, 12.0).OuterRing()});
    // second 完全在 first 内但部分在洞内、部分在填充区域。
    const auto second = MakeSquare(6.0, 6.0, 10.0, 10.0);
    const auto result = QueryPolygonPositiveAreaIntersection(first, second, kDefaultEpsilon);
    AssertSuccessTrue(result);
}
