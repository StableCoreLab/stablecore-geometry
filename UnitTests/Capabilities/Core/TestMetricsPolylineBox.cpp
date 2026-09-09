#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry.h"

using Geometry::Contains;
using Geometry::Intersects;
using Geometry::kDefaultEpsilon;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ISCSegment2d;
using Geometry::kPi;

using namespace Geometry;

namespace
{
    [[nodiscard]] SCBox2d MakeBox(double minX, double minY, double maxX, double maxY)
    {
        return SCBox2d::FromMinMax(SCPoint2d{minX, minY}, SCPoint2d{maxX, maxY});
    }

    [[nodiscard]] SCPolyline2d MakeOpenPolyline(const std::vector<SCPoint2d>& points)
    {
        return SCPolyline2d(points, SCPolylineClosure::Open);
    }

    [[nodiscard]] SCPolyline2d MakeClosedPolyline(const std::vector<SCPoint2d>& points)
    {
        return SCPolyline2d(points, SCPolylineClosure::Closed);
    }

    // 构造一个半圆弧多段线：圆心、半径、起始角度、扫掠角度。
    [[nodiscard]] SCPolyline2d MakeArcPolyline(const SCPoint2d& center,
                                                double radius,
                                                double startAngle,
                                                double sweepAngle)
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.push_back(std::make_shared<SCArcSegment2d>(center, radius, startAngle, sweepAngle));
        return SCPolyline2d(std::move(segments), SCPolylineClosure::Open);
    }
}  // namespace

// 完全在盒内：线段完全位于 B_eps 内部（且在原盒内），Contains 与 Intersects 均为 true。
// 对应内部关系 Contained。
TEST(MetricsPolylineBoxTest, ContainedLineReturnsTrueForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{2.0, 2.0}, {8.0, 8.0}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 完全在外：线段完全在 B_eps 之外，Contains 与 Intersects 均为 false。
// 对应内部关系 Disjoint。
TEST(MetricsPolylineBoxTest, DisjointLineReturnsFalseForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{20.0, 20.0}, {30.0, 30.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 线段与盒边的包围盒相交，但解析参数落在边段范围外。该用例覆盖 checked Line-Line
// 无交集证明路径，确保默认空求交结果只会在可独立证明时聚合为 Disjoint。
TEST(MetricsPolylineBoxTest, CheckedLineNoIntersectionProofReturnsFalseForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{9.0, 11.0}, {11.0, 9.1}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 路径仅从 B_eps 右上角外侧掠过：包围盒重叠，但真实交点参数刚好落在边段外。
// 局部数值容差不得把该间隙钳制为端点接触。
TEST(MetricsPolylineBoxTest, NearCornerOutsideEpsilonBandDoesNotTouch)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const double delta = 5e-10;
    const auto polyline = MakeOpenPolyline(
        {{9.0, 10.0 + kDefaultEpsilon + delta}, {10.0 + kDefaultEpsilon + delta, 11.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 边/角接触：线段端点恰好接触 B_eps 边界，Contains 为 false，Intersects 为 true。
// 对应内部关系 Touching。
TEST(MetricsPolylineBoxTest, TouchingLineReturnsFalseContainsTrueIntersects)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 线段从盒外接触到盒角 (10,10)。
    const auto polyline = MakeOpenPolyline({{15.0, 15.0}, {10.0, 10.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 穿越：线段从盒外穿过盒到另一侧，Contains 为 false，Intersects 为 true。
// 对应内部关系 Crossing。
TEST(MetricsPolylineBoxTest, CrossingLineReturnsFalseContainsTrueIntersects)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 线段从 (-5,5) 到 (15,5)，穿过整个盒。
    const auto polyline = MakeOpenPolyline({{-5.0, 5.0}, {15.0, 5.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 整段沿 B_eps 边界：线段完全沿盒边界，Contains 与 Intersects 均为 true。
// 对应内部关系 ContainedTouching。
TEST(MetricsPolylineBoxTest, ContainedTouchingLineReturnsTrueForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 线段完全沿盒底边 (0,0)→(10,0)。
    const auto polyline = MakeOpenPolyline({{0.0, 0.0}, {10.0, 0.0}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 距原盒小于 eps：线段在原盒外但在 B_eps 内，Contains 为 true，Intersects 为 true。
TEST(MetricsPolylineBoxTest, WithinEpsilonBandReturnsTrueForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 线段在原盒上方 0.001 处，eps=0.1 时在 B_eps 内。
    const auto polyline = MakeOpenPolyline({{2.0, 10.001}, {8.0, 10.001}});
    EXPECT_TRUE(Contains(box, polyline, 0.1));
    EXPECT_TRUE(Intersects(box, polyline, 0.1));
}

// 跨原盒边界但仍完全在 B_eps 内：线段一端在原盒内、一端在原盒外但在 B_eps 内。
// Contains 为 true（完全在 B_eps 内），Intersects 为 true。
// 按方案语义，Contains 检查的是 B_eps 而非原盒，故线段完全在 B_eps 内时 Contains 为 true。
TEST(MetricsPolylineBoxTest, CrossingOriginalButWithinEpsilonReturnsTrueForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 线段从 (5,5) 到 (10.05,5)，一端在盒内、一端在 eps 带内。
    const auto polyline = MakeOpenPolyline({{5.0, 5.0}, {10.05, 5.0}});
    EXPECT_TRUE(Contains(box, polyline, 0.1));
    EXPECT_TRUE(Intersects(box, polyline, 0.1));
}

// 多段折线混合：部分段在盒内、部分段在盒外穿越。
TEST(MetricsPolylineBoxTest, MultiSegmentPolylineMixedRelations)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 第一段在盒内，第二段穿越盒边。
    const auto polyline = MakeOpenPolyline({{2.0, 2.0}, {8.0, 2.0}, {15.0, 2.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 多段折线全在盒内：所有段都在 B_eps 内，Contains 为 true。
TEST(MetricsPolylineBoxTest, MultiSegmentPolylineAllContained)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{2.0, 2.0}, {8.0, 2.0}, {8.0, 8.0}, {2.0, 8.0}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// Line/Arc 混合多段线：线段和圆弧混合，段间端点必须连接。
TEST(MetricsPolylineBoxTest, MixedLineAndArcPolyline)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    // 圆弧在盒内：圆心 (5,5)，半径 2，半圆。
    // 弧起点 (7,5)，弧终点 (3,5)。
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{5.0, 5.0}, 2.0, 0.0, kPi));
    // 线段从弧终点 (3,5) 到 (3,2)，均在盒内。
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{3.0, 5.0}, SCPoint2d{3.0, 2.0}));
    const SCPolyline2d polyline(std::move(segments), SCPolylineClosure::Open);
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 圆弧端点外但穿盒：圆弧两端在盒外，但中段穿过盒。
TEST(MetricsPolylineBoxTest, ArcEndpointsOutsideButCrossingBox)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (5, -5)，半径 8，从角度 0 到 pi 的半圆弧。
    // 端点在 (13,-5) 和 (-3,-5)，均在盒外；中段穿过盒。
    const auto polyline = MakeArcPolyline(SCPoint2d{5.0, -5.0}, 8.0, 0.0, kPi);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 圆弧端点内但中段越界：圆弧两端在盒内，但中段超出 B_eps。
TEST(MetricsPolylineBoxTest, ArcEndpointsInsideButMiddleOutside)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (5,5)，半径 6，从角度 0.7 到 2.44（sweep≈1.74）的劣弧。
    // 起点 (≈9.59,≈8.86) 与终点 (≈0.41,≈8.86) 均在盒内；
    // 中点 (角度 π/2) 为 (5,11)，在盒外。
    const auto polyline = MakeArcPolyline(SCPoint2d{5.0, 5.0}, 6.0, 0.7, 2.44 - 0.7);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 圆弧完全在盒内：Contains 与 Intersects 均为 true。
TEST(MetricsPolylineBoxTest, ArcFullyInsideBox)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (5, 5)，半径 2，半圆弧。完全在盒内。
    const auto polyline = MakeArcPolyline(SCPoint2d{5.0, 5.0}, 2.0, 0.0, kPi);
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 闭合路径围盒但不接触：闭合多段线包围选择框，但路径本身不接触 B_eps。
// 通用 Intersects 返回 false（填充语义不在通用多段线盒关系中）。
TEST(MetricsPolylineBoxTest, ClosedPathEnclosesBoxButDoesNotTouch)
{
    const auto box = MakeBox(4.0, 4.0, 6.0, 6.0);
    // 闭合路径 [0,10]×[0,10] 包围盒 [4,6]×[4,6]，但路径本身不接触 B_eps。
    const auto polyline = MakeClosedPolyline({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 无效盒：无效盒返回 false。
TEST(MetricsPolylineBoxTest, InvalidBoxReturnsFalse)
{
    const SCBox2d invalidBox;
    const auto polyline = MakeOpenPolyline({{0.0, 0.0}, {10.0, 10.0}});
    EXPECT_FALSE(Contains(invalidBox, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(invalidBox, polyline, kDefaultEpsilon));
}

// 无效多段线：空多段线返回 false。
TEST(MetricsPolylineBoxTest, EmptyPolylineReturnsFalse)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const SCPolyline2d emptyPolyline;
    EXPECT_FALSE(Contains(box, emptyPolyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, emptyPolyline, kDefaultEpsilon));
}

// 零段多段线：不可选择，返回 false。
TEST(MetricsPolylineBoxTest, ZeroSegmentPolylineReturnsFalse)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 只有一个点的多段线，无法构成有效段。
    const auto polyline = MakeOpenPolyline({{5.0, 5.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 无效 eps：eps 非有限或非正返回 false。
TEST(MetricsPolylineBoxTest, InvalidEpsilonReturnsFalse)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{2.0, 2.0}, {8.0, 8.0}});
    EXPECT_FALSE(Contains(box, polyline, 0.0));
    EXPECT_FALSE(Intersects(box, polyline, 0.0));
    EXPECT_FALSE(Contains(box, polyline, -1e-9));
    EXPECT_FALSE(Intersects(box, polyline, -1e-9));
    EXPECT_FALSE(Contains(box, polyline, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(Intersects(box, polyline, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(Contains(box, polyline, std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(Intersects(box, polyline, std::numeric_limits<double>::infinity()));
}

// B_eps 溢出：盒坐标接近 double 最大值，加 eps 后溢出为无穷，返回 false。
TEST(MetricsPolylineBoxTest, BEpsilonOverflowReturnsFalse)
{
    // 盒最大坐标接近 double 最大值，加 eps 后溢出为无穷。
    const double near = std::numeric_limits<double>::max();
    const auto box = MakeBox(0.0, 0.0, near, near);
    const auto polyline = MakeOpenPolyline({{1.0, 1.0}, {2.0, 2.0}});
    // expandedMax = (near + eps, near + eps) = (inf, inf) → 非有限 → 返回 false。
    EXPECT_FALSE(Contains(box, polyline, 1.0));
    EXPECT_FALSE(Intersects(box, polyline, 1.0));
}

// 局部坐标极大量级回归：极大有限坐标的盒和线段仍能正确分类。
TEST(MetricsPolylineBoxTest, HugeFiniteCoordinatesRegression)
{
    const double huge = 1e150;
    const auto box = MakeBox(0.0, 0.0, huge, huge);
    const auto polyline = MakeOpenPolyline({{huge * 0.25, huge * 0.25}, {huge * 0.75, huge * 0.75}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 段的全局长度平方会溢出，但盒决定的局部坐标仍可可靠表达路径关系。
// 内核不得在建立局部坐标前以 segment.Length() 拒绝该合法输入。
TEST(MetricsPolylineBoxTest, HugeFiniteSegmentLengthDoesNotPreventLocalClassification)
{
    const double huge = 1e200;
    const auto box = MakeBox(0.0, 0.0, huge, huge);
    const auto polyline = MakeOpenPolyline({{huge * 0.2, huge * 0.5}, {huge * 0.8, huge * 0.5}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 极窄但有效的盒在按最大边长归一化后会产生短边；短边不能因默认 epsilon
// 被当作无效线段。路径完全位于 B_eps 内，两个公开关系均应为 true。
TEST(MetricsPolylineBoxTest, ThinBoxDoesNotRejectLocalShortEdges)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 1e-11);
    const auto polyline = MakeOpenPolyline({{2.0, 5e-12}, {8.0, 5e-12}});
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 局部坐标极小量级回归：极小有限坐标的盒和线段仍能正确分类。
// 注意：SCLineSegment2d::IsValid() 以 kDefaultEpsilon(1e-9) 判定退化，段长须 > 1e-9。
TEST(MetricsPolylineBoxTest, TinyFiniteCoordinatesRegression)
{
    const double tiny = 1e-6;
    const auto box = MakeBox(0.0, 0.0, tiny * 10.0, tiny * 10.0);
    const auto polyline = MakeOpenPolyline({{tiny, tiny}, {tiny * 9.0, tiny * 9.0}});
    // eps 相对于盒尺寸较大，线段在 B_eps 内。
    EXPECT_TRUE(Contains(box, polyline, tiny));
    EXPECT_TRUE(Intersects(box, polyline, tiny));
}

// 圆弧完全在盒外：Contains 与 Intersects 均为 false。
TEST(MetricsPolylineBoxTest, ArcFullyOutsideBox)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (20, 20)，半径 2，远离盒。
    const auto polyline = MakeArcPolyline(SCPoint2d{20.0, 20.0}, 2.0, 0.0, kPi);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 圆弧包围盒与选择盒重叠，但圆弧仅掠过右上角外侧，不与任一盒边段相交。
// 该用例覆盖 checked Arc-Line 无交集证明路径。
TEST(MetricsPolylineBoxTest, CheckedArcNoIntersectionProofReturnsFalseForBoth)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeArcPolyline(SCPoint2d{15.0, 15.0}, 7.0, kPi, kPi * 0.5);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 圆弧接触盒角：圆弧恰好接触 B_eps 边界。
TEST(MetricsPolylineBoxTest, ArcTouchingBoxCorner)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (15, 5)，半径 5，半圆弧从角度 pi 到 2*pi。
    // 端点 (10,5) 和 (20,5)，(10,5) 恰在盒右边。
    const auto polyline = MakeArcPolyline(SCPoint2d{15.0, 5.0}, 5.0, kPi, kPi);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 多段线含退化段：含零长度段的无效段导致整体失败，Contains 与 Intersects 均为 false。
TEST(MetricsPolylineBoxTest, DegenerateSegmentInPolylineReturnsFalse)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 零长度段（同一点）。
    const auto polyline = MakeOpenPolyline({{5.0, 5.0}, {5.0, 5.0}, {8.0, 8.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 非有限坐标输入：含 NaN 坐标的线段导致失败。
TEST(MetricsPolylineBoxTest, NonFiniteInputReturnsFalse)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    const auto polyline = MakeOpenPolyline({{std::numeric_limits<double>::quiet_NaN(), 5.0},
                                              {8.0, 8.0}});
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_FALSE(Intersects(box, polyline, kDefaultEpsilon));
}

// 完整圆弧多段线在盒内：360 度圆弧完全在盒内。
TEST(MetricsPolylineBoxTest, FullCircleArcInsideBox)
{
    const auto box = MakeBox(0.0, 0.0, 20.0, 20.0);
    // 圆心 (10,10)，半径 3，完整圆。
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{10.0, 10.0}, 3.0, 0.0, kPi));
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{10.0, 10.0}, 3.0, kPi, kPi));
    const SCPolyline2d polyline(std::move(segments), SCPolylineClosure::Open);
    EXPECT_TRUE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}

// 完整圆弧多段线穿越盒：360 度圆弧部分在盒内部分在盒外。
TEST(MetricsPolylineBoxTest, FullCircleArcCrossingBox)
{
    const auto box = MakeBox(0.0, 0.0, 10.0, 10.0);
    // 圆心 (10,5)，半径 3，完整圆。部分在盒内部分在盒外。
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{10.0, 5.0}, 3.0, 0.0, kPi));
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{10.0, 5.0}, 3.0, kPi, kPi));
    const SCPolyline2d polyline(std::move(segments), SCPolylineClosure::Open);
    EXPECT_FALSE(Contains(box, polyline, kDefaultEpsilon));
    EXPECT_TRUE(Intersects(box, polyline, kDefaultEpsilon));
}
