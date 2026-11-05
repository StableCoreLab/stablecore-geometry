#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "Core/Metrics.h"
#include "Core/Projection.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentProjection2d;
using Geometry::SCVector2d;

TEST(CoreAlgorithmsTest, ParameterAtLengthDispatchesThroughSegmentInterface)
{
    const SCLineSegment2d line({0.0, 0.0}, {4.0, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& genericLine = line;
    const ISCSegment2d& genericArc = arc;

    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 2.0), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 6.0, true), 1.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 0.5), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 2.0, true), 1.0);
}

TEST(CoreAlgorithmsTest, ParameterAtLengthAcceptsExplicitToleranceForSmallSegments)
{
    const SCLineSegment2d line({0.0, 0.0}, {1e-10, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1e-10, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& genericLine = line;
    const ISCSegment2d& genericArc = arc;

    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false, 1e-15), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 0.5, false, 1e-15), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 2e-10, true, 1e-15), 1.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false,
                                                std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, std::numeric_limits<double>::max(),
                                                false, 1e-15), 0.0);
}

TEST(CoreAlgorithmsTest, SmallCallerToleranceProjectsShortLine)
{
    constexpr double length = 1e-10;
    constexpr double eps = 1e-15;
    const SCLineSegment2d line({0.0, 0.0}, {length, 0.0});
    const SCPoint2d query{0.5 * length, 1e-11};
    EXPECT_EQ(ProjectPointToLineSegment(query, line, true).point, line.startPoint);

    const auto typed = ProjectPointToLineSegment(query, line, true, eps);
    const auto endpoints = ProjectPointToSegment(query, line.startPoint, line.endPoint, true, eps);
    for (const auto& projection : {typed, endpoints})
    {
        ASSERT_TRUE(projection.IsValid());
        EXPECT_TRUE(projection.point.AlmostEquals({0.5 * length, 0.0}, eps));
        EXPECT_NEAR(projection.parameter, 0.5, 1e-12);
        EXPECT_TRUE(projection.isOnSegment);
    }

    const auto beyond = ProjectPointToLineSegment({2.0 * length, 0.0}, line, true, eps);
    EXPECT_TRUE(beyond.point.AlmostEquals(line.endPoint, eps));
    EXPECT_FALSE(beyond.isOnSegment);
    EXPECT_FALSE(ProjectPointToLineSegment(query, line, true,
                                          std::numeric_limits<double>::infinity()).IsValid());
}

TEST(CoreAlgorithmsTest, SmallCallerToleranceProjectsArcAndGenericSegments)
{
    constexpr double radius = 1e-10;
    constexpr double eps = 1e-15;
    constexpr double nearAngle = 1e-6;
    const SCArcSegment2d arc({0.0, 0.0}, radius, 0.0, Geometry::kPi * 0.5);
    const SCPoint2d beforeStart{radius * std::cos(-nearAngle), radius * std::sin(-nearAngle)};
    const SCPoint2d afterEnd{radius * std::cos(Geometry::kPi * 0.5 + nearAngle),
                             radius * std::sin(Geometry::kPi * 0.5 + nearAngle)};

    const auto startProjection = ProjectPointToArcSegment(beforeStart, arc, true, eps);
    ASSERT_TRUE(startProjection.IsValid());
    EXPECT_TRUE(startProjection.point.AlmostEquals(arc.StartPoint(), eps));
    EXPECT_DOUBLE_EQ(startProjection.parameter, 0.0);
    EXPECT_TRUE(startProjection.isOnSegment);

    const auto endProjection = ProjectPointToArcSegment(afterEnd, arc, true, eps);
    ASSERT_TRUE(endProjection.IsValid());
    EXPECT_TRUE(endProjection.point.AlmostEquals(arc.EndPoint(), eps));
    EXPECT_DOUBLE_EQ(endProjection.parameter, 1.0);
    EXPECT_TRUE(endProjection.isOnSegment);

    const SCPoint2d farBeforeStart{radius * std::cos(-0.2), radius * std::sin(-0.2)};
    const auto farProjection = ProjectPointToArcSegment(farBeforeStart, arc, true, eps);
    EXPECT_TRUE(farProjection.point.AlmostEquals(arc.StartPoint(), eps));
    EXPECT_FALSE(farProjection.isOnSegment);

    const ISCSegment2d& genericArc = arc;
    EXPECT_TRUE(ProjectPointToSegment(beforeStart, genericArc, true, eps)
                    .point.AlmostEquals(arc.StartPoint(), eps));
    const SCLineSegment2d line({0.0, 0.0}, {radius, 0.0});
    const ISCSegment2d& genericLine = line;
    EXPECT_TRUE(ProjectPointToSegment({radius * 0.5, radius * 0.1}, genericLine, true, eps)
                    .point.AlmostEquals({radius * 0.5, 0.0}, eps));
    const SCArcSegment2d clockwise({0.0, 0.0}, radius, 0.0, -Geometry::kPi * 0.5);
    const SCPoint2d beforeClockwiseStart{radius * std::cos(nearAngle), radius * std::sin(nearAngle)};
    const auto clockwiseProjection = ProjectPointToArcSegment(beforeClockwiseStart, clockwise, true, eps);
    EXPECT_TRUE(clockwiseProjection.point.AlmostEquals(clockwise.StartPoint(), eps));
    EXPECT_TRUE(clockwiseProjection.isOnSegment);
    const auto unbounded = ProjectPointToArcSegment(beforeStart, arc, false, eps);
    EXPECT_TRUE(unbounded.point.AlmostEquals(beforeStart, eps));
    EXPECT_GT(unbounded.parameter, 1.0);
    EXPECT_FALSE(ProjectPointToArcSegment(beforeStart, arc, true, 0.0).IsValid());
}

TEST(CoreAlgorithmsTest, ExplicitToleranceComputesShortSegmentTangentsAndNormals)
{
    constexpr double epsilon = 1e-15;
    const SCLineSegment2d line({0.0, 0.0}, {1e-10, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1e-10, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& lineBase = line;
    const ISCSegment2d& arcBase = arc;

    EXPECT_EQ(TangentAt(line, 0.5), SCVector2d{});
    EXPECT_EQ(TangentAt(arc, 0.0), SCVector2d{});
    EXPECT_EQ(TangentAt(line, 0.5, epsilon), (SCVector2d{1.0, 0.0}));
    EXPECT_EQ(NormalAt(line, 0.5, epsilon), (SCVector2d{0.0, 1.0}));
    EXPECT_EQ(TangentAt(lineBase, 0.5, epsilon), (SCVector2d{1.0, 0.0}));
    EXPECT_EQ(NormalAt(lineBase, 0.5, epsilon), (SCVector2d{0.0, 1.0}));
    EXPECT_TRUE(TangentAt(arc, 0.0, epsilon).AlmostEquals({0.0, 1.0}, epsilon));
    EXPECT_TRUE(NormalAt(arc, 0.0, epsilon).AlmostEquals({-1.0, 0.0}, epsilon));
    EXPECT_TRUE(TangentAt(arcBase, 0.0, epsilon).AlmostEquals({0.0, 1.0}, epsilon));
    EXPECT_TRUE(NormalAt(arcBase, 0.0, epsilon).AlmostEquals({-1.0, 0.0}, epsilon));
    const SCArcSegment2d clockwise({0.0, 0.0}, 1e-10, 0.0, -Geometry::kPi * 0.5);
    EXPECT_TRUE(TangentAt(clockwise, 0.0, epsilon).AlmostEquals({0.0, -1.0}, epsilon));
    EXPECT_EQ(TangentAt(line, 0.5, 1e-10), SCVector2d{});
    const SCArcSegment2d shortArc({0.0, 0.0}, 1e-10, 0.0, 0.1);
    EXPECT_EQ(TangentAt(shortArc, 0.5, 2e-11), SCVector2d{});
    EXPECT_EQ(TangentAt(line, 0.5, 0.0), SCVector2d{});
    EXPECT_EQ(NormalAt(arc, 0.0, -1.0), SCVector2d{});
    EXPECT_EQ(TangentAt(line, std::numeric_limits<double>::infinity(), epsilon), SCVector2d{});
    EXPECT_EQ(TangentAt(arc, std::numeric_limits<double>::max(), epsilon), SCVector2d{});
}

TEST(CoreAlgorithmsTest, ExplicitToleranceProjectsAndOrientsShortSweepArc)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d arc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    ASSERT_TRUE(arc.IsValid(epsilon));
    const auto projection = Geometry::ProjectPointToArcSegment({1e6, 5e-5}, arc, true, epsilon);
    ASSERT_TRUE(projection.IsValid());
    EXPECT_NEAR(projection.parameter, 0.5, 1e-6);
    EXPECT_NEAR(projection.point.y, 5e-5, 1e-12);
    EXPECT_NEAR(Geometry::TangentAt(arc, 0.5, epsilon).y, 1.0, 1e-12);
    EXPECT_NEAR(Geometry::NormalAt(arc, 0.5, epsilon).x, -1.0, 1e-12);
}

TEST(CoreAlgorithmsTest, MeasuresAndProjectsBasicSegments)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, line), 1.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, arc), std::abs(std::sqrt(2.0) - 2.0), 1e-12);
    EXPECT_NEAR(line.Length(), 4.0, 1e-12);
    EXPECT_TRUE(arc.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}), 1e-12));

    const SCSegmentProjection2d lineProjection = ProjectPointToLineSegment(SCPoint2d{1.0, 2.0}, line, true);
    EXPECT_TRUE(lineProjection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));
    EXPECT_NEAR(lineProjection.parameter, 0.25, 1e-12);

    const SCSegmentProjection2d arcProjection = ProjectPointToArcSegment(SCPoint2d{1.0, 1.0}, arc, true);
    EXPECT_TRUE(arcProjection.point.AlmostEquals(SCPoint2d{std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));

    EXPECT_TRUE(line.PointAtLength(2.0, true).AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(TangentAt(line, 0.5).AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    EXPECT_TRUE(NormalAt(line, 0.5).AlmostEquals(SCVector2d{0.0, 1.0}, 1e-12));
}

TEST(CoreAlgorithmsTest, ProjectionOverloadsPreserveDocumentedMembershipSemantics)
{
    const SCLineSegment2d segment({0.0, 0.0}, {4.0, 0.0});
    const ISCSegment2d& generic = segment;
    for (const bool explicitTolerance : {false, true})
    {
        SCOPED_TRACE(explicitTolerance);
        for (const bool clamp : {false, true})
        {
            SCOPED_TRACE(clamp);
            for (const double x : {-2.0, 0.0, 2.0, 4.0, 6.0})
            {
                SCOPED_TRACE(x);
                const SCPoint2d query{x, 3.0};
                const auto typed = explicitTolerance
                    ? ProjectPointToLineSegment(query, segment, clamp, 1e-12)
                    : ProjectPointToLineSegment(query, segment, clamp);
                const auto base = explicitTolerance
                    ? ProjectPointToSegment(query, generic, clamp, 1e-12)
                    : ProjectPointToSegment(query, generic, clamp);
                const auto endpoints = explicitTolerance
                    ? ProjectPointToSegment(query, segment.startPoint, segment.endPoint, clamp, 1e-12)
                    : ProjectPointToSegment(query, segment.startPoint, segment.endPoint, clamp);
                const double expectedX = clamp ? (x < 0.0 ? 0.0 : (x > 4.0 ? 4.0 : x)) : x;
                const bool rawOnSegment = x >= 0.0 && x <= 4.0;
                for (const auto& result : {typed, base, endpoints})
                {
                    ASSERT_TRUE(result.IsValid());
                    EXPECT_EQ(result.point, SCPoint2d(expectedX, 0.0));
                    EXPECT_DOUBLE_EQ(result.parameter, expectedX / 4.0);
                    EXPECT_DOUBLE_EQ(result.distanceSquared, 9.0 + (x - expectedX) * (x - expectedX));
                }
                EXPECT_EQ(typed.isOnSegment, rawOnSegment);
                EXPECT_EQ(base.isOnSegment, rawOnSegment);
                EXPECT_EQ(endpoints.isOnSegment, rawOnSegment);
            }
        }
    }
}
