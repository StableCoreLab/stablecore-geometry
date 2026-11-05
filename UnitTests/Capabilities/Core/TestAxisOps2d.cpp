#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include "Core/AxisOps.h"
#include "Core/Projection.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCAxisProjection2d;
using Geometry::SCAxisSample2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentKind2d;
using Geometry::SCSegmentProjection2d;
using Geometry::SCSegmentSplit2d;
using Geometry::SCSegmentTrim2d;
using Geometry::SCSnapResult2d;
using Geometry::SCVector2d;

TEST(CoreAlgorithmsTest, SamplesAxisAndEditsSegments)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    const SCAxisSample2d sample = SampleAxis(line, 0.5);
    EXPECT_TRUE(sample.point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(sample.tangent.AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    const SCAxisProjection2d axisProjection = ProjectPointToAxis(SCPoint2d{1.0, 1.0}, line);
    EXPECT_TRUE(axisProjection.projection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));

    const SCSegmentSplit2d split = SplitSegment(line, 0.5);
    ASSERT_TRUE(split.success);
    ASSERT_EQ(split.first->Kind(), SCSegmentKind2d::Line);
    ASSERT_EQ(split.second->Kind(), SCSegmentKind2d::Line);

    const SCSegmentTrim2d trim = TrimSegment(arc, 0.25, 0.75);
    ASSERT_TRUE(trim.success);
    ASSERT_EQ(trim.segment->Kind(), SCSegmentKind2d::Arc);
}

TEST(CoreAlgorithmsTest, SnapsToBasicSegments)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    const std::unique_ptr<ISCSegment2d> lineClone = line.Clone();
    const std::unique_ptr<ISCSegment2d> arcClone = arc.Clone();
    const std::array<const ISCSegment2d*, 2> snapSegments{lineClone.get(), arcClone.get()};
    const SCSnapResult2d snap = SnapPointToSegments(SCPoint2d{1.0, 1.0}, snapSegments, 2.0);
    ASSERT_TRUE(snap.snapped);
}

TEST(CoreAlgorithmsTest, AxisOperationsAcceptCallerToleranceForShortSegments)
{
    const SCLineSegment2d line({0.0, 0.0}, {1e-10, 0.0});
    constexpr double epsilon = 1e-12;
    ASSERT_TRUE(line.IsValid(epsilon));

    const auto sample = SampleAxis(line, 0.25, epsilon);
    EXPECT_TRUE(sample.IsValid());
    EXPECT_NEAR(sample.point.x, 2.5e-11, 1e-25);
    EXPECT_NEAR(sample.tangent.x, 1.0, 1e-12);
    EXPECT_NEAR(sample.normal.y, 1.0, 1e-12);

    const auto atLength = SampleAxisAtLength(line, 5e-11, true, epsilon);
    EXPECT_TRUE(atLength.IsValid());
    EXPECT_NEAR(atLength.point.x, 5e-11, 1e-25);
    EXPECT_NEAR(atLength.parameter, 0.5, 1e-12);

    const auto projection = ProjectPointToAxis({2.5e-11, 1e-11}, line, epsilon);
    ASSERT_TRUE(projection.projection.IsValid());
    EXPECT_NEAR(projection.projection.point.x, 2.5e-11, 1e-25);
    EXPECT_NEAR(projection.projection.parameter, 0.25, 1e-12);
    EXPECT_NEAR(projection.tangent.x, 1.0, 1e-12);

    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, 1e-10);
    ASSERT_TRUE(arc.IsValid(epsilon));
    const auto arcSample = SampleAxisAtLength(arc, 5e-11, true, epsilon);
    EXPECT_TRUE(arcSample.IsValid());
    EXPECT_NEAR(arcSample.point.y, 5e-11, 1e-20);
    EXPECT_NEAR(arcSample.tangent.y, 1.0, 1e-12);
    const auto arcProjection = ProjectPointToAxis({1.0, 5e-11}, arc, epsilon);
    ASSERT_TRUE(arcProjection.projection.IsValid());
    EXPECT_NEAR(arcProjection.projection.parameter, 0.5, 1e-12);
    EXPECT_FALSE(ProjectPointToAxis({2.5e-11, 1e-11}, line, 0.0).projection.IsValid());
    EXPECT_FALSE(SampleAxis(line, 0.25, 0.0).IsValid());
    EXPECT_FALSE(SampleAxisAtLength(line, 5e-11, true, 0.0).IsValid());
    EXPECT_FALSE(SCAxisSample2d{}.IsValid());
    EXPECT_FALSE(SCAxisProjection2d{}.IsValid());
    EXPECT_FALSE(SCSegmentProjection2d{}.IsValid());
    EXPECT_FALSE(ProjectPointToArcSegment({1.0, 0.0}, SCArcSegment2d{}, true).IsValid());
}

TEST(CoreAlgorithmsTest, SnapResultRejectsInvalidPayload)
{
    EXPECT_TRUE(SCSnapResult2d{}.IsValid());
    SCSnapResult2d result{true, {1.0, 0.0}, 1.0, 0, 0.5};
    ASSERT_TRUE(result.IsValid());
    result.parameter = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(result.IsValid());
    result.parameter = 0.5;
    result.distanceSquared = -1.0;
    EXPECT_FALSE(result.IsValid());
}

TEST(CoreAlgorithmsTest, AxisProjectionChoosesNearestArcEndpointAcrossPeriodBoundary)
{
    for (const double direction : {1.0, -1.0})
    {
        SCOPED_TRACE(direction);
        const SCArcSegment2d arc({}, 2.0, direction * 0.1, direction * 0.2);
        const double queryAngle = direction * 0.099;
        const SCPoint2d query{2.0 * std::cos(queryAngle), 2.0 * std::sin(queryAngle)};
        const auto axis = ProjectPointToAxis(query, arc);
        ASSERT_TRUE(axis.projection.IsValid());
        EXPECT_DOUBLE_EQ(axis.projection.parameter, 0.0);
        EXPECT_TRUE(axis.projection.point.AlmostEquals(arc.StartPoint(), 1e-12));
        EXPECT_TRUE(axis.projection.isOnSegment);
        const auto direct = ProjectPointToArcSegment(query, arc, true);
        EXPECT_EQ(axis.projection.point, direct.point);
        EXPECT_DOUBLE_EQ(axis.projection.distanceSquared, direct.distanceSquared);
        EXPECT_TRUE(axis.tangent.AlmostEquals(Geometry::TangentAt(arc, 0.0), 1e-12));
    }
    const SCArcSegment2d centered({}, 2.0, -0.5, 1.0);
    for (const SCPoint2d query : {SCPoint2d{}, SCPoint2d{1e-10, 0.0}})
    {
        const auto centerProjection = ProjectPointToAxis(query, centered);
        ASSERT_TRUE(centerProjection.projection.IsValid());
        EXPECT_EQ(centerProjection.projection.point, centered.StartPoint());
        EXPECT_DOUBLE_EQ(centerProjection.projection.parameter, 0.0);
        EXPECT_TRUE(centerProjection.projection.isOnSegment);
    }
}

TEST(CoreAlgorithmsTest, AxisProjectionPreservesLargeFiniteLineParameters)
{
    for (const double length : {4.0, 1e200})
    {
        SCOPED_TRACE(length);
        const SCLineSegment2d line({}, {length, 0.0});
        const SCPoint2d query{length * 0.5, 0.0};
        const auto axis = ProjectPointToAxis(query, line);
        ASSERT_TRUE(axis.projection.IsValid());
        EXPECT_EQ(axis.projection.point, query);
        EXPECT_DOUBLE_EQ(axis.projection.parameter, 0.5);
        EXPECT_DOUBLE_EQ(axis.projection.distanceSquared, 0.0);
        EXPECT_TRUE(axis.projection.isOnSegment);
        EXPECT_EQ(axis.tangent, SCVector2d(1.0, 0.0));
        EXPECT_EQ(axis.normal, SCVector2d(0.0, 1.0));
    }
    const SCLineSegment2d line({}, {4.0, 0.0});
    const auto beyond = ProjectPointToAxis({6.0, 1.0}, line);
    ASSERT_TRUE(beyond.projection.IsValid());
    EXPECT_EQ(beyond.projection.point, line.endPoint);
    EXPECT_DOUBLE_EQ(beyond.projection.parameter, 1.0);
    EXPECT_TRUE(beyond.projection.isOnSegment);
}

TEST(CoreAlgorithmsTest, AxisSamplingRetainsDefaultArcDegeneracyRules)
{
    for (const double direction : {1.0, -1.0})
    {
        SCOPED_TRACE(direction);
        const SCArcSegment2d arc({}, 2.0, 0.0, direction * Geometry::kPi);
        const auto sample = SampleAxis(arc, 0.5);
        ASSERT_TRUE(sample.IsValid());
        EXPECT_TRUE(sample.point.AlmostEquals({0.0, direction * 2.0}, 1e-12));
        EXPECT_TRUE(sample.tangent.AlmostEquals({-1.0, 0.0}, 1e-12));
        EXPECT_TRUE(sample.normal.AlmostEquals({0.0, -1.0}, 1e-12));
    }
    const SCArcSegment2d invalid({}, 2.0, 0.3, 0.0);
    ASSERT_FALSE(invalid.IsValid());
    const auto invalidSample = SampleAxis(invalid, 0.5);
    EXPECT_EQ(invalidSample.point, invalid.StartPoint());
    EXPECT_EQ(invalidSample.tangent, SCVector2d{});
    EXPECT_EQ(invalidSample.normal, SCVector2d{});
    EXPECT_FALSE(invalidSample.IsValid());
    const auto invalidProjection = ProjectPointToAxis({}, invalid);
    EXPECT_EQ(invalidProjection.tangent, SCVector2d{});
    EXPECT_EQ(invalidProjection.normal, SCVector2d{});
    EXPECT_FALSE(invalidProjection.IsValid());

    const SCArcSegment2d tiny({}, 1e-10, 0.0, Geometry::kPi * 0.5);
    ASSERT_TRUE(tiny.IsValid());
    const auto defaultSample = SampleAxis(tiny, 0.5);
    EXPECT_EQ(defaultSample.tangent, SCVector2d{});
    EXPECT_FALSE(defaultSample.IsValid());
    const auto explicitSample = SampleAxis(tiny, 0.5, 1e-15);
    ASSERT_TRUE(explicitSample.IsValid());
    EXPECT_NEAR(explicitSample.tangent.Length(), 1.0, 1e-12);
    EXPECT_NEAR(explicitSample.normal.Length(), 1.0, 1e-12);
}
