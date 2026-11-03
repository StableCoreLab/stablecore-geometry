#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "Core/Intersection.h"

namespace
{
    void CheckParameterization(const Geometry::ISCSegment2d& first,
                               const Geometry::ISCSegment2d& second,
                               const std::vector<Geometry::SCSegmentIntersection2d>& hits,
                               double eps = 1e-10)
    {
        double previous = -1.0;
        for (const auto& hit : hits)
        {
            ASSERT_TRUE(hit.HasIntersection());
            ASSERT_EQ(hit.pointCount, hit.kind == Geometry::SCIntersectionKind2d::Overlap ? 2U : 1U);
            EXPECT_GE(hit.points[0].parameterOnFirst, previous);
            for (std::size_t i = 0; i < hit.pointCount; ++i)
            {
                const auto& point = hit.points[i];
                EXPECT_GE(point.parameterOnFirst, 0.0);
                EXPECT_LE(point.parameterOnFirst, 1.0);
                EXPECT_GE(point.parameterOnSecond, 0.0);
                EXPECT_LE(point.parameterOnSecond, 1.0);
                EXPECT_TRUE(first.PointAt(point.parameterOnFirst).AlmostEquals(point.point, eps));
                EXPECT_TRUE(second.PointAt(point.parameterOnSecond).AlmostEquals(point.point, eps));
            }
            if (hit.pointCount == 2)
            {
                const double t = 0.5 * (hit.points[0].parameterOnFirst + hit.points[1].parameterOnFirst);
                const double u = 0.5 * (hit.points[0].parameterOnSecond + hit.points[1].parameterOnSecond);
                EXPECT_TRUE(first.PointAt(t).AlmostEquals(second.PointAt(u), eps));
            }
            previous = hit.points[hit.pointCount - 1].parameterOnFirst;
        }
    }
}

TEST(SegmentIntersectionAllTest, ReturnsBothDisconnectedCoincidentArcIntervals)
{
    const Geometry::SCArcSegment2d first({0.0, 0.0}, 1.0, 0.0, 1.5 * Geometry::kPi);
    const Geometry::SCArcSegment2d second({0.0, 0.0}, 1.0, Geometry::kPi, 1.5 * Geometry::kPi);
    const auto hits = Geometry::IntersectAll(first, second);
    ASSERT_EQ(hits.size(), 2U);
    for (const auto& hit : hits) EXPECT_EQ(hit.kind, Geometry::SCIntersectionKind2d::Overlap);
    EXPECT_NEAR(hits[0].points[0].parameterOnFirst, 0.0, 1e-12);
    EXPECT_NEAR(hits[0].points[1].parameterOnFirst, 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(hits[1].points[0].parameterOnFirst, 2.0 / 3.0, 1e-12);
    EXPECT_NEAR(hits[1].points[1].parameterOnFirst, 1.0, 1e-12);
    CheckParameterization(first, second, hits);
    CheckParameterization(second, first, Geometry::IntersectAll(second, first));
    const auto legacy = Geometry::Intersect(first, second);
    ASSERT_EQ(legacy.pointCount, 2U);
    EXPECT_EQ(legacy.kind, Geometry::SCIntersectionKind2d::Overlap);
}

TEST(SegmentIntersectionAllTest, FullCirclesSplitAtEachParameterSeam)
{
    for (const double firstDirection : {-1.0, 1.0})
    {
        for (const double secondDirection : {-1.0, 1.0})
        {
            const Geometry::SCArcSegment2d first({0.0, 0.0}, 1.0, 0.0, firstDirection * Geometry::kTwoPi);
            const Geometry::SCArcSegment2d second({0.0, 0.0}, 1.0, Geometry::kPi / 2.0,
                                                  secondDirection * Geometry::kTwoPi);
            const auto hits = Geometry::IntersectAll(first, second);
            ASSERT_EQ(hits.size(), 2U);
            EXPECT_DOUBLE_EQ(hits.front().points[0].parameterOnFirst, 0.0);
            EXPECT_DOUBLE_EQ(hits.back().points[1].parameterOnFirst, 1.0);
            EXPECT_NEAR(hits[0].points[1].parameterOnFirst, hits[1].points[0].parameterOnFirst, 1e-12);
            CheckParameterization(first, second, hits);
            const auto identical = Geometry::IntersectAll(first, first);
            ASSERT_EQ(identical.size(), 1U);
            EXPECT_EQ(identical[0].kind, Geometry::SCIntersectionKind2d::Overlap);
            CheckParameterization(first, first, identical);
        }
    }
}

TEST(SegmentIntersectionAllTest, KeepsIsolatedContactBesideOverlap)
{
    const Geometry::SCArcSegment2d first({0.0, 0.0}, 1.0, 0.0, 1.5 * Geometry::kPi);
    const Geometry::SCArcSegment2d second({0.0, 0.0}, 1.0, Geometry::kPi, Geometry::kPi);
    const auto hits = Geometry::IntersectAll(first, second);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_EQ(hits[0].kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_DOUBLE_EQ(hits[0].points[0].parameterOnFirst, 0.0);
    EXPECT_EQ(hits[1].kind, Geometry::SCIntersectionKind2d::Overlap);
    CheckParameterization(first, second, hits);

    const Geometry::SCArcSegment2d upper({0.0, 0.0}, 1.0, 0.0, Geometry::kPi);
    const Geometry::SCArcSegment2d lower({0.0, 0.0}, 1.0, Geometry::kPi, Geometry::kPi);
    const auto contacts = Geometry::IntersectAll(upper, lower);
    ASSERT_EQ(contacts.size(), 2U);
    for (const auto& hit : contacts) EXPECT_EQ(hit.kind, Geometry::SCIntersectionKind2d::Point);
    CheckParameterization(upper, lower, contacts);
}

TEST(SegmentIntersectionAllTest, PolylineQueriesKeepAllOverlapEndpoints)
{
    const auto first = std::make_shared<Geometry::SCArcSegment2d>(
        Geometry::SCPoint2d{0.0, 0.0}, 1.0, 0.0, 1.5 * Geometry::kPi);
    const auto second = std::make_shared<Geometry::SCArcSegment2d>(
        Geometry::SCPoint2d{0.0, 0.0}, 1.0, Geometry::kPi, 1.5 * Geometry::kPi);
    const Geometry::SCPolyline2d a(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{first},
                                   Geometry::SCPolylineClosure::Open);
    const Geometry::SCPolyline2d b(std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{second},
                                   Geometry::SCPolylineClosure::Open);
    for (const auto& hits : {Geometry::Intersect(a, b), Geometry::Intersect(a, *second),
                             Geometry::Intersect(*first, b)})
    {
        ASSERT_EQ(hits.size(), 4U);
        for (std::size_t i = 0; i < hits.size(); ++i)
        {
            EXPECT_EQ(hits[i].kind, Geometry::SCIntersectionKind2d::Overlap);
            EXPECT_NEAR(hits[i].globalParameterOnFirst, static_cast<double>(i) / 3.0, 1e-12);
            EXPECT_TRUE(first->PointAt(hits[i].parameterOnFirstSegment).AlmostEquals(hits[i].point, 1e-12));
            EXPECT_TRUE(second->PointAt(hits[i].parameterOnSecondSegment).AlmostEquals(hits[i].point, 1e-12));
        }
    }
}

TEST(SegmentIntersectionAllTest, OrdinaryIntersectionsRemainSortedAndInvalidInputIsEmpty)
{
    const Geometry::SCArcSegment2d circle({0.0, 0.0}, 1.0, 0.0, Geometry::kTwoPi);
    const Geometry::SCLineSegment2d line({-2.0, 0.0}, {2.0, 0.0});
    const auto hits = Geometry::IntersectAll(line, circle);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_NEAR(hits[0].points[0].parameterOnFirst, 0.25, 1e-12);
    EXPECT_NEAR(hits[1].points[0].parameterOnFirst, 0.75, 1e-12);
    CheckParameterization(line, circle, hits);
    CheckParameterization(circle, line, Geometry::IntersectAll(circle, line));
    const Geometry::SCLineSegment2d subline({-1.0, 0.0}, {1.0, 0.0});
    const auto overlap = Geometry::IntersectAll(line, subline);
    ASSERT_EQ(overlap.size(), 1U);
    EXPECT_EQ(overlap[0].kind, Geometry::SCIntersectionKind2d::Overlap);
    CheckParameterization(line, subline, overlap);
    EXPECT_TRUE(Geometry::IntersectAll(circle, Geometry::SCArcSegment2d{}).empty());
    for (const double eps : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_TRUE(Geometry::IntersectAll(circle, circle, eps).empty());
    }
}

TEST(SegmentIntersectionAllTest, CoincidentIntervalsMatchIndependentAngularScan)
{
    const auto normalize = [](double angle) {
        double result = std::fmod(angle, Geometry::kTwoPi);
        if (result < 0.0) result += Geometry::kTwoPi;
        return std::abs(result - Geometry::kTwoPi) < 1e-12 ? 0.0 : result;
    };
    for (const double firstStart : {-0.7, 0.0, 0.5, 3.2})
    for (const double secondStart : {-0.7, 0.0, 0.5, 3.2})
    for (const double firstSweep : {-2.0, -1.5, -1.0, -0.25, 0.25, 1.0, 1.5, 2.0})
    for (const double secondSweep : {-2.0, -1.5, -1.0, -0.25, 0.25, 1.0, 1.5, 2.0})
    {
        SCOPED_TRACE(::testing::Message() << firstStart << ',' << firstSweep << ';'
                                         << secondStart << ',' << secondSweep);
        const Geometry::SCArcSegment2d first({0.0, 0.0}, 1.0, firstStart * Geometry::kPi,
                                              firstSweep * Geometry::kPi);
        const Geometry::SCArcSegment2d second({0.0, 0.0}, 1.0, secondStart * Geometry::kPi,
                                               secondSweep * Geometry::kPi);
        const auto hits = Geometry::IntersectAll(first, second, 1e-12);
        CheckParameterization(first, second, hits);
        for (int sample = 0; sample <= 64; ++sample)
        {
            const double t = static_cast<double>(sample) / 64.0;
            const auto point = first.PointAt(t);
            const double angle = std::atan2(point.y, point.x);
            const double delta = normalize((secondSweep > 0.0 ? 1.0 : -1.0) * (angle - second.startAngle));
            const bool expected = delta <= std::abs(second.sweepAngle) + 1e-12;
            const bool found = std::any_of(hits.begin(), hits.end(), [&](const auto& hit) {
                return t >= hit.points[0].parameterOnFirst - 1e-12 &&
                    t <= hit.points[hit.pointCount - 1].parameterOnFirst + 1e-12;
            });
            ASSERT_EQ(found, expected) << "first parameter " << t;
        }
    }
}

TEST(SegmentIntersectionAllTest, ShortSweepOverlapUsesExplicitDistanceTolerance)
{
    const double eps = 1e-9;
    const Geometry::SCArcSegment2d first({0.0, 0.0}, 1e6, 0.0, 1e-10);
    const Geometry::SCArcSegment2d second({0.0, 0.0}, 1e6, 2.5e-11, 5e-11);
    ASSERT_TRUE(first.IsValid(eps));
    ASSERT_TRUE(second.IsValid(eps));
    const auto hits = Geometry::IntersectAll(first, second, eps);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits[0].kind, Geometry::SCIntersectionKind2d::Overlap);
    EXPECT_NEAR(hits[0].points[0].parameterOnFirst, 0.25, 1e-12);
    EXPECT_NEAR(hits[0].points[1].parameterOnFirst, 0.75, 1e-12);
    for (const auto& point : hits[0].points)
    {
        EXPECT_TRUE(first.PointAt(point.parameterOnFirst, eps).AlmostEquals(point.point, eps));
        EXPECT_TRUE(second.PointAt(point.parameterOnSecond, eps).AlmostEquals(point.point, eps));
    }
}

TEST(SegmentIntersectionAllTest, CoincidentIntervalsPreserveLargeAndSmallRadiusScales)
{
    for (const double radius : {1e-200, 1.0, 1e200})
    {
        SCOPED_TRACE(::testing::Message() << "radius " << radius);
        const double eps = radius * 1e-10;
        const Geometry::SCArcSegment2d first({0.0, 0.0}, radius, 0.0, 1.5 * Geometry::kPi);
        const Geometry::SCArcSegment2d second({0.0, 0.0}, radius, Geometry::kPi, 1.5 * Geometry::kPi);
        const auto hits = Geometry::IntersectAll(first, second, eps);
        ASSERT_EQ(hits.size(), 2U);
        EXPECT_NEAR(hits[0].points[0].parameterOnFirst, 0.0, 1e-12);
        EXPECT_NEAR(hits[0].points[1].parameterOnFirst, 1.0 / 3.0, 1e-12);
        EXPECT_NEAR(hits[1].points[0].parameterOnFirst, 2.0 / 3.0, 1e-12);
        EXPECT_NEAR(hits[1].points[1].parameterOnFirst, 1.0, 1e-12);
        for (const auto& hit : hits)
        {
            EXPECT_EQ(hit.kind, Geometry::SCIntersectionKind2d::Overlap);
            for (const auto& point : hit.points)
            {
                const auto a = first.PointAt(point.parameterOnFirst, eps);
                const auto b = second.PointAt(point.parameterOnSecond, eps);
                EXPECT_TRUE(a.IsValid());
                EXPECT_TRUE(b.IsValid());
                EXPECT_NEAR(a.x / radius, b.x / radius, 1e-12);
                EXPECT_NEAR(a.y / radius, b.y / radius, 1e-12);
            }
        }
    }
}

TEST(SegmentIntersectionAllTest, CoincidentEndpointGapUsesDistanceToleranceAcrossSeam)
{
    const double eps = 1e-6;
    const Geometry::SCArcSegment2d first({0.0, 0.0}, 1.0, 0.0, Geometry::kPi / 2.0);
    for (const double gap : {eps * 0.5, eps * 2.0})
    {
        const Geometry::SCArcSegment2d second({0.0, 0.0}, 1.0, -Geometry::kPi / 2.0,
                                               Geometry::kPi / 2.0 - gap);
        for (const auto& hits : {Geometry::IntersectAll(first, second, eps),
                                 Geometry::IntersectAll(second, first, eps)})
        {
            ASSERT_EQ(hits.size(), gap < eps ? 1U : 0U);
            if (!hits.empty())
            {
                EXPECT_EQ(hits[0].kind, Geometry::SCIntersectionKind2d::Point);
                EXPECT_LE(std::hypot(hits[0].points[0].point.x - 1.0, hits[0].points[0].point.y), eps);
            }
        }
    }
    const Geometry::SCLineSegment2d tangent({-2.0, 1.0}, {2.0, 1.0});
    const Geometry::SCArcSegment2d circle({0.0, 0.0}, 1.0, 0.0, Geometry::kTwoPi);
    const auto touch = Geometry::IntersectAll(tangent, circle);
    ASSERT_EQ(touch.size(), 1U);
    EXPECT_EQ(touch[0].kind, Geometry::SCIntersectionKind2d::Tangent);
    CheckParameterization(tangent, circle, touch);
    EXPECT_TRUE(Geometry::IntersectAll(Geometry::SCLineSegment2d({-2.0, 2.0}, {2.0, 2.0}), circle).empty());
}
