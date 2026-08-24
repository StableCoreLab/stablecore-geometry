#include <gtest/gtest.h>

#include <array>
#include <numbers>

#include "Geometry.h"
#include "support/GeometryTestSupport.h"

using Geometry::SCBox2d;
using Geometry::SCBox3d;
using Geometry::SCBoxTree2d;
using Geometry::SCBoxTree3d;
using Geometry::SCBoxTreeEntry2d;
using Geometry::SCBoxTreeEntry3d;
using Geometry::SCBoxTreeKnnHit2d;
using Geometry::SCBoxTreeKnnHit3d;
using Geometry::SCGeometryTolerance3d;
using Geometry::SCIntersectionKind2d;
using Geometry::SCArcSegment2d;
using Geometry::SCLine2d;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCLineSegment2d;
using Geometry::SCLineSegment3d;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPolyline2d;
using Geometry::SCSegmentSearch2d;
using Geometry::SCSegmentSearch3d;
using Geometry::SCTangentPoints2d;
using Geometry::TangentPoints;

TEST(MissingAlgorithmsCapabilityTest, TangentPointsReturnsTwoPointsForExternalPoint)
{
    const Geometry::SCCircle2d circle(SCPoint2d{0.0, 0.0}, 5.0);
    const SCTangentPoints2d result = TangentPoints(SCPoint2d{10.0, 0.0}, circle);

    ASSERT_TRUE(result.IsValid());
    EXPECT_EQ(result.pointCount, 2U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(result.points[0], (SCPoint2d{2.5, -2.5 * std::sqrt(3.0)}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(result.points[1], (SCPoint2d{2.5, 2.5 * std::sqrt(3.0)}), 1e-12);
}

TEST(MissingAlgorithmsCapabilityTest, Line2dIntersectionFindsPointAndSegmentHit)
{
    const SCLine2d first = SCLine2d::FromTwoPoints(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCLine2d second = SCLine2d::FromTwoPoints(SCPoint2d{2.0, -2.0}, SCPoint2d{2.0, 2.0});
    const auto lineLine = Geometry::Intersect(first, second);
    ASSERT_TRUE(lineLine.HasIntersection());
    ASSERT_EQ(lineLine.pointCount, 1U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineLine.points[0].point, (SCPoint2d{2.0, 0.0}), 1e-12);

    const SCLineSegment2d segment(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0});
    const auto lineSegment = Geometry::Intersect(first, segment);
    ASSERT_TRUE(lineSegment.HasIntersection());
    ASSERT_EQ(lineSegment.pointCount, 1U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineSegment.points[0].point, (SCPoint2d{2.0, 0.0}), 1e-12);
}

TEST(MissingAlgorithmsCapabilityTest, IntersectExtendedPolicyNoneMatchesIntersectForCollinearLines)
{
    const SCLineSegment2d first{SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}};
    const SCLineSegment2d second{SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}};

    const auto expected = Geometry::Intersect(first, second);
    const auto actual = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::None);

    ASSERT_TRUE(expected.HasIntersection());
    ASSERT_TRUE(actual.HasIntersection());
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_EQ(actual.pointCount, expected.pointCount);
    EXPECT_FALSE(actual.infiniteOverlap);
    ASSERT_EQ(actual.pointCount, 2U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(actual.points[0].point, expected.points[0].point, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(actual.points[1].point, expected.points[1].point, 1e-12);
    EXPECT_TRUE(actual.onFirstSegment);
    EXPECT_TRUE(actual.onSecondSegment);
}

TEST(MissingAlgorithmsCapabilityTest, ProjectPointToLineSegment3dClampsAndProjects)
{
    const SCLineSegment3d segment{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}};
    const auto projection = Geometry::ProjectPointToLineSegment(SCPoint3d{1.0, 0.0, 1.0}, segment, true);

    ASSERT_TRUE(projection.IsValid());
    EXPECT_DOUBLE_EQ(projection.parameter, 0.5);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(projection.point, (SCPoint3d{1.0, 0.0, 0.0}), 1e-12);
    EXPECT_DOUBLE_EQ(projection.distanceSquared, 1.0);
}

TEST(MissingAlgorithmsCapabilityTest, QueryKNearestReturnsOrderedHits)
{
    SCBoxTree2d tree2d;
    tree2d.Add(1, SCBox2d::FromMinMax(SCPoint2d{2.0, 2.0}, SCPoint2d{3.0, 3.0}));
    tree2d.Add(2, SCBox2d::FromMinMax(SCPoint2d{0.5, 0.5}, SCPoint2d{1.0, 1.0}));
    tree2d.Add(3, SCBox2d::FromMinMax(SCPoint2d{5.0, 5.0}, SCPoint2d{6.0, 6.0}));

    const auto hits2d = tree2d.QueryKNearest(SCPoint2d{0.0, 0.0}, 2);
    ASSERT_EQ(hits2d.size(), 2U);
    EXPECT_EQ(hits2d[0].id, 2U);
    EXPECT_EQ(hits2d[1].id, 1U);

    SCSegmentSearch2d search2d;
    search2d.Add(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}});
    search2d.Add(SCLineSegment2d{SCPoint2d{3.0, 0.0}, SCPoint2d{4.0, 0.0}});
    const auto hitsSearch2d = search2d.QueryKNearest(SCPoint2d{0.5, 0.5}, 1);
    ASSERT_EQ(hitsSearch2d.size(), 1U);
    EXPECT_EQ(hitsSearch2d[0].id, 0U);

    SCBoxTree3d tree3d;
    tree3d.Add(7, SCBox3d::FromMinMax(SCPoint3d{2.0, 2.0, 2.0}, SCPoint3d{3.0, 3.0, 3.0}));
    tree3d.Add(8, SCBox3d::FromMinMax(SCPoint3d{0.5, 0.5, 0.5}, SCPoint3d{1.0, 1.0, 1.0}));
    const auto hits3d = tree3d.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 1);
    ASSERT_EQ(hits3d.size(), 1U);
    EXPECT_EQ(hits3d[0].id, 8U);

    SCSegmentSearch3d search3d;
    search3d.Add(SCLineSegment3d{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 0.0, 0.0}});
    search3d.Add(SCLineSegment3d{SCPoint3d{3.0, 0.0, 0.0}, SCPoint3d{4.0, 0.0, 0.0}});
    const auto hitsSearch3d = search3d.QueryKNearest(SCPoint3d{0.25, 0.0, 0.0}, 1);
    ASSERT_EQ(hitsSearch3d.size(), 1U);
    EXPECT_EQ(hitsSearch3d[0].id, 0U);
}

TEST(MissingAlgorithmsCapabilityTest, QueryKNearestBreaksEqualDistanceTiesById)
{
    SCBoxTree2d tree2d;
    tree2d.Add(30, SCBox2d::FromMinMax(SCPoint2d{2.0, 0.0}, SCPoint2d{3.0, 1.0}));
    tree2d.Add(10, SCBox2d::FromMinMax(SCPoint2d{-3.0, 0.0}, SCPoint2d{-2.0, 1.0}));
    tree2d.Add(20, SCBox2d::FromMinMax(SCPoint2d{-0.5, 2.0}, SCPoint2d{0.5, 3.0}));

    const auto hits2d = tree2d.QueryKNearest(SCPoint2d{0.0, 0.0}, 2);
    ASSERT_EQ(hits2d.size(), 2U);
    EXPECT_EQ(hits2d[0].id, 10U);
    EXPECT_EQ(hits2d[1].id, 20U);
    EXPECT_DOUBLE_EQ(hits2d[0].distanceSquared, hits2d[1].distanceSquared);

    SCBoxTree3d tree3d;
    tree3d.Add(30, SCBox3d::FromMinMax(SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{3.0, 1.0, 1.0}));
    tree3d.Add(10, SCBox3d::FromMinMax(SCPoint3d{-3.0, 0.0, 0.0}, SCPoint3d{-2.0, 1.0, 1.0}));
    tree3d.Add(20, SCBox3d::FromMinMax(SCPoint3d{-0.5, 2.0, 0.0}, SCPoint3d{0.5, 3.0, 1.0}));

    const auto hits3d = tree3d.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 2);
    ASSERT_EQ(hits3d.size(), 2U);
    EXPECT_EQ(hits3d[0].id, 10U);
    EXPECT_EQ(hits3d[1].id, 20U);
    EXPECT_DOUBLE_EQ(hits3d[0].distanceSquared, hits3d[1].distanceSquared);
}

TEST(MissingAlgorithmsCapabilityTest, IntersectExtendedLineArcFindsExtendedHit)
{
    const SCLineSegment2d lineSegment{SCPoint2d{3.0, 3.0}, SCPoint2d{4.0, 4.0}};
    const SCArcSegment2d arc{SCPoint2d{0.0, 0.0}, std::sqrt(8.0), 0.0, std::numbers::pi / 2.0};

    const auto withoutExtension = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None);
    EXPECT_FALSE(withoutExtension.HasIntersection());

    const auto extended = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_TRUE(extended.HasIntersection());
    EXPECT_EQ(extended.kind, SCIntersectionKind2d::Point);
    ASSERT_EQ(extended.pointCount, 1U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(extended.points[0].point, (SCPoint2d{2.0, 2.0}), 1e-12);
    EXPECT_FALSE(extended.onFirstSegment);
    EXPECT_TRUE(extended.onSecondSegment);
}

TEST(MissingAlgorithmsCapabilityTest, IntersectExtendedLineArcUsesCallerEpsilonForArcBoundary)
{
    const double delta = 5e-7;
    const double eps = 1e-6;
    const double angle = std::numbers::pi / 2.0 + delta;
    const SCPoint2d tangentPoint{std::cos(angle), std::sin(angle)};
    const Geometry::SCVector2d tangentDirection{-std::sin(angle), std::cos(angle)};
    const SCLineSegment2d lineSegment{tangentPoint - tangentDirection * 0.5, tangentPoint + tangentDirection * 0.5};
    const SCArcSegment2d arc{SCPoint2d{0.0, 0.0}, 1.0, 0.0, std::numbers::pi / 2.0};

    const auto strict = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None, 1e-9);
    EXPECT_FALSE(strict.HasIntersection());

    const auto relaxed = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None, eps);
    ASSERT_TRUE(relaxed.HasIntersection());
    EXPECT_EQ(relaxed.kind, SCIntersectionKind2d::Tangent);
    ASSERT_EQ(relaxed.pointCount, 1U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(relaxed.points[0].point, tangentPoint, 1e-12);
    EXPECT_TRUE(relaxed.onFirstSegment);
    EXPECT_TRUE(relaxed.onSecondSegment);
}

TEST(MissingAlgorithmsCapabilityTest, ArcArcExtendedIntersectionHonorsPerSideExtension)
{
    const SCArcSegment2d first{SCPoint2d{0.0, 0.0}, 5.0, 0.0, std::numbers::pi / 4.0};
    const SCArcSegment2d second{SCPoint2d{0.0, 0.0}, 5.0, std::numbers::pi / 2.0, std::numbers::pi / 6.0};

    const auto withoutExtension = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::None);
    EXPECT_FALSE(withoutExtension.HasIntersection());

    const auto extendFirst = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_TRUE(extendFirst.HasIntersection());
    EXPECT_EQ(extendFirst.kind, SCIntersectionKind2d::Overlap);
    ASSERT_EQ(extendFirst.pointCount, 2U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(extendFirst.points[0].point, (SCPoint2d{0.0, 5.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(extendFirst.points[1].point, second.EndPoint(), 1e-12);
    EXPECT_FALSE(extendFirst.onFirstSegment);
    EXPECT_TRUE(extendFirst.onSecondSegment);
}

TEST(MissingAlgorithmsCapabilityTest, SegmentSearch3dNearestUsesProvidedTolerance)
{
    SCSegmentSearch3d search;
    const SCLine3d line = SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, Geometry::SCVector3d{1e-6, 0.0, 0.0});
    search.Add(SCLineCurve3d::FromLine(line, Geometry::SCIntervald{0.0, 1.0}));

    const auto strictHit = search.Nearest(SCPoint3d{0.0, 1.0, 0.0}, SCGeometryTolerance3d{1e-9, 1e-9, 1e-9});
    ASSERT_TRUE(strictHit.has_value());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(strictHit->point, (SCPoint3d{0.0, 0.0, 0.0}), 1e-12);

    const auto looseHit = search.Nearest(SCPoint3d{0.0, 1.0, 0.0}, SCGeometryTolerance3d{1e-3, 1e-9, 1e-9});
    EXPECT_FALSE(looseHit.has_value());
}

TEST(MissingAlgorithmsCapabilityTest, PolylineIntersectionReturnsSharedCrossing)
{
    const SCPolyline2d first(std::vector<SCPoint2d>{SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{4.0, 0.0}});
    const SCPolyline2d second(std::vector<SCPoint2d>{SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0}});

    const auto intersections = Geometry::Intersect(first, second);
    ASSERT_EQ(intersections.size(), 1U);
    EXPECT_EQ(intersections[0].kind, SCIntersectionKind2d::Point);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(intersections[0].point, (SCPoint2d{2.0, 0.0}), 1e-12);
}

TEST(MissingAlgorithmsCapabilityTest, SnapPointToSegments3dFindsNearestSegment)
{
    const SCLineSegment3d first{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}};
    const SCLineSegment3d second{SCPoint3d{0.0, 3.0, 0.0}, SCPoint3d{2.0, 3.0, 0.0}};
    const std::array<const SCLineSegment3d*, 2> segments{&first, &second};
    const auto snapped = Geometry::SnapPointToSegments3d(SCPoint3d{1.0, 0.2, 0.0}, segments, 10.0);

    ASSERT_TRUE(snapped.snapped);
    EXPECT_EQ(snapped.segmentIndex, 0U);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(snapped.point, (SCPoint3d{1.0, 0.0, 0.0}), 1e-12);
}
