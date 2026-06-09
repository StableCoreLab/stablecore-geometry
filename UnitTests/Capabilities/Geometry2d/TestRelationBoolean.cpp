#include <gtest/gtest.h>
#include <vector>

#include "Core/Boolean.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
#include "Support/GeometryTestSupport.h"

using Geometry::Difference;
using Geometry::Intersect;
using Geometry::SCLineSegment2d;
using Geometry::LocatePoint;
using Geometry::SCMultiPolygon2d;
using Geometry::SCPoint2d;
using Geometry::SCPointContainment2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Union;
using Geometry::Validate;

namespace
{
    double TotalArea(const SCMultiPolygon2d& polygons)
    {
        double total = 0.0;
        for (std::size_t i = 0; i < polygons.Count(); ++i)
        {
            total += polygons[i].Area();
        }
        return total;
    }
}  // namespace

TEST(RelationBooleanTest, CoversCurrentCapabilities)
{
    const SCLineSegment2d horizontal(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 0.0}, horizontal), SCPointContainment2d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 1.0}, horizontal), SCPointContainment2d::Outside);

    const SCPolyline2d squareRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                SCPolylineClosure::Closed);
    const SCPolygon2d square(squareRing);
    ASSERT_EQ(LocatePoint(SCPoint2d{2.0, 2.0}, square), SCPointContainment2d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint2d{5.0, 2.0}, square), SCPointContainment2d::Outside);
    ASSERT_TRUE(Validate(square).valid);

    const SCPolyline2d clipRing({SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 4.0}, SCPoint2d{2.0, 4.0}},
                              SCPolylineClosure::Closed);
    const SCPolygon2d clip(clipRing);
    const auto intersection = Intersect(square, clip);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(intersection), 8.0, 1e-9);

    const auto united = Union(square, clip);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(united), 24.0, 1e-9);

    const SCPolyline2d innerRing({SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 1.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{1.0, 3.0}},
                               SCPolylineClosure::Closed);
    const SCPolygon2d inner(innerRing);
    const auto difference = Difference(square, inner);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(difference), 12.0, 1e-9);

    const SCPolygon2d horizontalBar(SCPolyline2d(
        {SCPoint2d{0.0, 1.0}, SCPoint2d{4.0, 1.0}, SCPoint2d{4.0, 3.0}, SCPoint2d{0.0, 3.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d verticalBar(SCPolyline2d({SCPoint2d{1.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}, SCPoint2d{1.0, 4.0}},
                                           SCPolylineClosure::Closed));

    const auto crossUnion = Union(horizontalBar, verticalBar);
    ASSERT_GE(crossUnion.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(crossUnion), 12.0, 1e-9);

    const auto crossDifference = Difference(horizontalBar, verticalBar);
    ASSERT_GE(crossDifference.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(crossDifference), 4.0, 1e-9);

    const SCPolygon2d outer(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 6.0}, SCPoint2d{0.0, 6.0}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d nested(SCPolyline2d({SCPoint2d{2.0, 2.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{2.0, 4.0}},
                                      SCPolylineClosure::Closed));

    const auto containedUnion = Union(outer, nested);
    ASSERT_GE(containedUnion.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(containedUnion), 36.0, 1e-9);

    const auto containedIntersection = Intersect(outer, nested);
    ASSERT_GE(containedIntersection.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(containedIntersection), 4.0, 1e-9);

    const auto containedDifference = Difference(outer, nested);
    ASSERT_GE(containedDifference.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(containedDifference), 32.0, 1e-9);

    const SCPolygon2d overlapStripA(SCPolyline2d(
        {SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 2.0}, SCPoint2d{0.0, 2.0}}, SCPolylineClosure::Closed));
    const SCPolygon2d overlapStripB(SCPolyline2d(
        {SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}, SCPoint2d{6.0, 2.0}, SCPoint2d{2.0, 2.0}}, SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(overlapStripA, overlapStripB)), 4.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(overlapStripA, overlapStripB)), 12.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(overlapStripA, overlapStripB)), 4.0, 1e-9);

    const SCPolygon2d overlapFamilyA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                               SCPoint2d{8.0, 0.0},
                                               SCPoint2d{8.0, 2.0},
                                               SCPoint2d{5.0, 2.0},
                                               SCPoint2d{5.0, 4.0},
                                               SCPoint2d{0.0, 4.0}},
                                              SCPolylineClosure::Closed));
    const SCPolygon2d overlapFamilyB(SCPolyline2d({SCPoint2d{3.0, 0.0},
                                               SCPoint2d{10.0, 0.0},
                                               SCPoint2d{10.0, 3.0},
                                               SCPoint2d{6.0, 3.0},
                                               SCPoint2d{6.0, 5.0},
                                               SCPoint2d{3.0, 5.0}},
                                              SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(overlapFamilyA, overlapFamilyB)), 14.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(overlapFamilyA, overlapFamilyB)), 39.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(overlapFamilyA, overlapFamilyB)), 12.0, 1e-9);

    const SCPolygon2d nearDegenerateOverlapA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                                       SCPoint2d{8.0, 0.0},
                                                       SCPoint2d{8.0, 2.0},
                                                       SCPoint2d{5.0, 2.0},
                                                       SCPoint2d{5.0, 2.000001},
                                                       SCPoint2d{0.0, 2.000001}},
                                                      SCPolylineClosure::Closed));
    const SCPolygon2d nearDegenerateOverlapB(SCPolyline2d({SCPoint2d{3.0, 0.0},
                                                       SCPoint2d{10.0, 0.0},
                                                       SCPoint2d{10.0, 2.000001},
                                                       SCPoint2d{6.0, 2.000001},
                                                       SCPoint2d{6.0, 4.0},
                                                       SCPoint2d{3.0, 4.0}},
                                                      SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(nearDegenerateOverlapA, nearDegenerateOverlapB)), 10.000002, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(nearDegenerateOverlapA, nearDegenerateOverlapB)), 26.000007, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(nearDegenerateOverlapA, nearDegenerateOverlapB)), 6.000003, 1e-9);

    const SCPolygon2d ultraThinOverlapA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                                  SCPoint2d{8.0, 0.0},
                                                  SCPoint2d{8.0, 2.0},
                                                  SCPoint2d{5.0, 2.0},
                                                  SCPoint2d{5.0, 2.00000001},
                                                  SCPoint2d{0.0, 2.00000001}},
                                                 SCPolylineClosure::Closed));
    const SCPolygon2d ultraThinOverlapB(SCPolyline2d({SCPoint2d{3.0, 0.0},
                                                  SCPoint2d{10.0, 0.0},
                                                  SCPoint2d{10.0, 2.00000001},
                                                  SCPoint2d{6.0, 2.00000001},
                                                  SCPoint2d{6.0, 4.0},
                                                  SCPoint2d{3.0, 4.0}},
                                                 SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(ultraThinOverlapA, ultraThinOverlapB)), 10.00000002, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(ultraThinOverlapA, ultraThinOverlapB)), 26.00000007, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(ultraThinOverlapA, ultraThinOverlapB)), 6.00000003, 1e-9);

    const auto equalIntersection = Intersect(square, square);
    ASSERT_GE(equalIntersection.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(equalIntersection), 16.0, 1e-9);

    const auto equalDifference = Difference(square, square);
    ASSERT_EQ(equalDifference.Count(), 0);

    const SCPolygon2d disjointOther(SCPolyline2d(
        {SCPoint2d{10.0, 0.0}, SCPoint2d{12.0, 0.0}, SCPoint2d{12.0, 2.0}, SCPoint2d{10.0, 2.0}}, SCPolylineClosure::Closed));
    const auto disjointUnion = Union(square, disjointOther);
    ASSERT_GE(disjointUnion.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(disjointUnion), 20.0, 1e-9);

    const SCPolygon2d edgeTouch(SCPolyline2d({SCPoint2d{4.0, 1.0}, SCPoint2d{6.0, 1.0}, SCPoint2d{6.0, 3.0}, SCPoint2d{4.0, 3.0}},
                                         SCPolylineClosure::Closed));
    const auto touchingDifference = Difference(square, edgeTouch);
    ASSERT_GE(touchingDifference.Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(touchingDifference), 16.0, 1e-9);

    const SCPolygon2d duplicateEdgeFamilyA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                                     SCPoint2d{6.0, 0.0},
                                                     SCPoint2d{6.0, 1.0},
                                                     SCPoint2d{4.0, 1.0},
                                                     SCPoint2d{4.0, 1.000000001},
                                                     SCPoint2d{6.0, 1.000000001},
                                                     SCPoint2d{6.0, 4.0},
                                                     SCPoint2d{0.0, 4.0}},
                                                    SCPolylineClosure::Closed));
    const SCPolygon2d duplicateEdgeFamilyB(SCPolyline2d(
        {SCPoint2d{2.0, -1.0}, SCPoint2d{8.0, -1.0}, SCPoint2d{8.0, 3.0}, SCPoint2d{2.0, 3.0}}, SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 12.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 36.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 12.0, 1e-8);

    const SCPolygon2d repeatedCollinearChainA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                                        SCPoint2d{2.0, 0.0},
                                                        SCPoint2d{4.0, 0.0},
                                                        SCPoint2d{6.0, 0.0},
                                                        SCPoint2d{6.0, 3.0},
                                                        SCPoint2d{4.0, 3.0},
                                                        SCPoint2d{2.0, 3.0},
                                                        SCPoint2d{0.0, 3.0}},
                                                       SCPolylineClosure::Closed));
    const SCPolygon2d repeatedCollinearChainB(SCPolyline2d({SCPoint2d{3.0, -1.0},
                                                        SCPoint2d{7.0, -1.0},
                                                        SCPoint2d{7.0, 2.0},
                                                        SCPoint2d{5.0, 2.0},
                                                        SCPoint2d{5.0, 2.000000001},
                                                        SCPoint2d{3.0, 2.000000001}},
                                                       SCPolylineClosure::Closed));
    GEOMETRY_TEST_ASSERT_NEAR(
        TotalArea(Intersect(repeatedCollinearChainA, repeatedCollinearChainB)), 6.000000003, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(repeatedCollinearChainA, repeatedCollinearChainB)), 24.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(
        TotalArea(Difference(repeatedCollinearChainA, repeatedCollinearChainB)), 11.999999997, 1e-8);
}

TEST(RelationBooleanTest, HandlesHigherDegreeRepeatedCollinearFamilyOverlap)
{
    constexpr double delta = 1e-9;

    const SCPolygon2d repeatedFamilyA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                                SCPoint2d{6.0, 0.0},
                                                SCPoint2d{6.0, 4.0},
                                                SCPoint2d{5.0, 4.0},
                                                SCPoint2d{5.0, 4.0 - delta},
                                                SCPoint2d{4.0, 4.0 - delta},
                                                SCPoint2d{4.0, 4.0},
                                                SCPoint2d{3.0, 4.0},
                                                SCPoint2d{3.0, 4.0 - delta},
                                                SCPoint2d{2.0, 4.0 - delta},
                                                SCPoint2d{2.0, 4.0},
                                                SCPoint2d{0.0, 4.0}},
                                               SCPolylineClosure::Closed));
    const SCPolygon2d repeatedFamilyB(SCPolyline2d(
        {SCPoint2d{3.0, -1.0}, SCPoint2d{5.0, -1.0}, SCPoint2d{5.0, 5.0}, SCPoint2d{3.0, 5.0}}, SCPolylineClosure::Closed));

    ASSERT_TRUE(Validate(repeatedFamilyA).valid);
    ASSERT_TRUE(Validate(repeatedFamilyB).valid);

    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(repeatedFamilyA, repeatedFamilyB)), 8.0 - delta, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(repeatedFamilyA, repeatedFamilyB)), 28.0 - delta, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(repeatedFamilyA, repeatedFamilyB)), 16.0 - delta, 1e-8);
}

TEST(RelationBooleanTest, HandlesNearDegenerateIntersectionClusters)
{
    constexpr double delta = 1e-8;

    const SCPolygon2d clusterA(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                         SCPoint2d{8.0, 0.0},
                                         SCPoint2d{8.0, 4.0 - delta},
                                         SCPoint2d{8.0, 4.0 + delta},
                                         SCPoint2d{8.0, 8.0},
                                         SCPoint2d{0.0, 8.0}},
                                        SCPolylineClosure::Closed));
    const SCPolygon2d clusterB(SCPolyline2d({SCPoint2d{3.0, -1.0},
                                         SCPoint2d{5.0, -1.0},
                                         SCPoint2d{5.0, 4.0 - delta},
                                         SCPoint2d{5.0, 4.0 + delta},
                                         SCPoint2d{5.0, 9.0},
                                         SCPoint2d{3.0, 9.0}},
                                        SCPolylineClosure::Closed));

    ASSERT_TRUE(Validate(clusterA).valid);
    ASSERT_TRUE(Validate(clusterB).valid);

    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(clusterA, clusterB)), 16.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(clusterA, clusterB)), 68.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(clusterA, clusterB)), 48.0, 1e-8);
}

TEST(RelationBooleanTest, HandlesBelowToleranceArrangementDegeneracies)
{
    constexpr double delta = 1e-12;

    const SCPolygon2d first(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                      SCPoint2d{8.0, 0.0},
                                      SCPoint2d{8.0, 2.0},
                                      SCPoint2d{5.0, 2.0},
                                      SCPoint2d{5.0, 2.0 + delta},
                                      SCPoint2d{0.0, 2.0 + delta}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d({SCPoint2d{3.0, 0.0},
                                       SCPoint2d{10.0, 0.0},
                                       SCPoint2d{10.0, 2.0 + delta},
                                       SCPoint2d{6.0, 2.0 + delta},
                                       SCPoint2d{6.0, 4.0},
                                       SCPoint2d{3.0, 4.0}},
                                      SCPolylineClosure::Closed));

    ASSERT_GE(Intersect(first, second).Count(), 1);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Intersect(first, second)), 10.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Union(first, second)), 26.0, 1e-8);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(Difference(first, second)), 6.0, 1e-8);
}

