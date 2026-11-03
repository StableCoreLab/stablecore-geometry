#include <gtest/gtest.h>

#include "Core/Boolean.h"
#include "Core/Validation.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/PolygonBooleanFixtures2d.h"
#include "Types/Geometry2d/SCPoint2.h"

using Geometry::Difference;
using Geometry::Intersect;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Union;
using Geometry::Validate;
using Geometry::Test::TotalArea;

TEST(RelationBooleanTest, HandlesSteppedOverlapFamily)
{
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
    EXPECT_NEAR(TotalArea(Intersect(overlapFamilyA, overlapFamilyB)), 14.0, 1e-9);
    EXPECT_NEAR(TotalArea(Union(overlapFamilyA, overlapFamilyB)), 39.0, 1e-9);
    EXPECT_NEAR(TotalArea(Difference(overlapFamilyA, overlapFamilyB)), 12.0, 1e-9);
}

TEST(RelationBooleanTest, HandlesNearDegenerateOverlapFamily)
{
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
    EXPECT_NEAR(TotalArea(Intersect(nearDegenerateOverlapA, nearDegenerateOverlapB)), 10.000002, 1e-9);
    EXPECT_NEAR(TotalArea(Union(nearDegenerateOverlapA, nearDegenerateOverlapB)), 26.000007, 1e-9);
    EXPECT_NEAR(TotalArea(Difference(nearDegenerateOverlapA, nearDegenerateOverlapB)), 6.000003, 1e-9);
}

TEST(RelationBooleanTest, HandlesUltraThinOverlapFamily)
{
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
    EXPECT_NEAR(TotalArea(Intersect(ultraThinOverlapA, ultraThinOverlapB)), 10.00000002, 1e-9);
    EXPECT_NEAR(TotalArea(Union(ultraThinOverlapA, ultraThinOverlapB)), 26.00000007, 1e-9);
    EXPECT_NEAR(TotalArea(Difference(ultraThinOverlapA, ultraThinOverlapB)), 6.00000003, 1e-9);
}

TEST(RelationBooleanTest, HandlesEqualPolygonIntersectionAndDifference)
{
    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                          SCPolylineClosure::Closed));
    const auto equalIntersection = Intersect(square, square);
    ASSERT_GE(equalIntersection.Size(), 1);
    EXPECT_NEAR(TotalArea(equalIntersection), 16.0, 1e-9);

    const auto equalDifference = Difference(square, square);
    ASSERT_EQ(equalDifference.Size(), 0);
}

TEST(RelationBooleanTest, UnitesDisjointPolygons)
{
    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                          SCPolylineClosure::Closed));
    const SCPolygon2d disjointOther(SCPolyline2d(
        {SCPoint2d{10.0, 0.0}, SCPoint2d{12.0, 0.0}, SCPoint2d{12.0, 2.0}, SCPoint2d{10.0, 2.0}}, SCPolylineClosure::Closed));
    const auto disjointUnion = Union(square, disjointOther);
    ASSERT_GE(disjointUnion.Size(), 1);
    EXPECT_NEAR(TotalArea(disjointUnion), 20.0, 1e-9);
}

TEST(RelationBooleanTest, PreservesDifferenceAtTouchingEdge)
{
    const SCPolygon2d square(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                          SCPolylineClosure::Closed));
    const SCPolygon2d edgeTouch(SCPolyline2d({SCPoint2d{4.0, 1.0}, SCPoint2d{6.0, 1.0}, SCPoint2d{6.0, 3.0}, SCPoint2d{4.0, 3.0}},
                                         SCPolylineClosure::Closed));
    const auto touchingDifference = Difference(square, edgeTouch);
    ASSERT_GE(touchingDifference.Size(), 1);
    EXPECT_NEAR(TotalArea(touchingDifference), 16.0, 1e-9);
}

TEST(RelationBooleanTest, HandlesNearDuplicateEdgeFamily)
{
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
    EXPECT_NEAR(TotalArea(Intersect(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 12.0, 1e-8);
    EXPECT_NEAR(TotalArea(Union(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 36.0, 1e-8);
    EXPECT_NEAR(TotalArea(Difference(duplicateEdgeFamilyA, duplicateEdgeFamilyB)), 12.0, 1e-8);
}

TEST(RelationBooleanTest, HandlesRepeatedCollinearChain)
{
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
    EXPECT_NEAR(
        TotalArea(Intersect(repeatedCollinearChainA, repeatedCollinearChainB)), 6.000000003, 1e-8);
    EXPECT_NEAR(TotalArea(Union(repeatedCollinearChainA, repeatedCollinearChainB)), 24.0, 1e-8);
    EXPECT_NEAR(
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

    EXPECT_NEAR(TotalArea(Intersect(repeatedFamilyA, repeatedFamilyB)), 8.0 - delta, 1e-8);
    EXPECT_NEAR(TotalArea(Union(repeatedFamilyA, repeatedFamilyB)), 28.0 - delta, 1e-8);
    EXPECT_NEAR(TotalArea(Difference(repeatedFamilyA, repeatedFamilyB)), 16.0 - delta, 1e-8);
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

    EXPECT_NEAR(TotalArea(Intersect(clusterA, clusterB)), 16.0, 1e-8);
    EXPECT_NEAR(TotalArea(Union(clusterA, clusterB)), 68.0, 1e-8);
    EXPECT_NEAR(TotalArea(Difference(clusterA, clusterB)), 48.0, 1e-8);
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

    ASSERT_GE(Intersect(first, second).Size(), 1);
    EXPECT_NEAR(TotalArea(Intersect(first, second)), 10.0, 1e-8);
    EXPECT_NEAR(TotalArea(Union(first, second)), 26.0, 1e-8);
    EXPECT_NEAR(TotalArea(Difference(first, second)), 6.0, 1e-8);
}
