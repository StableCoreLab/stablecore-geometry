#include <gtest/gtest.h>

#include "Core/PolygonTopology.h"
#include "Geometry2d/SCArcSegment2d.h"

using namespace Geometry;

namespace
{
    SCPolygon2d MakeSquare(double minX, double minY, double maxX, double maxY)
    {
        return SCPolygon2d(SCPolyline2d({{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}},
                                        SCPolylineClosure::Closed));
    }

    SCPolygon2d MakeCircle(const SCPoint2d& center, double radius)
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.push_back(std::make_shared<SCArcSegment2d>(center, radius, 0.0, kPi));
        segments.push_back(std::make_shared<SCArcSegment2d>(center, radius, kPi, kPi));
        return SCPolygon2d(SCPolyline2d(std::move(segments), SCPolylineClosure::Closed));
    }
}

TEST(PolygonTopologyTest, NormalizesReverseOrientedOuterAndHole)
{
    const SCPolygon2d polygon(
        SCPolyline2d({{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}}, SCPolylineClosure::Closed),
        {SCPolyline2d({{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}}, SCPolylineClosure::Closed)});
    const auto normalized = NormalizePolygon(polygon, 1e-9);
    ASSERT_TRUE(normalized.success);
    EXPECT_TRUE(normalized.polygon.IsValid());
    EXPECT_NEAR(normalized.polygon.Area(), 96.0, 1e-9);
}

TEST(PolygonTopologyTest, TessellatesAndTriangulatesWithStableMetadata)
{
    const auto polygon = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto first = TessellateAndTriangulatePolygon(polygon, 1e-9, 0.25);
    const auto second = TessellateAndTriangulatePolygon(polygon, 1e-9, 0.25);
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);
    ASSERT_EQ(first.polygon.vertices.size(), second.polygon.vertices.size());
    ASSERT_EQ(first.polygon.triangles, second.polygon.triangles);
    EXPECT_EQ(first.polygon.loopStartIndices.size(), 1U);
    for (std::size_t i = 0; i < first.polygon.vertices.size(); ++i)
    {
        EXPECT_EQ(first.polygon.vertices[i].stableVertexId, i);
        EXPECT_EQ(first.polygon.vertices[i].point, second.polygon.vertices[i].point);
    }
    for (int iteration = 0; iteration < 100; ++iteration)
    {
        const auto replay = TessellateAndTriangulatePolygon(polygon, 1e-9, 0.25);
        ASSERT_TRUE(replay.success);
        ASSERT_EQ(replay.polygon.vertices.size(), first.polygon.vertices.size());
        for (std::size_t i = 0; i < replay.polygon.vertices.size(); ++i)
        {
            EXPECT_EQ(replay.polygon.vertices[i].stableVertexId, first.polygon.vertices[i].stableVertexId);
            EXPECT_EQ(replay.polygon.vertices[i].loopIndex, first.polygon.vertices[i].loopIndex);
            EXPECT_EQ(replay.polygon.vertices[i].sourceSegmentIndex, first.polygon.vertices[i].sourceSegmentIndex);
            EXPECT_DOUBLE_EQ(replay.polygon.vertices[i].sourceParameter, first.polygon.vertices[i].sourceParameter);
            EXPECT_EQ(replay.polygon.vertices[i].point, first.polygon.vertices[i].point);
        }
        EXPECT_EQ(replay.polygon.triangles, first.polygon.triangles);
    }
}

TEST(PolygonTopologyTest, SupportsTwoArcCircleBoundary)
{
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{0.0, 0.0}, 2.0, 0.0, kPi));
    segments.push_back(std::make_shared<SCArcSegment2d>(SCPoint2d{0.0, 0.0}, 2.0, kPi, kPi));
    const SCPolygon2d circle(SCPolyline2d(std::move(segments), SCPolylineClosure::Closed));
    const auto result = TessellateAndTriangulatePolygon(circle, 1e-9, 0.05);
    ASSERT_TRUE(result.success) << "failure=" << static_cast<int>(result.failure);
    EXPECT_GT(result.polygon.vertices.size(), 4U);
    EXPECT_FALSE(result.polygon.triangles.empty());
}

TEST(PolygonTopologyTest, ClassifiesAndAppendsStrictlyContainedHole)
{
    const auto outer = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto candidate = MakeSquare(2.0, 2.0, 4.0, 4.0);
    const auto containment = ClassifyContainment(outer, candidate, 1e-9);
    ASSERT_TRUE(containment.success);
    EXPECT_EQ(containment.containment, SCPolygonContainment::StrictInside);
    const auto appended = AppendHole(outer, candidate.OuterRing(), 1e-9);
    ASSERT_TRUE(appended.success);
    EXPECT_EQ(appended.polygon.HoleCount(), 1U);
    EXPECT_NEAR(appended.polygon.Area(), 96.0, 1e-9);
}

TEST(PolygonTopologyTest, ClassifiesTouchingCrossingAndDisjointRegions)
{
    const auto container = MakeSquare(0.0, 0.0, 10.0, 10.0);
    EXPECT_EQ(ClassifyContainment(container, MakeSquare(10.0, 10.0, 12.0, 12.0), 1e-9).containment,
              SCPolygonContainment::Touching);
    EXPECT_EQ(ClassifyContainment(container, MakeSquare(8.0, -2.0, 12.0, 4.0), 1e-9).containment,
              SCPolygonContainment::Intersecting);
    EXPECT_EQ(ClassifyContainment(container, MakeSquare(20.0, 20.0, 22.0, 22.0), 1e-9).containment,
              SCPolygonContainment::Disjoint);
}

TEST(PolygonTopologyTest, ClassifiesCandidateInsideExistingHole)
{
    const auto container = SCPolygon2d(
        MakeSquare(0.0, 0.0, 10.0, 10.0).OuterRing(),
        {MakeSquare(2.0, 2.0, 8.0, 8.0).OuterRing()});
    const auto candidate = MakeSquare(3.0, 3.0, 4.0, 4.0);
    const auto result = ClassifyContainment(container, candidate, 1e-9);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.containment, SCPolygonContainment::InsideHole);
}

TEST(PolygonTopologyTest, TessellatesMultipleHolesWithAreaPreservation)
{
    const auto polygon = SCPolygon2d(
        MakeSquare(0.0, 0.0, 20.0, 20.0).OuterRing(),
        {MakeSquare(2.0, 2.0, 5.0, 5.0).OuterRing(), MakeSquare(12.0, 12.0, 17.0, 17.0).OuterRing()});
    const auto result = TessellateAndTriangulatePolygon(polygon, 1e-9, 0.1);
    ASSERT_TRUE(result.success) << "failure=" << static_cast<int>(result.failure);
    EXPECT_EQ(result.polygon.loopStartIndices.size(), 3U);
    EXPECT_EQ(result.polygon.triangles.size(), 14U);
    for (std::size_t i = 0; i < result.polygon.vertices.size(); ++i)
    {
        EXPECT_EQ(result.polygon.vertices[i].stableVertexId, i);
    }
    EXPECT_EQ(result.polygon.vertices[result.polygon.loopStartIndices[1]].stableVertexId,
              result.polygon.loopStartIndices[1]);
    EXPECT_EQ(result.polygon.vertices[result.polygon.loopStartIndices[2]].stableVertexId,
              result.polygon.loopStartIndices[2]);
}

TEST(PolygonTopologyTest, RejectsHoleTouchingOuterBoundary)
{
    const auto polygon = SCPolygon2d(
        MakeSquare(0.0, 0.0, 10.0, 10.0).OuterRing(),
        {MakeSquare(0.0, 2.0, 3.0, 5.0).OuterRing()});
    const auto result = NormalizePolygon(polygon, 1e-9);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.failure, SCPolygonTopologyFailure::BoundaryTouching);
}

TEST(PolygonTopologyTest, ReportsBoundaryOverlapAsIntersecting)
{
    const auto first = MakeSquare(0.0, 0.0, 10.0, 10.0);
    const auto second = SCPolygon2d(
        SCPolyline2d({{0.0, 0.0}, {10.0, 0.0}, {10.0, -2.0}, {0.0, -2.0}}, SCPolylineClosure::Closed));
    const auto result = ClassifyContainment(first, second, 1e-9);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.containment, SCPolygonContainment::Intersecting);
}

TEST(PolygonTopologyTest, ClassifiesCandidateContainingContainerAndCrossingHole)
{
    const auto container = SCPolygon2d(
        MakeSquare(0.0, 0.0, 10.0, 10.0).OuterRing(),
        {MakeSquare(3.0, 3.0, 7.0, 7.0).OuterRing()});
    const auto enclosing = MakeSquare(-2.0, -2.0, 12.0, 12.0);
    const auto crossingHole = MakeSquare(1.0, 1.0, 9.0, 9.0);
    EXPECT_EQ(ClassifyContainment(container, enclosing, 1e-9).containment,
              SCPolygonContainment::Intersecting);
    EXPECT_EQ(ClassifyContainment(container, crossingHole, 1e-9).containment,
              SCPolygonContainment::Intersecting);
}

TEST(PolygonTopologyTest, ClassifiesCurvedContainmentAndHoleRegions)
{
    const auto container = MakeSquare(0.0, 0.0, 20.0, 20.0);
    const auto strictInside = MakeCircle({5.0, 5.0}, 1.0);
    const auto crossing = MakeCircle({19.0, 10.0}, 2.0);
    const auto strictResult = ClassifyContainment(container, strictInside, 1e-9);
    ASSERT_TRUE(strictResult.success) << static_cast<int>(strictResult.failure);
    EXPECT_EQ(strictResult.containment,
              SCPolygonContainment::StrictInside);
    EXPECT_EQ(ClassifyContainment(container, crossing, 1e-9).containment,
              SCPolygonContainment::Intersecting);
}
