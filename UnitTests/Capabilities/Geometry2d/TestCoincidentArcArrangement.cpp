#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "../../../Source/Core/Detail/CurveArrangement2d.h"
#include "../../../Source/Detail/RingIntegral2d.h"
#include "Geometry.h"

namespace
{
    using Rings = std::vector<std::vector<std::unique_ptr<Geometry::ISCSegment2d>>>;

    Rings OverlappingArcs(bool reverseSecond)
    {
        Rings rings(2);
        rings[0].push_back(std::make_unique<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0.0, 0.0}, 1.0, 0.0, 1.5 * Geometry::kPi));
        rings[1].push_back(std::make_unique<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0.0, 0.0}, 1.0,
            reverseSecond ? 2.5 * Geometry::kPi : Geometry::kPi,
            (reverseSecond ? -1.5 : 1.5) * Geometry::kPi));
        return rings;
    }

    void ExpectQuarterCircleArrangement(const Geometry::Detail::CurveArrangementResult2d& result)
    {
        ASSERT_TRUE(result.success);
        EXPECT_TRUE(result.hasOverlap);
        EXPECT_FALSE(result.hasAmbiguousEvent);
        ASSERT_EQ(result.pieces.size(), 4U);
        double totalLength = 0.0;
        for (const auto& piece : result.pieces)
        {
            ASSERT_NE(piece.segment, nullptr);
            EXPECT_EQ(piece.segment->Kind(), Geometry::SCSegmentKind2d::Arc);
            EXPECT_NEAR(piece.segment->Length(), Geometry::kPi / 2.0, 1e-10);
            totalLength += piece.segment->Length();
            EXPECT_LT(piece.sourceStart, piece.sourceEnd);
            EXPECT_LE(piece.sourceRing, 1U);
            EXPECT_EQ(piece.sourceSegment, 0U);
        }
        EXPECT_NEAR(totalLength, Geometry::kTwoPi, 1e-10);
        ASSERT_EQ(result.faceRings.size(), 2U);
        double signedArea = 0.0;
        for (const auto& face : result.faceRings)
        {
            const double area = Geometry::Detail::ComputeSignedArea(face);
            EXPECT_NEAR(std::abs(area), Geometry::kPi, 1e-10);
            signedArea += area;
        }
        EXPECT_NEAR(signedArea, 0.0, 1e-10);
    }

    Geometry::SCPolygon2d Circle(double start, bool reversed)
    {
        const double direction = reversed ? -1.0 : 1.0;
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments;
        segments.push_back(std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0.0, 0.0}, 1.0, start, direction * 1.5 * Geometry::kPi));
        segments.push_back(std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0.0, 0.0}, 1.0, start + direction * 1.5 * Geometry::kPi,
            direction * 0.5 * Geometry::kPi));
        return Geometry::SCPolygon2d(Geometry::SCPolyline2d(std::move(segments), Geometry::SCPolylineClosure::Closed));
    }

    Geometry::SCPolygon2d MajorCircularSegment(double start)
    {
        auto arc = std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0.0, 0.0}, 1.0, start, 1.5 * Geometry::kPi);
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments{
            arc, std::make_shared<Geometry::SCLineSegment2d>(arc->EndPoint(), arc->StartPoint())};
        return Geometry::SCPolygon2d(Geometry::SCPolyline2d(std::move(segments), Geometry::SCPolylineClosure::Closed));
    }
}

TEST(CoincidentArcArrangementTest, GeneralArrangementSplitsEveryOverlapBoundary)
{
    for (const bool reverse : {false, true})
    {
        SCOPED_TRACE(reverse);
        const auto rings = OverlappingArcs(reverse);
        ExpectQuarterCircleArrangement(Geometry::Detail::BuildCurveArrangement2d(rings, 1e-9));
    }
}

TEST(CoincidentArcArrangementTest, CheckedArrangementSplitsEveryOverlapBoundary)
{
    for (const bool reverse : {false, true})
    {
        SCOPED_TRACE(reverse);
        const auto rings = OverlappingArcs(reverse);
        ExpectQuarterCircleArrangement(Geometry::Detail::BuildPositiveAreaCurveArrangement2d(rings, 1e-9));
    }
}

TEST(CoincidentArcArrangementTest, EqualCirclesWithDifferentSeamsClassifyAsIntersecting)
{
    const auto first = Circle(0.0, false);
    ASSERT_TRUE(Geometry::NormalizePolygon(first, 1e-9).success);
    for (const bool reversed : {false, true})
    {
        const auto second = Circle(Geometry::kPi, reversed);
        ASSERT_TRUE(Geometry::NormalizePolygon(second, 1e-9).success);
        for (const auto& result : {Geometry::ClassifyContainment(first, second, 1e-9),
                                   Geometry::ClassifyContainment(second, first, 1e-9)})
        {
            ASSERT_TRUE(result.success);
            EXPECT_EQ(result.failure, Geometry::SCPolygonTopologyFailure::None);
            EXPECT_EQ(result.containment, Geometry::SCPolygonContainment::Intersecting);
        }
    }
}

TEST(CoincidentArcArrangementTest, DisconnectedSharedArcsHavePositiveFilledAreaIntersection)
{
    const auto first = MajorCircularSegment(0.0);
    const auto second = MajorCircularSegment(Geometry::kPi);
    ASSERT_TRUE(Geometry::NormalizePolygon(first, 1e-9).success);
    ASSERT_TRUE(Geometry::NormalizePolygon(second, 1e-9).success);
    for (const auto& result : {Geometry::QueryPolygonPositiveAreaIntersection(first, second, 1e-9),
                               Geometry::QueryPolygonPositiveAreaIntersection(second, first, 1e-9)})
    {
        ASSERT_TRUE(result.success);
        EXPECT_EQ(result.failure, Geometry::SCPolygonPositiveAreaIntersectionFailure2d::None);
        EXPECT_TRUE(result.hasPositiveAreaIntersection);
    }
}

TEST(CoincidentArcArrangementTest, HoleOverlapDiagnosticIncludesIntervalsAfterIsolatedContact)
{
    const auto hole = [](double start) {
        auto arc = std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0, 0}, 1.0, start, -1.5 * Geometry::kPi);
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>> segments{
            arc, std::make_shared<Geometry::SCLineSegment2d>(arc->EndPoint(), arc->StartPoint())};
        return Geometry::SCPolyline2d(std::move(segments), Geometry::SCPolylineClosure::Closed);
    };
    const Geometry::SCPolyline2d outer({{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}, Geometry::SCPolylineClosure::Closed);
    const Geometry::SCPolygon2d polygon(outer, {hole(0.0), hole(-Geometry::kPi / 2.0)});
    const auto result = Geometry::NormalizePolygon(polygon, 1e-9);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.failure, Geometry::SCPolygonTopologyFailure::BoundaryOverlap);
}
