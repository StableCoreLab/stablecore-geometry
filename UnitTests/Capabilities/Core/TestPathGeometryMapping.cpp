#include <array>
#include <numbers>
#include <variant>

#include <gtest/gtest.h>

#include "Core/ArcOps3d.h"
#include "Core/PathGeometry.h"

using namespace Geometry;

namespace
{
    const SCPathGeometryTolerance kTolerance{1e-8, 1e-8, 1e-12};
}

TEST(PathGeometryTest, PreservesSourceLengthsAndZeroSegmentMapping)
{
    const std::array<SCPoint3d, 4> points{{{0,0,0}, {1,0,0}, {1,0,0}, {3,0,0}}};
    const auto result = SimplifyPolylineWithMapping(points, kTolerance, SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.simplifiedSegments.size(), 1u);
    ASSERT_EQ(result.sourceMappings.size(), 3u);
    EXPECT_EQ(result.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_TRUE(result.sourceMappings[1].exactZeroLength);
    EXPECT_FALSE(result.sourceMappings[1].simplifiedSegmentIndex.has_value());
    EXPECT_EQ(result.sourceMappings[2].simplifiedSegmentIndex, 0u);
    EXPECT_DOUBLE_EQ(result.coverages[0].coveredTrueLength, 3.0);
    EXPECT_DOUBLE_EQ(result.sourceCumulativeLengths[3], 3.0);
}

TEST(PathGeometryTest, ClosedRepeatedEndpointKeepsOriginalSegmentNumbering)
{
    const std::array<SCPoint3d, 5> points{{{0,0,0}, {2,0,0}, {2,2,0},
                                            {0,2,0}, {0,0,0}}};
    const auto measured = MeasurePolyline3d(points, SCPolylineClosure::Closed, kTolerance);
    ASSERT_TRUE(measured.success);
    ASSERT_EQ(measured.segmentLengths.size(), 5u);
    EXPECT_DOUBLE_EQ(measured.segmentLengths[4], 0.0);
    EXPECT_DOUBLE_EQ(measured.totalLength, 8.0);

    const auto simplified = SimplifyPolylineWithMapping(points, kTolerance,
                                                        SCPolylineClosure::Closed);
    ASSERT_TRUE(simplified.success);
    EXPECT_EQ(simplified.closure, SCPolylineClosure::Closed);
    EXPECT_EQ(simplified.simplifiedPoints.size(), 4u);
    EXPECT_EQ(simplified.simplifiedSegments.size(), 4u);
    ASSERT_EQ(simplified.sourceMappings.size(), 5u);
    EXPECT_TRUE(simplified.sourceMappings[4].exactZeroLength);
    EXPECT_FALSE(simplified.sourceMappings[4].simplifiedSegmentIndex.has_value());
    EXPECT_DOUBLE_EQ(simplified.sourceTotalLength, 8.0);
}

TEST(PathGeometryTest, SimplificationChecksTheWholeMergeRange)
{
    const SCPathGeometryTolerance tolerance{1e-3, 1e-2, 1e-12};
    const std::array<SCPoint3d, 5> points{{{0,0,0}, {1,0,0}, {2,0.00075,0},
                                            {3,0.00225,0}, {4,0.0045,0}}};
    const auto result = SimplifyPolylineWithMapping(points, tolerance,
                                                    SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.simplifiedSegments.size(), 2u);
    ASSERT_EQ(result.coverages.size(), 2u);
    EXPECT_EQ(result.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_EQ(result.sourceMappings[2].simplifiedSegmentIndex, 0u);
    EXPECT_EQ(result.sourceMappings[3].simplifiedSegmentIndex, 1u);
    EXPECT_EQ(result.coverages[0].firstSourceSegmentIndex, 0u);
    EXPECT_EQ(result.coverages[0].lastSourceSegmentIndex, 2u);
    EXPECT_NEAR(result.sourceTotalLength,
                result.coverages[0].coveredTrueLength + result.coverages[1].coveredTrueLength,
                1e-12);
}

TEST(PathGeometryTest, ShortPositiveFoldRemainsMappedAndClosedSeamDoesNotMerge)
{
    const std::array<SCPoint3d, 4> folded{{{0,0,0}, {1,0,0},
                                            {1,5e-9,0}, {2,5e-9,0}}};
    const auto shortFold = SimplifyPolylineWithMapping(folded, kTolerance,
                                                       SCPolylineClosure::Open);
    ASSERT_TRUE(shortFold.success);
    ASSERT_EQ(shortFold.simplifiedSegments.size(), 3u);
    EXPECT_FALSE(shortFold.sourceMappings[1].exactZeroLength);
    EXPECT_EQ(shortFold.sourceMappings[1].simplifiedSegmentIndex, 1u);
    EXPECT_DOUBLE_EQ(shortFold.sourceSegmentLengths[1], 5e-9);

    const std::array<SCPoint3d, 5> closed{{{1,0,0}, {2,0,0}, {2,1,0},
                                            {0,1,0}, {0,0,0}}};
    const auto seam = SimplifyPolylineWithMapping(closed, kTolerance,
                                                  SCPolylineClosure::Closed);
    ASSERT_TRUE(seam.success);
    ASSERT_EQ(seam.simplifiedSegments.size(), 5u);
    EXPECT_EQ(seam.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_EQ(seam.sourceMappings[4].simplifiedSegmentIndex, 4u);
}

TEST(PathGeometryTest, MixedLineArcPathKeepsArcAndTrueCoverage)
{
    const double pi = std::numbers::pi_v<double>;
    const std::array<SCPathSegment3d, 3> segments{
        SCLineSegment3d{{-1,0,0}, {0,0,0}},
        SCArcSegment3d{{0,1,0}, {0,0,1}, {0,-1,0}, 1.0, pi / 2.0},
        SCLineSegment3d{{1,1,0}, {2,1,0}}
    };
    const auto result = SimplifyPolylineWithMapping(segments, kTolerance,
                                                    SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.simplifiedSegments.size(), 3u);
    EXPECT_TRUE(std::holds_alternative<SCArcSegment3d>(result.simplifiedSegments[1]));
    EXPECT_EQ(result.sourceMappings[1].simplifiedSegmentIndex, 1u);
    EXPECT_NEAR(result.coverages[1].coveredTrueLength, pi / 2.0, 1e-12);
    EXPECT_NEAR(result.sourceTotalLength, 2.0 + pi / 2.0, 1e-12);
    EXPECT_TRUE(result.simplifiedPoints.empty());
}

TEST(PathGeometryTest, RejectsGapCreatedByRemovingZeroSegments)
{
    const SCPathGeometryTolerance loose{1.0, 1e-8, 1e-12};
    const std::array<SCPathSegment3d, 4> segments{
        SCLineSegment3d{{-2,0,0}, {0,0,0}},
        SCLineSegment3d{{0.75,0,0}, {0.75,0,0}},
        SCLineSegment3d{{1.5,0,0}, {1.5,0,0}},
        SCLineSegment3d{{2.25,0,0}, {2.25,3,0}}
    };
    const auto result = SimplifyPolylineWithMapping(segments, loose, SCPolylineClosure::Open);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::DiscontinuousPath);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 3u);
    EXPECT_TRUE(result.simplifiedSegments.empty());
    EXPECT_TRUE(result.sourceMappings.empty());
    EXPECT_TRUE(result.sourceCumulativeLengths.empty());
    EXPECT_DOUBLE_EQ(result.sourceTotalLength, 0.0);
    EXPECT_EQ(result.inputKind, SCPolylineSimplificationInputKind::SegmentSequence);
}

TEST(PathGeometryTest, ZeroSegmentPointsParticipateInMergeRangeCheck)
{
    const double t = kTolerance.distanceTolerance;
    const std::array<SCPathSegment3d, 5> segments{
        SCLineSegment3d{{0,0,0}, {1,0,0}},
        SCLineSegment3d{{1,0.75 * t,0}, {1,0.75 * t,0}},
        SCLineSegment3d{{1,1.5 * t,0}, {1,1.5 * t,0}},
        SCLineSegment3d{{1,0.75 * t,0}, {1,0.75 * t,0}},
        SCLineSegment3d{{1,0,0}, {2,0,0}}
    };
    const auto result = SimplifyPolylineWithMapping(segments, kTolerance, SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.simplifiedSegments.size(), 2u);
    EXPECT_EQ(result.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_EQ(result.sourceMappings[4].simplifiedSegmentIndex, 1u);
}

TEST(PathGeometryTest, ZeroSegmentsOnTheRangeMayBeSkippedForDirectionChecks)
{
    const std::array<SCPathSegment3d, 4> segments{
        SCLineSegment3d{{0,0,0}, {1,0,0}},
        SCLineSegment3d{{1,0,0}, {1,0,0}},
        SCLineSegment3d{{1,0,0}, {1,0,0}},
        SCLineSegment3d{{1,0,0}, {2,0,0}}
    };
    const auto result = SimplifyPolylineWithMapping(segments, kTolerance,
                                                    SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.simplifiedSegments.size(), 1u);
    EXPECT_EQ(result.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_EQ(result.sourceMappings[3].simplifiedSegmentIndex, 0u);
    EXPECT_TRUE(result.sourceMappings[1].exactZeroLength);
    EXPECT_TRUE(result.sourceMappings[2].exactZeroLength);
}

TEST(PathGeometryTest, SegmentSequenceAnchorsAtFirstRemainingSegment)
{
    const std::array<SCPathSegment3d, 2> segments{
        SCLineSegment3d{{2 + 5e-9,0,0}, {2 + 5e-9,0,0}},
        SCArcSegment3d{{0,0,0}, {0,0,1}, {1,0,0}, 2.0,
                       std::numbers::pi_v<double> / 2.0}
    };
    const auto result = SimplifyPolylineWithMapping(segments, kTolerance, SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.simplifiedSegments.size(), 1u);
    EXPECT_TRUE(result.sourceMappings[0].exactZeroLength);
    EXPECT_EQ(result.sourceMappings[1].simplifiedSegmentIndex, 0u);
    ASSERT_TRUE(std::holds_alternative<SCArcSegment3d>(result.simplifiedSegments[0]));
    EXPECT_NEAR(result.coverages[0].coveredTrueLength, std::numbers::pi_v<double>, 1e-12);
}

TEST(PathGeometryTest, RetainsShortPositiveThreeDimensionalSourceLength)
{
    const SCPathGeometryTolerance tight{1e-12, 1e-8, 1e-12};
    const std::array<SCPoint3d, 2> points{{{0,0,0}, {5e-10,0,0}}};
    const auto result = SimplifyPolylineWithMapping(points, tight, SCPolylineClosure::Open);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.sourceMappings.size(), 1u);
    EXPECT_FALSE(result.sourceMappings[0].exactZeroLength);
    EXPECT_EQ(result.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_DOUBLE_EQ(result.sourceSegmentLengths[0], 5e-10);
}

TEST(PathGeometryTest, RejectsInvalidFrameAndPseudoCircle)
{
    const SCPathGeometryTolerance loose{1e-8, 1.0, 0.5};
    const SCPlaneFrame3d skewed{{0,0,0}, {1,0,0}, {1,1,0}, {0,0,1}};
    const std::array<SCPoint3d, 1> points{{{1,2,3}}};
    EXPECT_EQ(ProjectToFrame(skewed, points, loose).failure, SCPathGeometryFailure::InvalidFrame);
    const SCArcSegment3d pseudo{{0,0,0}, {0,0,1}, {1,1,0}, 2.0,
                                 std::numbers::pi_v<double> / 2.0};
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    EXPECT_EQ(ProjectArcToFrame(pseudo, xy, loose).failure,
              SCPathGeometryFailure::DegenerateDirection);
}
