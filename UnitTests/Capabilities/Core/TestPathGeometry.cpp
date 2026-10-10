#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Core/ArcOps3d.h"
#include "Core/Offset.h"
#include "Core/PathGeometry.h"
#include "Geometry2d/SCLineSegment2d.h"

using namespace Geometry;

namespace
{
    const SCPathGeometryTolerance kTolerance{1e-8, 1e-8, 1e-12};
}

TEST(PathGeometryTest, FitsPlaneAndBuildsDeterministicFrame)
{
    const std::array<SCPoint3d, 4> points{{{0,0,2}, {2,0,2}, {2,3,2}, {0,3,2}}};
    const auto fit = FitPlane(points, kTolerance);
    ASSERT_TRUE(fit.success);
    EXPECT_NEAR(fit.plane.normal.z, 1.0, 1e-12);
    EXPECT_NEAR(fit.maxAbsoluteDeviation, 0.0, 1e-12);
    const auto frame = MakePlaneFrame(fit.plane, points, kTolerance);
    ASSERT_TRUE(frame.success);
    EXPECT_NEAR(frame.frame.origin.z, 2.0, 1e-12);
    EXPECT_NEAR(frame.frame.u.x, 1.0, 1e-12);
    EXPECT_NEAR(frame.frame.v.y, 1.0, 1e-12);
    const std::array<SCPoint3d, 1> offPlane{{{2,3,4}}};
    const auto projected = ProjectToFrame(frame.frame, offPlane, kTolerance);
    ASSERT_TRUE(projected.success);
    EXPECT_NEAR(projected.points[0].x, 2.0, 1e-12);
    EXPECT_NEAR(projected.points[0].y, 3.0, 1e-12);
    EXPECT_NEAR(projected.signedDistances[0], 2.0, 1e-12);
    const auto lifted = LiftFromFrame(frame.frame, projected.points, kTolerance);
    ASSERT_TRUE(lifted.success);
    EXPECT_NEAR(lifted.points[0].z, 2.0, 1e-12);
}

TEST(PathGeometryTest, KeepsThinPlaneAndCollinearFallback)
{
    const std::array<SCPoint3d, 3> thin{{{0,0,0}, {1e6,0,0}, {1e6,0.001,0}}};
    const auto fit = FitPlane(thin, kTolerance);
    ASSERT_TRUE(fit.success);
    EXPECT_NEAR(fit.plane.normal.z, 1.0, 1e-12);
    EXPECT_EQ(MakeCollinearFrame(thin, kTolerance).failure, SCPathGeometryFailure::NonCollinear);

    const std::array<SCPoint3d, 3> straight{{{0,0,0}, {1,0,0}, {3,0,0}}};
    EXPECT_EQ(FitPlane(straight, kTolerance).failure, SCPathGeometryFailure::RankDeficient);
    const auto frame = MakeCollinearFrame(straight, kTolerance);
    ASSERT_TRUE(frame.success);
    EXPECT_NEAR(frame.frame.u.x, 1.0, 1e-12);
    const auto lengths = MeasurePolyline3d(straight, SCPolylineClosure::Open, kTolerance);
    ASSERT_TRUE(lengths.success);
    EXPECT_DOUBLE_EQ(lengths.totalLength, 3.0);
}

TEST(PathGeometryTest, FitPreservesDuplicateWeightAndReportsAmbiguousNormal)
{
    const std::array<SCPoint3d, 4> weighted{{{0,0,0}, {2,0,0}, {0,2,0}, {0,0,0}}};
    const auto fit = FitPlane(weighted, kTolerance);
    ASSERT_TRUE(fit.success);
    EXPECT_NEAR(fit.plane.origin.x, 0.5, 1e-12);
    EXPECT_NEAR(fit.plane.origin.y, 0.5, 1e-12);

    const std::array<SCPoint3d, 4> nonplanar{{{0,0,0}, {2,0,0}, {0,2,0}, {0,0,0.1}}};
    const auto tilted = FitPlane(nonplanar, kTolerance);
    ASSERT_TRUE(tilted.success);
    EXPECT_GT(tilted.maxAbsoluteDeviation, 0.0);

    const std::array<SCPoint3d, 7> isotropic{{{0,0,0}, {1,0,0}, {-1,0,0},
                                               {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}}};
    EXPECT_EQ(FitPlane(isotropic, kTolerance).failure, SCPathGeometryFailure::AmbiguousPlane);
    const std::array<SCPoint3d, 2> insufficient{{{0,0,0}, {1,0,0}}};
    EXPECT_EQ(FitPlane(insufficient, kTolerance).failure,
              SCPathGeometryFailure::InsufficientPoints);
}

TEST(PathGeometryTest, ProjectionChecksHandednessAndEmptyBatch)
{
    const SCPlaneFrame3d reversed{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,-1}};
    const std::array<SCPoint3d, 1> point{{{1,2,3}}};
    EXPECT_EQ(ProjectToFrame(reversed, point, kTolerance).failure,
              SCPathGeometryFailure::InvalidFrame);

    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    EXPECT_TRUE(ProjectToFrame(xy, std::span<const SCPoint3d>{}, kTolerance).success);
    EXPECT_TRUE(LiftFromFrame(xy, std::span<const SCPoint2d>{}, kTolerance).success);
    const double infinity = std::numeric_limits<double>::infinity();
    const std::array<SCPoint3d, 2> invalid{{{0,0,0}, {infinity,0,0}}};
    const auto rejected = ProjectToFrame(xy, invalid, kTolerance);
    EXPECT_EQ(rejected.failure, SCPathGeometryFailure::NonFiniteInput);
    EXPECT_EQ(rejected.inputIndex, 1u);
}

TEST(PathGeometryTest, FrameSkipsRepeatedFirstPointAndKeepsWorldAxisTieOrder)
{
    const std::array<SCPoint3d, 4> planar{{{0,0,0}, {0,0,0},
                                            {1,0,0}, {0,1,0}}};
    const SCPlane scaledPlane{{0,0,0}, {0,0,1e200}};
    const auto planeFrame = MakePlaneFrame(scaledPlane, planar, kTolerance);
    ASSERT_TRUE(planeFrame.success);
    EXPECT_NEAR(planeFrame.frame.u.x, 1.0, 1e-12);
    EXPECT_NEAR(planeFrame.frame.n.z, 1.0, 1e-12);

    const std::array<SCPoint3d, 4> diagonal{{{0,0,0}, {0,0,0},
                                              {1,1,1}, {2,2,2}}};
    const auto lineFrame = MakeCollinearFrame(diagonal, kTolerance);
    ASSERT_TRUE(lineFrame.success);
    EXPECT_NEAR(lineFrame.frame.u.x, 1.0 / std::sqrt(3.0), 1e-12);
    EXPECT_GT(lineFrame.frame.v.x, 0.0);
    EXPECT_LT(lineFrame.frame.v.y, 0.0);
    EXPECT_LT(lineFrame.frame.v.z, 0.0);
}

TEST(PathGeometryTest, TranslatedCoordinatesPreserveFitAndProjection)
{
    const double base = 1e12;
    const std::array<SCPoint3d, 4> points{{{base,base,base}, {base + 2,base,base},
                                            {base + 2,base + 2,base}, {base,base + 2,base}}};
    const auto fit = FitPlane(points, kTolerance);
    ASSERT_TRUE(fit.success);
    EXPECT_DOUBLE_EQ(fit.plane.origin.x, base + 1.0);
    EXPECT_DOUBLE_EQ(fit.plane.origin.y, base + 1.0);
    const auto frame = MakePlaneFrame(fit.plane, points, kTolerance);
    ASSERT_TRUE(frame.success);
    const std::array<SCPoint3d, 1> offPlane{{{base + 1,base + 1,base + 4}}};
    const auto projected = ProjectToFrame(frame.frame, offPlane, kTolerance);
    ASSERT_TRUE(projected.success);
    EXPECT_DOUBLE_EQ(projected.points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(projected.points[0].y, 1.0);
    EXPECT_DOUBLE_EQ(projected.signedDistances[0], 4.0);
}

TEST(PathGeometryTest, ReversedPlaneRemainsRightHandedAndCoincidentInputFails)
{
    const std::array<SCPoint3d, 4> reversed{{{0,3,2}, {2,3,2}, {2,0,2}, {0,0,2}}};
    const auto fit = FitPlane(reversed, kTolerance);
    ASSERT_TRUE(fit.success);
    EXPECT_NEAR(fit.plane.normal.z, -1.0, 1e-12);
    const auto frame = MakePlaneFrame(fit.plane, reversed, kTolerance);
    ASSERT_TRUE(frame.success);
    EXPECT_NEAR(Dot(Cross(frame.frame.u, frame.frame.v), frame.frame.n), 1.0, 1e-12);

    const std::array<SCPoint3d, 3> coincident{{{1,1,1}, {1,1,1}, {1,1,1}}};
    EXPECT_EQ(FitPlane(coincident, kTolerance).failure,
              SCPathGeometryFailure::CoincidentPoints);
    EXPECT_EQ(MakeCollinearFrame(coincident, kTolerance).failure,
              SCPathGeometryFailure::CoincidentPoints);
    EXPECT_EQ(MeasurePolyline3d(coincident, SCPolylineClosure::Open, kTolerance).failure,
              SCPathGeometryFailure::CoincidentPoints);
}

TEST(PathGeometryTest, ReportsTrueLengthIndependentlyOfProjection)
{
    const std::array<SCPoint3d, 2> points{{{0,0,0}, {3,0,4}}};
    const auto length = MeasurePolyline3d(points, SCPolylineClosure::Open, kTolerance);
    ASSERT_TRUE(length.success);
    EXPECT_DOUBLE_EQ(length.totalLength, 5.0);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const auto projection = ProjectToFrame(xy, points, kTolerance);
    ASSERT_TRUE(projection.success);
    EXPECT_DOUBLE_EQ(projection.points[1].x, 3.0);
    EXPECT_DOUBLE_EQ(projection.signedDistances[1], 4.0);
}

TEST(PathGeometryTest, ReportsNonFiniteCumulativeLengthAtTheOverflowingSegment)
{
    const std::array<SCPoint3d, 3> points{{{0,0,0}, {1e308,0,0}, {0,0,0}}};
    const auto length = MeasurePolyline3d(points, SCPolylineClosure::Open, kTolerance);
    EXPECT_FALSE(length.success);
    EXPECT_EQ(length.failure, SCPathGeometryFailure::NonFiniteResult);
    EXPECT_EQ(length.inputIndex, 1u);
    EXPECT_TRUE(length.segmentLengths.empty());
    EXPECT_TRUE(length.cumulativeLengths.empty());
}

TEST(PathGeometryTest, ReportsNonFiniteSegmentConnectionDifference)
{
    const std::array<SCPathSegment3d, 2> segments{
        SCLineSegment3d{{1e308,0,0}, {1e308,1,0}},
        SCLineSegment3d{{-1e308,1,0}, {-1e308,2,0}}
    };
    const auto result = SimplifyPolylineWithMapping(segments, kTolerance,
                                                    SCPolylineClosure::Open);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::NonFiniteResult);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
    EXPECT_TRUE(result.simplifiedSegments.empty());
}

TEST(PathGeometryTest, ConstructsAndProjectsCircle)
{
    const double pi = std::numbers::pi_v<double>;
    const auto circle = CreateArcSegment3d({2,0,0}, {2,0,0}, {0,0,0}, {0,0,1},
                                            2.0, 2.0 * pi, kTolerance);
    ASSERT_TRUE(circle.success);
    EXPECT_NEAR(circle.trueLength, 4.0 * pi, 1e-12);
    EXPECT_NEAR(circle.fitSamples[1].y, 2.0, 1e-12);
    const std::array<SCPoint3d, 4> framePoints{{circle.fitSamples[0], circle.fitSamples[1],
                                                  circle.fitSamples[2], circle.fitSamples[3]}};
    const auto fit = FitPlane(framePoints, kTolerance);
    ASSERT_TRUE(fit.success);
    const auto frame = MakePlaneFrame(fit.plane, framePoints, kTolerance);
    ASSERT_TRUE(frame.success);
    const auto projected = ProjectArcToFrame(circle.arc, frame.frame, kTolerance);
    ASSERT_TRUE(projected.success);
    EXPECT_TRUE(projected.arc.IsValid());
    EXPECT_NEAR(projected.projectedLength2d, 4.0 * pi, 1e-12);

    const std::array<SCPathSegment3d, 1> segments{{SCPathSegment3d{circle.arc}}};
    const auto simplified = SimplifyPolylineWithMapping(segments, kTolerance,
                                                        SCPolylineClosure::Closed);
    ASSERT_TRUE(simplified.success);
    ASSERT_EQ(simplified.simplifiedSegments.size(), 1u);
    ASSERT_EQ(simplified.sourceMappings.size(), 1u);
    EXPECT_EQ(simplified.sourceMappings[0].simplifiedSegmentIndex, 0u);
    EXPECT_NEAR(simplified.coverages[0].coveredTrueLength, 4.0 * pi, 1e-12);
    EXPECT_NEAR(simplified.sourceTotalLength, 4.0 * pi, 1e-12);
}

TEST(PathGeometryTest, RejectsIncompatibleAndUnrepresentableArcProjection)
{
    const double pi = std::numbers::pi_v<double>;
    const auto quarter = CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0}, {0,0,1},
                                             2.0, pi / 2.0, kTolerance);
    ASSERT_TRUE(quarter.success);
    const SCPlaneFrame3d yz{{0,0,0}, {0,1,0}, {0,0,1}, {1,0,0}};
    EXPECT_EQ(ProjectArcToFrame(quarter.arc, yz, kTolerance).failure,
              SCPathGeometryFailure::IncompatiblePlanes);

    const SCPathGeometryTolerance tight{1e-12, 1e-12, 1e-12};
    const auto tinySweep = CreateArcSegment3d({2,0,0}, {2,1e-9,0}, {0,0,0}, {0,0,1},
                                               2.0, 5e-10, tight);
    ASSERT_TRUE(tinySweep.success);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    EXPECT_EQ(ProjectArcToFrame(tinySweep.arc, xy, tight).failure,
              SCPathGeometryFailure::Unrepresentable2d);
}

TEST(PathGeometryTest, RejectsArcWhoseOffPlaneMaximumFallsBetweenSamples)
{
    const double tilt = 4e-9;
    const double normalZ = std::sqrt(1.0 - 2.0 * tilt * tilt);
    const SCVector3d normal{tilt, tilt, normalZ};
    const double radialZ = -tilt / normalZ;
    const double radialScale = std::hypot(1.0, radialZ);
    const SCVector3d radial{1.0 / radialScale, 0.0, radialZ / radialScale};
    const SCArcSegment3d arc{{0,0,0}, normal, radial, 2.0,
                             2.0 * std::numbers::pi_v<double>};
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const SCVector3d tangent = Cross(normal, radial);
    for (int sample = 0; sample <= 4; ++sample)
    {
        const double angle = sample * std::numbers::pi_v<double> / 2.0;
        const double height = 2.0 * (radial.z * std::cos(angle) +
                                     tangent.z * std::sin(angle));
        EXPECT_LT(std::abs(height), kTolerance.distanceTolerance);
    }

    const auto projected = ProjectArcToFrame(arc, xy, kTolerance);
    EXPECT_EQ(projected.failure, SCPathGeometryFailure::IncompatiblePlanes);
}

TEST(PathGeometryTest, ReportsNonFinitePlaneFitDifference)
{
    const std::array<SCPoint3d, 3> points{{{1e308,0,0}, {-1e308,0,0},
                                            {1e308,1,0}}};
    EXPECT_EQ(FitPlane(points, kTolerance).failure,
              SCPathGeometryFailure::NonFiniteResult);
}

TEST(PathGeometryTest, SignedArcConstructionAndInvalidInputs)
{
    const double pi = std::numbers::pi_v<double>;
    const auto clockwise = CreateArcSegment3d({2,0,0}, {0,-2,0}, {0,0,0},
                                               {0,0,1}, 2.0, -pi / 2.0, kTolerance);
    ASSERT_TRUE(clockwise.success);
    EXPECT_NEAR(clockwise.trueLength, pi, 1e-12);
    EXPECT_LT(clockwise.samples[1].y, 0.0);
    EXPECT_EQ(CreateArcSegment3d({2,0,0}, {0,-2,0}, {0,0,0}, {0,0,1},
                                 2.0, pi / 2.0, kTolerance).failure,
              SCPathGeometryFailure::EndpointMismatch);
    EXPECT_EQ(CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0}, {0,0,1},
                                 0.0, pi / 2.0, kTolerance).failure,
              SCPathGeometryFailure::InvalidRadius);
    EXPECT_EQ(CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0}, {0,0,1},
                                 2.0, 3.0 * pi, kTolerance).failure,
              SCPathGeometryFailure::InvalidSweep);
    const auto overflow = CreateArcSegment3d({1e308,0,0}, {1e308,0,0}, {0,0,0},
                                              {0,0,1}, 1e308, 2.0 * pi, kTolerance);
    EXPECT_EQ(overflow.failure, SCPathGeometryFailure::NonFiniteResult);
    EXPECT_DOUBLE_EQ(overflow.trueLength, 0.0);
}

TEST(PathGeometryTest, ArcConstructionHandlesZeroCrossingAndNearFullSweep)
{
    const double pi = std::numbers::pi_v<double>;
    const double rootTwo = std::sqrt(2.0);
    const auto crossing = CreateArcSegment3d({rootTwo,-rootTwo,0},
                                              {rootTwo,rootTwo,0}, {0,0,0},
                                              {0,0,1}, 2.0, pi / 2.0, kTolerance);
    ASSERT_TRUE(crossing.success);
    EXPECT_NEAR(crossing.fitSamples[2].x, 2.0, 1e-12);
    EXPECT_NEAR(crossing.fitSamples[2].y, 0.0, 1e-12);

    const double sweep = 2.0 * pi - 1e-4;
    const auto nearFull = CreateArcSegment3d({2,0,0},
                                              {2 * std::cos(sweep), 2 * std::sin(sweep), 0},
                                              {0,0,0}, {0,0,1}, 2.0, sweep, kTolerance);
    ASSERT_TRUE(nearFull.success);
    EXPECT_NEAR(nearFull.trueLength, 2.0 * sweep, 1e-12);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const auto projected = ProjectArcToFrame(nearFull.arc, xy, kTolerance);
    ASSERT_TRUE(projected.success);
    EXPECT_NEAR(projected.arc.sweepAngle, sweep, 1e-12);
}

TEST(PathGeometryTest, ArcConstructionRejectsOffPlaneEndpoint)
{
    const auto result = CreateArcSegment3d({2,0,0}, {0,2,2e-8}, {0,0,0},
                                            {0,0,1}, 2.0,
                                            std::numbers::pi_v<double> / 2.0,
                                            kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::EndpointMismatch);
    EXPECT_EQ(result.inputIndex, 1u);
}

TEST(PathGeometryTest, ArcNormalNormalizationAndOppositeFrameOrientation)
{
    const double pi = std::numbers::pi_v<double>;
    const auto normalized = CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0},
                                                {0,0,1 + 5e-13}, 2.0, pi / 2.0,
                                                kTolerance);
    ASSERT_TRUE(normalized.success);
    EXPECT_NEAR(normalized.arc.unitNormal.z, 1.0, 1e-15);
    EXPECT_EQ(CreateArcSegment3d({2,0,0}, {0,2,0}, {0,0,0},
                                 {0,0,1.1}, 2.0, pi / 2.0, kTolerance).failure,
              SCPathGeometryFailure::InvalidNormal);

    const auto reversed = CreateArcSegment3d({2,0,0}, {0,-2,0}, {0,0,0},
                                              {0,0,-1}, 2.0, pi / 2.0, kTolerance);
    ASSERT_TRUE(reversed.success);
    const SCPlaneFrame3d xy{{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    const auto projected = ProjectArcToFrame(reversed.arc, xy, kTolerance);
    ASSERT_TRUE(projected.success);
    EXPECT_NEAR(projected.arc.sweepAngle, -pi / 2.0, 1e-12);
    EXPECT_NEAR(projected.trueLength3d, pi, 1e-12);
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

TEST(PathGeometryTest, StrictOffsetMapsJoinsAndRejectsInputCrossing)
{
    const SCPolyline2d elbow({SCPoint2d{0,0}, SCPoint2d{10,0}, SCPoint2d{10,10}},
                             SCPolylineClosure::Open);
    const auto offset = OffsetWithMapping(elbow, 1.0, {4.0}, kTolerance);
    ASSERT_TRUE(offset.success);
    ASSERT_EQ(offset.mappings.size(), 2u);
    EXPECT_NEAR(offset.mappings[0].sourceEndParameter, 0.9, 1e-12);
    EXPECT_NEAR(offset.mappings[1].sourceStartParameter, 0.1, 1e-12);
    EXPECT_NEAR(offset.outputTotalLength2d, 18.0, 1e-12);
    EXPECT_EQ(OffsetWithMapping(elbow, 1.0, {1.0}, kTolerance).failure,
              SCPathGeometryFailure::MiterLimitExceeded);

    const SCPolyline2d crossing({SCPoint2d{0,0}, SCPoint2d{10,0},
                                 SCPoint2d{0,1}, SCPoint2d{1,-1}}, SCPolylineClosure::Open);
    const auto rejected = OffsetWithMapping(crossing, -1.0, {40.0}, kTolerance);
    EXPECT_EQ(rejected.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(rejected.inputIndex, 0u);
    EXPECT_EQ(rejected.relatedIndex, 2u);
}

TEST(PathGeometryTest, SingleLineOffsetKeepsIdentityParametersAndLengths)
{
    const SCPolyline2d line({SCPoint2d{0,0}, SCPoint2d{10,0}}, SCPolylineClosure::Open);
    const auto result = OffsetWithMapping(line, 2.0, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.mappings.size(), 1u);
    ASSERT_EQ(result.outputCumulativeLengths.size(), 2u);
    EXPECT_DOUBLE_EQ(result.mappings[0].sourceStartParameter, 0.0);
    EXPECT_DOUBLE_EQ(result.mappings[0].sourceEndParameter, 1.0);
    EXPECT_DOUBLE_EQ(result.mappings[0].sourceLength2d, 10.0);
    EXPECT_DOUBLE_EQ(result.mappings[0].outputLength2d, 10.0);
    EXPECT_DOUBLE_EQ(result.outputCumulativeLengths[0], 0.0);
    EXPECT_DOUBLE_EQ(result.outputCumulativeLengths[1], 10.0);
    EXPECT_DOUBLE_EQ(result.inputTotalLength2d, 10.0);
    EXPECT_DOUBLE_EQ(result.outputTotalLength2d, 10.0);
    const auto shifted = result.polyline.SegmentAt(0);
    ASSERT_NE(shifted, nullptr);
    EXPECT_DOUBLE_EQ(shifted->StartPoint().y, 2.0);
    EXPECT_DOUBLE_EQ(shifted->EndPoint().y, 2.0);
}

TEST(PathGeometryTest, TwoArcClosedPathAllowsOnlyItsTwoDeclaredContacts)
{
    std::vector<std::shared_ptr<ISCSegment2d>> arcs;
    arcs.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, std::numbers::pi_v<double>));
    arcs.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, std::numbers::pi_v<double>, std::numbers::pi_v<double>));
    const SCPolyline2d circle(std::move(arcs), SCPolylineClosure::Closed);
    ASSERT_TRUE(circle.IsValid());
    const auto result = OffsetWithMapping(circle, 0.5, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.polyline.IsClosed());
    EXPECT_EQ(result.mappings.size(), 2u);
}

TEST(PathGeometryTest, WholeCircleCannotBeEmbeddedInLongerChain)
{
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, 2.0 * std::numbers::pi_v<double>));
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{2,0}, SCPoint2d{3,0}));
    const SCPolyline2d chain(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(chain.IsValid());
    const auto result = OffsetWithMapping(chain, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 0u);
}

TEST(PathGeometryTest, TopologyReportsEarliestPairBeforeLaterSelfClosure)
{
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{-2,0}, SCPoint2d{1,0}));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 1.0, 0.0, 2.0 * std::numbers::pi_v<double>));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
}

TEST(PathGeometryTest, AdjacentSegmentsRejectSecondContactNearDeclaredJoin)
{
    const double shift = 1e-5;
    const double radius = std::hypot(1.0, shift);
    const SCArcSegment2d arc({shift,1}, radius, std::atan2(-1.0, -shift),
                             std::numbers::pi_v<double> / 2.0);
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{2,0}, SCPoint2d{0,0}));
    segments.push_back(std::make_shared<SCArcSegment2d>(arc));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const SCPathGeometryTolerance loose{1e-3, 1e-8, 1e-12};
    const auto result = OffsetWithMapping(path, 0.0, {4.0}, loose);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
}

TEST(PathGeometryTest, AdjacentLinesRejectOverlapShorterThanTolerance)
{
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{0,0}, SCPoint2d{1,0}));
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{1 - 5e-11,0},
                                                         SCPoint2d{1e14,0}));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
}

TEST(PathGeometryTest, CloseDistinctOffsetJoinsRemainAmbiguous)
{
    const double pi = std::numbers::pi_v<double>;
    const double radius = std::sqrt(2.0);
    const double distance = (radius - 1.0) / 2.0 - 1e-12;
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{-2,0}, SCPoint2d{0,0}));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{1,-1}, radius, 3.0 * pi / 4.0, pi / 2.0));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const SCPathGeometryTolerance loose{1e-4, 1e-8, 1e-12};
    const auto result = OffsetWithMapping(path, distance, {6.0}, loose);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::AmbiguousJoin);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
}

TEST(PathGeometryTest, LineArcOffsetKeepsUniqueJoinAndMapping)
{
    const double pi = std::numbers::pi_v<double>;
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{-2,0}, SCPoint2d{0,0}));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{1,-1}, std::sqrt(2.0), 3.0 * pi / 4.0, pi / 2.0));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, -0.1, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.mappings.size(), 2u);
    EXPECT_LT(result.mappings[0].sourceEndParameter, 1.0);
    EXPECT_GT(result.mappings[1].sourceStartParameter, 0.0);
    EXPECT_EQ(result.mappings[0].sourceSegmentIndex, 0u);
    EXPECT_EQ(result.mappings[1].outputSegmentIndex, 1u);
    const auto line = result.polyline.SegmentAt(0);
    const auto arc = result.polyline.SegmentAt(1);
    ASSERT_NE(line, nullptr);
    ASSERT_NE(arc, nullptr);
    EXPECT_NEAR(line->EndPoint().x, arc->StartPoint().x, kTolerance.distanceTolerance);
    EXPECT_NEAR(line->EndPoint().y, arc->StartPoint().y, kTolerance.distanceTolerance);
}

TEST(PathGeometryTest, TangentArcArcOffsetKeepsUniqueJoin)
{
    const double pi = std::numbers::pi_v<double>;
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,1}, 1.0, pi, pi / 2.0));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,-1}, 1.0, pi / 2.0, -pi / 2.0));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 0.2, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.mappings.size(), 2u);
    EXPECT_NEAR(result.mappings[0].sourceEndParameter, 1.0, 1e-12);
    EXPECT_NEAR(result.mappings[1].sourceStartParameter, 0.0, 1e-12);
    const auto first = result.polyline.SegmentAt(0);
    const auto second = result.polyline.SegmentAt(1);
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    EXPECT_NEAR(first->EndPoint().y, 0.2, kTolerance.distanceTolerance);
    EXPECT_NEAR(second->StartPoint().y, 0.2, kTolerance.distanceTolerance);
}

TEST(PathGeometryTest, OppositeArcJoinChoosesTheValidUnwrappedAngle)
{
    const double pi = std::numbers::pi_v<double>;
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{1,0}, 1.0, pi / 2.0, -pi / 2.0));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, 3.0 * pi / 2.0));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 1.0, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.mappings.size(), 2u);
    EXPECT_NEAR(result.mappings[0].sourceEndParameter, 3.0, 1e-12);
    EXPECT_NEAR(result.mappings[1].sourceStartParameter, 2.0 / 3.0, 1e-12);
    const auto first = result.polyline.SegmentAt(0);
    const auto second = result.polyline.SegmentAt(1);
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    EXPECT_NEAR(first->EndPoint().x, -1.0, kTolerance.distanceTolerance);
    EXPECT_NEAR(second->StartPoint().x, -1.0, kTolerance.distanceTolerance);
}

TEST(PathGeometryTest, NearlyCoincidentCirclesStillRequireAnIntersection)
{
    const double pi = std::numbers::pi_v<double>;
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, pi / 2.0));
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0 + 5e-11, pi / 2.0, pi / 2.0));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 0.1, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::JoinNotFound);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 1u);
}

TEST(PathGeometryTest, SignedArcOffsetPreservesDistanceDirection)
{
    std::vector<std::shared_ptr<ISCSegment2d>> arcs;
    arcs.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, std::numbers::pi_v<double> / 2.0));
    const SCPolyline2d path(std::move(arcs), SCPolylineClosure::Open);
    const auto left = OffsetWithMapping(path, 0.5, {4.0}, kTolerance);
    ASSERT_TRUE(left.success);
    const auto right = OffsetWithMapping(path, -0.5, {4.0}, kTolerance);
    ASSERT_TRUE(right.success);
    const auto leftArc = left.polyline.SegmentAt(0);
    const auto rightArc = right.polyline.SegmentAt(0);
    ASSERT_NE(leftArc, nullptr);
    ASSERT_NE(rightArc, nullptr);
    EXPECT_NEAR(static_cast<const SCArcSegment2d&>(*leftArc).radius, 1.5, 1e-12);
    EXPECT_NEAR(static_cast<const SCArcSegment2d&>(*rightArc).radius, 2.5, 1e-12);

    std::vector<std::shared_ptr<ISCSegment2d>> clockwiseSegments;
    clockwiseSegments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 2.0, 0.0, -std::numbers::pi_v<double> / 2.0));
    const SCPolyline2d clockwise(std::move(clockwiseSegments), SCPolylineClosure::Open);
    const auto clockwiseLeft = OffsetWithMapping(clockwise, 0.5, {4.0}, kTolerance);
    ASSERT_TRUE(clockwiseLeft.success);
    const auto clockwiseArc = clockwiseLeft.polyline.SegmentAt(0);
    ASSERT_NE(clockwiseArc, nullptr);
    EXPECT_NEAR(static_cast<const SCArcSegment2d&>(*clockwiseArc).radius, 2.5, 1e-12);
}

TEST(PathGeometryTest, ShortSegmentsStillReceiveTopologyCheck)
{
    const SCPolyline2d crossing({SCPoint2d{0,0}, SCPoint2d{1e-5,0},
                                 SCPoint2d{0,1e-5}, SCPoint2d{1e-5,-1e-5}},
                                SCPolylineClosure::Open);
    ASSERT_TRUE(crossing.IsValid());
    const auto result = OffsetWithMapping(crossing, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 2u);
}

TEST(PathGeometryTest, NearArcEndpointUsesDistanceToleranceInTopology)
{
    const double sweep = 1e-4;
    const double nearEndAngle = sweep + 5e-9;
    const double crossingX = std::cos(nearEndAngle);
    const SCArcSegment2d arc({0,0}, 1.0, 0.0, sweep);
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(arc));
    segments.push_back(std::make_shared<SCLineSegment2d>(arc.EndPoint(), SCPoint2d{crossingX,1}));
    segments.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{crossingX,1},
                                                       SCPoint2d{crossingX,-1}));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::SelfIntersection);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_EQ(result.relatedIndex, 2u);
}

TEST(PathGeometryTest, ExistingTwoDimensionalValidityIsAnAdditionalGate)
{
    std::vector<std::shared_ptr<ISCSegment2d>> lines;
    lines.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{0,0}, SCPoint2d{1,0}));
    lines.push_back(std::make_shared<SCLineSegment2d>(SCPoint2d{1 + 5e-9,0}, SCPoint2d{2,0}));
    const SCPolyline2d gapped(std::move(lines), SCPolylineClosure::Open);
    EXPECT_FALSE(gapped.IsValid());
    const auto rejectedGap = OffsetWithMapping(gapped, 0.0, {4.0}, kTolerance);
    EXPECT_EQ(rejectedGap.failure, SCPathGeometryFailure::Unrepresentable2d);
    EXPECT_EQ(rejectedGap.inputIndex, 0u);
    EXPECT_EQ(rejectedGap.relatedIndex, 1u);

    const SCPathGeometryTolerance tight{1e-12, 1e-8, 1e-12};
    const SCPolyline2d shortLine({SCPoint2d{0,0}, SCPoint2d{5e-10,0}},
                                 SCPolylineClosure::Open);
    EXPECT_EQ(OffsetWithMapping(shortLine, 0.0, {4.0}, tight).failure,
              SCPathGeometryFailure::Unrepresentable2d);
}

TEST(PathGeometryTest, OffsetLengthsUseRobustNormForLargeFiniteLine)
{
    const SCPolyline2d line({SCPoint2d{0,0}, SCPoint2d{1e200,0}},
                            SCPolylineClosure::Open);
    const auto result = OffsetWithMapping(line, 0.0, {4.0}, kTolerance);
    ASSERT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(result.inputTotalLength2d, 1e200);
    EXPECT_DOUBLE_EQ(result.outputTotalLength2d, 1e200);
    ASSERT_EQ(result.mappings.size(), 1u);
    EXPECT_DOUBLE_EQ(result.mappings[0].sourceLength2d, 1e200);
}

TEST(PathGeometryTest, OffsetRejectsFiniteRadiusWithOverflowingSupportLength)
{
    std::vector<std::shared_ptr<ISCSegment2d>> segments;
    segments.push_back(std::make_shared<SCArcSegment2d>(
        SCPoint2d{0,0}, 1e308, 0.0, 1.5));
    const SCPolyline2d path(std::move(segments), SCPolylineClosure::Open);
    ASSERT_TRUE(path.IsValid());
    const auto result = OffsetWithMapping(path, -5e307, {4.0}, kTolerance);
    EXPECT_EQ(result.failure, SCPathGeometryFailure::NonFiniteResult);
    EXPECT_EQ(result.inputIndex, 0u);
    EXPECT_TRUE(result.mappings.empty());
    EXPECT_TRUE(result.outputCumulativeLengths.empty());
}

TEST(PathGeometryTest, CornerReturnsInnerAngleAndSignedTurn)
{
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d left({1,0}, {1,1});
    const SCLineSegment2d reverse({1,0}, {0,0});
    const auto corner = EvaluateCornerGeometry(incoming, left, kTolerance);
    ASSERT_TRUE(corner.success);
    EXPECT_EQ(corner.turn, SCCornerTurn2d::Left);
    EXPECT_NEAR(corner.interiorAngleRad, std::numbers::pi_v<double> / 2.0, 1e-12);
    const auto reversal = EvaluateCornerGeometry(incoming, reverse, kTolerance);
    ASSERT_TRUE(reversal.success);
    EXPECT_EQ(reversal.turn, SCCornerTurn2d::Reversal);
    EXPECT_DOUBLE_EQ(reversal.signedTurnRad, std::numbers::pi_v<double>);
    const SCLineSegment2d right({1,0}, {1,-1});
    const auto rightCorner = EvaluateCornerGeometry(incoming, right, kTolerance);
    ASSERT_TRUE(rightCorner.success);
    EXPECT_EQ(rightCorner.turn, SCCornerTurn2d::Right);
    EXPECT_NEAR(rightCorner.signedTurnRad, -std::numbers::pi_v<double> / 2.0, 1e-12);
    const SCLineSegment2d straight({1,0}, {2,0});
    const auto straightCorner = EvaluateCornerGeometry(incoming, straight, kTolerance);
    ASSERT_TRUE(straightCorner.success);
    EXPECT_EQ(straightCorner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(straightCorner.interiorAngleRad, std::numbers::pi_v<double>);
}

TEST(PathGeometryTest, ArcCornerUsesEndpointTangents)
{
    const SCLineSegment2d incoming({-1,0}, {0,0});
    const SCArcSegment2d outgoing({0,1}, 1.0, -std::numbers::pi_v<double> / 2.0,
                                  std::numbers::pi_v<double> / 2.0);
    const auto corner = EvaluateCornerGeometry(incoming, outgoing, kTolerance);
    ASSERT_TRUE(corner.success);
    EXPECT_EQ(corner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(corner.interiorAngleRad, std::numbers::pi_v<double>);
}

TEST(PathGeometryTest, CornerNormalizesToleranceBandsAndArcArcTangents)
{
    const double pi = std::numbers::pi_v<double>;
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d nearStraight({1,0}, {2,5e-9});
    const auto straight = EvaluateCornerGeometry(incoming, nearStraight, kTolerance);
    ASSERT_TRUE(straight.success);
    EXPECT_EQ(straight.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(straight.signedTurnRad, 0.0);

    const SCLineSegment2d beyondStraight({1,0}, {2,2e-8});
    const auto left = EvaluateCornerGeometry(incoming, beyondStraight, kTolerance);
    ASSERT_TRUE(left.success);
    EXPECT_EQ(left.turn, SCCornerTurn2d::Left);
    EXPECT_GT(left.signedTurnRad, kTolerance.angleToleranceRad);

    const SCLineSegment2d nearNegativeReversal({1,0}, {0,-5e-9});
    const auto reversal = EvaluateCornerGeometry(incoming, nearNegativeReversal, kTolerance);
    ASSERT_TRUE(reversal.success);
    EXPECT_EQ(reversal.turn, SCCornerTurn2d::Reversal);
    EXPECT_DOUBLE_EQ(reversal.signedTurnRad, pi);
    EXPECT_DOUBLE_EQ(reversal.interiorAngleRad, 0.0);

    const SCArcSegment2d arcIn({0,1}, 1.0, pi, pi / 2.0);
    const SCArcSegment2d arcOut({0,-1}, 1.0, pi / 2.0, -pi / 2.0);
    const auto arcCorner = EvaluateCornerGeometry(arcIn, arcOut, kTolerance);
    ASSERT_TRUE(arcCorner.success);
    EXPECT_EQ(arcCorner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(arcCorner.interiorAngleRad, pi);
}

TEST(PathGeometryTest, CornerRejectsGapZeroDirectionAndNonFiniteInput)
{
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d gapped({2,0}, {3,0});
    const auto gap = EvaluateCornerGeometry(incoming, gapped, kTolerance);
    EXPECT_EQ(gap.failure, SCPathGeometryFailure::DiscontinuousPath);
    EXPECT_EQ(gap.inputIndex, 0u);
    EXPECT_EQ(gap.relatedIndex, 1u);

    const SCLineSegment2d zero({1,0}, {1,0});
    const auto degenerate = EvaluateCornerGeometry(incoming, zero, kTolerance);
    EXPECT_EQ(degenerate.failure, SCPathGeometryFailure::DegenerateDirection);
    EXPECT_EQ(degenerate.inputIndex, 1u);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const SCLineSegment2d invalid({1,0}, {nan,0});
    const auto rejected = EvaluateCornerGeometry(incoming, invalid, kTolerance);
    EXPECT_EQ(rejected.failure, SCPathGeometryFailure::NonFiniteInput);
    EXPECT_EQ(rejected.inputIndex, 1u);
}
