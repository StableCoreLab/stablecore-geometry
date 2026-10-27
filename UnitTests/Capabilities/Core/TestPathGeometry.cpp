#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

#include <gtest/gtest.h>

#include "Core/ArcOps3d.h"
#include "Core/PathGeometry.h"

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
