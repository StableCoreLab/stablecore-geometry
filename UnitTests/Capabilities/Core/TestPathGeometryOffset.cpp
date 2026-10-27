#include <cmath>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Core/Offset.h"
#include "Core/PathGeometry.h"
#include "Geometry2d/SCLineSegment2d.h"

using namespace Geometry;

namespace
{
    const SCPathGeometryTolerance kTolerance{1e-8, 1e-8, 1e-12};
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
