#include <gtest/gtest.h>

#include "sdk/GeometryOffset.h"

using geometry::sdk::Point2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;

TEST(OffsetGapTest, ReverseEdgeAndSelfIntersectionRecoveryRemainOpen)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{6.0, 0.0}, Point2d{6.0, 1.0}, Point2d{2.5, 1.0}, Point2d{2.5, 4.0}, Point2d{6.0, 4.0},
             Point2d{6.0, 5.0}, Point2d{0.0, 5.0}},
            PolylineClosure::Closed));
    (void)source;
    GTEST_SKIP() << "Known gap: reverse-edge, self-intersection, hole/outer semantic flip, and multi-failure offset recovery are not closed yet.";
}

TEST(OffsetGapTest, SinglePolygonHoleSemanticRecoveryRemainsOpen)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
            PolylineClosure::Closed),
        {Polyline2d(
            {Point2d{1.0, 1.0}, Point2d{1.0, 3.0}, Point2d{3.0, 3.0}, Point2d{3.0, 1.0}},
            PolylineClosure::Closed)});
    (void)source;
    GTEST_SKIP() << "Known gap: single-polygon offset does not yet reliably preserve hole semantics after rebuild.";
}

TEST(OffsetGapTest, NarrowBridgeSplitRecoveryRemainsOpen)
{
    const Polygon2d source(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 0.8}, Point2d{4.0, 0.8}, Point2d{4.0, 0.0}, Point2d{6.0, 0.0},
             Point2d{6.0, 2.0}, Point2d{4.0, 2.0}, Point2d{4.0, 1.2}, Point2d{2.0, 1.2}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}},
            PolylineClosure::Closed));
    (void)source;
    GTEST_SKIP() << "Known gap: inward offset of a narrow bridge does not yet reliably split into stable multipolygon output.";
}
