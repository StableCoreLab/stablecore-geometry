#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "Core/Editing.h"
#include "Core/Sampling.h"
#include "Core/Transform.h"

using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::InsertPoint;
using Geometry::SCLineSegment2d;
using Geometry::Mirror;
using Geometry::Normalize;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Rotate;
using Geometry::SampleByMaxAngle;
using Geometry::Stretch;
using Geometry::Translate;
using Geometry::SCVector2d;

TEST(TransformSamplingTest, CoversCurrentCapabilities)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0});
    const auto moved = Translate(line, SCVector2d{1.0, 2.0});
    EXPECT_TRUE(moved.startPoint.AlmostEquals(SCPoint2d{1.0, 2.0}, 1e-12));

    const auto rotated = Rotate(SCPoint2d{1.0, 0.0}, SCPoint2d{0.0, 0.0}, Geometry::kPi * 0.5);
    EXPECT_TRUE(rotated.AlmostEquals(SCPoint2d{0.0, 1.0}, 1e-12));

    const auto mirrored = Mirror(SCPoint2d{1.0, 2.0}, SCPoint2d{0.0, 0.0}, SCVector2d{1.0, 0.0});
    EXPECT_TRUE(mirrored.AlmostEquals(SCPoint2d{1.0, -2.0}, 1e-12));

    const auto stretched =
        Stretch(SCPoint2d{1.0, 1.0}, SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}), SCVector2d{3.0, 0.0});
    EXPECT_TRUE(stretched.AlmostEquals(SCPoint2d{4.0, 1.0}, 1e-12));

    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 1.0, 0.0, Geometry::kPi * 0.5);
    const auto arcSamples = SampleByMaxAngle(arc, Geometry::kPi * 0.25);
    ASSERT_EQ(arcSamples.size(), 3);

    const SCPolyline2d path({SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 0.0}, SCPoint2d{2.0, 2.0}},
                          SCPolylineClosure::Open);
    const auto normalized = Normalize(path);
    ASSERT_EQ(normalized.PointCount(), 3);

    const auto inserted =
        InsertPoint(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open), SCPoint2d{2.0, 0.0});
    ASSERT_EQ(inserted.PointCount(), 3);

    const SCPolyline2d mixedPath(
        {std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}),
         std::make_shared<SCArcSegment2d>(
             SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, 0.0, Geometry::SCArcDirection::CounterClockwise)},
        SCPolylineClosure::Open);
    const SCPolyline2d movedMixedPath = Translate(mixedPath, SCVector2d{2.0, 3.0});
    ASSERT_TRUE(movedMixedPath.IsValid());
    ASSERT_EQ(movedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2::Line);
    ASSERT_EQ(movedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2::Arc);
    const SCPolyline2d rotatedMixedPath = Rotate(mixedPath, SCPoint2d{0.0, 0.0}, Geometry::kPi * 0.5);
    ASSERT_TRUE(rotatedMixedPath.IsValid());
    ASSERT_EQ(rotatedMixedPath.SegmentAt(0)->Kind(), Geometry::SCSegmentKind2::Line);
    ASSERT_EQ(rotatedMixedPath.SegmentAt(1)->Kind(), Geometry::SCSegmentKind2::Arc);

    const SCPolygon2d polygon(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                                       SCPolylineClosure::Closed));
    const auto movedPolygon = Translate(polygon, SCVector2d{1.0, 0.0});
    EXPECT_TRUE(movedPolygon.OuterRing().PointAt(0).AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));
}
