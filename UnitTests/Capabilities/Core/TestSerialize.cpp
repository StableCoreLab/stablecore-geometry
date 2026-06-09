#include <gtest/gtest.h>
#include <numbers>
#include <string>

#include "Serialize/GeometryText.h"
#include "Support/GeometryTestSupport.h"

TEST(SerializeTest, CoversCurrentCapabilities)
{
    using Geometry::SCArcSegment2d;
    using Geometry::SCBox2d;
    using Geometry::SCPoint2d;
    using Geometry::SCPolygon2d;
    using Geometry::SCPolyline2d;
    using Geometry::SCPolylineClosure;
    using Geometry::SCSegmentProjection2d;
    using Geometry::SCVector2d;
    using Geometry::Serialize::FromText;
    using Geometry::Serialize::ToText;

    const SCPoint2d point = SCPoint2d::FromXY(1.25, -3.5);
    const SCVector2d vector = SCVector2d::FromXY(-4.0, 8.5);
    const SCBox2d box = SCBox2d::FromMinMax(SCPoint2d{0.0, 1.0}, SCPoint2d{2.0, 3.0});
    const SCSegmentProjection2d projection{SCPoint2d{4.0, 5.0}, 0.25, 12.5, true};
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> * 0.5);
    const SCPolyline2d polyline({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    const SCPolygon2d polygon(Geometry::Test::MakeRectangleRing(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 4.0}),
                            {Geometry::Test::MakeRectangleHoleRing(SCPoint2d{1.0, 1.0}, SCPoint2d{3.0, 3.0})});

    const std::string pointText = ToText(point);
    const std::string vectorText = ToText(vector);
    const std::string boxText = ToText(box);
    const std::string projectionText = ToText(projection);
    const std::string arcText = ToText(arc);
    const std::string polylineText = ToText(polyline);
    const std::string polygonText = ToText(polygon);

    ASSERT_EQ(pointText, "SCPoint2d 1.25 -3.5");
    ASSERT_EQ(vectorText, "SCVector2d -4 8.5");
    ASSERT_EQ(boxText, "SCBox2d 0 1 2 3");
    ASSERT_EQ(projectionText, "SCSegmentProjection2d 4 5 0.25 12.5 1");
    ASSERT_EQ(arcText, "SCArcSegment2d 0 0 1 0 1.5707963267948966");
    ASSERT_EQ(polylineText, "SCPolyline2d open 3 0 0 3 0 3 4");
    ASSERT_EQ(polygonText,
              "SCPolygon2d SCPolyline2d closed 4 0 0 4 0 4 4 0 4 1 SCPolyline2d "
              "closed 4 1 1 1 3 3 3 3 1");

    SCPoint2d parsedPoint{};
    SCVector2d parsedVector{};
    SCBox2d parsedBox{};
    SCSegmentProjection2d parsedProjection{};
    SCArcSegment2d parsedArc{};
    SCPolyline2d parsedPolyline;
    SCPolygon2d parsedPolygon;

    ASSERT_TRUE(FromText(pointText, parsedPoint));
    ASSERT_TRUE(FromText(vectorText, parsedVector));
    ASSERT_TRUE(FromText(boxText, parsedBox));
    ASSERT_TRUE(FromText(projectionText, parsedProjection));
    ASSERT_TRUE(FromText(arcText, parsedArc));
    ASSERT_TRUE(FromText(polylineText, parsedPolyline));
    ASSERT_TRUE(FromText(polygonText, parsedPolygon));

    ASSERT_EQ(parsedPoint, point);
    ASSERT_EQ(parsedVector, vector);
    ASSERT_EQ(parsedBox, box);
    ASSERT_EQ(parsedProjection.point, projection.point);
    ASSERT_EQ(parsedProjection.parameter, projection.parameter);
    ASSERT_EQ(parsedProjection.distanceSquared, projection.distanceSquared);
    ASSERT_EQ(parsedProjection.isOnSegment, projection.isOnSegment);
    ASSERT_TRUE(parsedArc.AlmostEquals(arc));
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(parsedPolyline, polyline, 1e-12);
    GEOMETRY_TEST_ASSERT_POLYGON_NEAR(parsedPolygon, polygon, 1e-12);

    ASSERT_FALSE(FromText("SCPoint2d 1.0", parsedPoint));
    ASSERT_FALSE(FromText("SCVector2d 1 2 3", parsedVector));
    ASSERT_FALSE(FromText("SCBox2d 0 0 1", parsedBox));
    ASSERT_FALSE(FromText("SCSegmentProjection2d 1 2 3 4", parsedProjection));
    ASSERT_FALSE(FromText("SCArcSegment2d 0 0 1 0 0", parsedArc));
    ASSERT_FALSE(FromText("SCPolyline2d open 2 0 0 1", parsedPolyline));
    ASSERT_FALSE(FromText("SCPolygon2d SCPolyline2d open 2 0 0 1 0 0", parsedPolygon));
}

