#include <gtest/gtest.h>
#include <ostream>
#include <string>

#include "Serialize/GeometryText.h"

namespace
{
    enum class InvalidInputKind
    {
        Point,
        Vector,
        Box,
        Projection,
        Arc,
        Polyline,
        Polygon,
    };

    struct InvalidInputCase
    {
        const char* name;
        InvalidInputKind kind;
        const char* text;
    };

    void PrintTo(const InvalidInputCase& value, std::ostream* stream)
    {
        *stream << value.name << " {" << value.text << "}";
    }

    class SerializeInvalidInputTest : public ::testing::TestWithParam<InvalidInputCase>
    {
    };

    TEST_P(SerializeInvalidInputTest, RejectsMalformedText)
    {
        using Geometry::SCArcSegment2d;
        using Geometry::SCBox2d;
        using Geometry::SCPoint2d;
        using Geometry::SCPolygon2d;
        using Geometry::SCPolyline2d;
        using Geometry::SCSegmentProjection2d;
        using Geometry::SCVector2d;
        using Geometry::Serialize::FromText;

        bool parsed = false;
        switch (GetParam().kind)
        {
            case InvalidInputKind::Point:
            {
                SCPoint2d value{};
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Vector:
            {
                SCVector2d value{};
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Box:
            {
                SCBox2d value{};
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Projection:
            {
                SCSegmentProjection2d value{};
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Arc:
            {
                SCArcSegment2d value{};
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Polyline:
            {
                SCPolyline2d value;
                parsed = FromText(GetParam().text, value);
                break;
            }
            case InvalidInputKind::Polygon:
            {
                SCPolygon2d value;
                parsed = FromText(GetParam().text, value);
                break;
            }
        }

        EXPECT_FALSE(parsed);
    }

    INSTANTIATE_TEST_SUITE_P(
        MalformedInputs,
        SerializeInvalidInputTest,
        ::testing::Values(InvalidInputCase{"Point", InvalidInputKind::Point, "SCPoint2d 1.0"},
                          InvalidInputCase{"Vector", InvalidInputKind::Vector, "SCVector2d 1 2 3"},
                          InvalidInputCase{"Box", InvalidInputKind::Box, "SCBox2d 0 0 1"},
                          InvalidInputCase{"Projection", InvalidInputKind::Projection, "SCSegmentProjection2d 1 2 3 4"},
                          InvalidInputCase{"Arc", InvalidInputKind::Arc, "SCArcSegment2d 0 0 1 0 0"},
                          InvalidInputCase{"Polyline", InvalidInputKind::Polyline, "SCPolyline2d open 2 0 0 1"},
                          InvalidInputCase{"Polygon", InvalidInputKind::Polygon, "SCPolygon2d SCPolyline2d open 2 0 0 1 0 0"}),
        [](const ::testing::TestParamInfo<InvalidInputCase>& info) { return info.param.name; });
}

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
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 1.0, 0.0, Geometry::kPi * 0.5);
    const SCPolyline2d polyline({SCPoint2d{0.0, 0.0}, SCPoint2d{3.0, 0.0}, SCPoint2d{3.0, 4.0}}, SCPolylineClosure::Open);
    const SCPolygon2d polygon(
        SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                     SCPolylineClosure::Closed),
        {SCPolyline2d({SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}},
                      SCPolylineClosure::Closed)});

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
    ASSERT_EQ(parsedPolyline.PointCount(), polyline.PointCount());
    EXPECT_EQ(parsedPolyline.IsClosed(), polyline.IsClosed());
    for (std::size_t i = 0; i < polyline.PointCount(); ++i)
    {
        EXPECT_TRUE(parsedPolyline.PointAt(i).AlmostEquals(polyline.PointAt(i), 1e-12));
    }

    ASSERT_EQ(parsedPolygon.HoleCount(), polygon.HoleCount());
    ASSERT_EQ(parsedPolygon.OuterRing().PointCount(), polygon.OuterRing().PointCount());
    for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
    {
        EXPECT_TRUE(parsedPolygon.OuterRing().PointAt(i).AlmostEquals(polygon.OuterRing().PointAt(i), 1e-12));
    }
    for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
    {
        ASSERT_EQ(parsedPolygon.HoleAt(holeIndex).PointCount(), polygon.HoleAt(holeIndex).PointCount());
        for (std::size_t pointIndex = 0; pointIndex < polygon.HoleAt(holeIndex).PointCount(); ++pointIndex)
        {
            EXPECT_TRUE(parsedPolygon.HoleAt(holeIndex).PointAt(pointIndex)
                            .AlmostEquals(polygon.HoleAt(holeIndex).PointAt(pointIndex), 1e-12));
        }
    }

}
