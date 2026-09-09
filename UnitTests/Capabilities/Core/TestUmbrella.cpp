#include <gtest/gtest.h>

#include "Geometry.h"

using Geometry::BodyBooleanIssue3d;
using Geometry::ClassifyParallelSegmentProjection;
using Geometry::Contains;
using Geometry::DifferenceBodies;
using Geometry::IntersectBodies;
using Geometry::Intersects;
using Geometry::QueryPolygonPositiveAreaIntersection;
using Geometry::SCBox2d;
using Geometry::SCLineSegment2d;
using Geometry::SCParallelSegmentProjectionRelation2d;
using Geometry::SCParallelSegmentProjectionTolerance2d;
using Geometry::SCPlane;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPolygon2d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronFace3d;
using Geometry::PolyhedronLoop3d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SearchPolygons;
using Geometry::SCSearchPolyIssue2d;
using Geometry::UnionBodies;
using Geometry::SCVector3d;

using namespace Geometry;

namespace
{
    [[nodiscard]] PolyhedronBody BuildAxisAlignedBoxBody(
        double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        return PolyhedronBody({
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{minX, minY, minZ}, SCVector3d{0.0, 0.0, -1.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{minX, minY, minZ},
                                 SCPoint3d{minX, maxY, minZ},
                                 SCPoint3d{maxX, maxY, minZ},
                                 SCPoint3d{maxX, minY, minZ},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{minX, minY, maxZ}, SCVector3d{0.0, 0.0, 1.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{minX, minY, maxZ},
                                 SCPoint3d{maxX, minY, maxZ},
                                 SCPoint3d{maxX, maxY, maxZ},
                                 SCPoint3d{minX, maxY, maxZ},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{minX, minY, minZ}, SCVector3d{0.0, -1.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{minX, minY, minZ},
                                 SCPoint3d{maxX, minY, minZ},
                                 SCPoint3d{maxX, minY, maxZ},
                                 SCPoint3d{minX, minY, maxZ},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{maxX, minY, minZ}, SCVector3d{1.0, 0.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{maxX, minY, minZ},
                                 SCPoint3d{maxX, maxY, minZ},
                                 SCPoint3d{maxX, maxY, maxZ},
                                 SCPoint3d{maxX, minY, maxZ},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{minX, maxY, minZ}, SCVector3d{0.0, 1.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{minX, maxY, minZ},
                                 SCPoint3d{minX, maxY, maxZ},
                                 SCPoint3d{maxX, maxY, maxZ},
                                 SCPoint3d{maxX, maxY, minZ},
                             })),
            PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{minX, minY, minZ}, SCVector3d{-1.0, 0.0, 0.0}),
                             PolyhedronLoop3d({
                                 SCPoint3d{minX, minY, minZ},
                                 SCPoint3d{minX, minY, maxZ},
                                 SCPoint3d{minX, maxY, maxZ},
                                 SCPoint3d{minX, maxY, minZ},
                             })),
        });
    }
}  // namespace

TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesSearchPolyContract)
{
    const SCMultiPolyline2d lines{SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}}, SCPolylineClosure::Open),
                                SCPolyline2d({SCPoint2d{0.0, 4.0}, SCPoint2d{0.0, 0.0}}, SCPolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_EQ(result.issue, SCSearchPolyIssue2d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_EQ(result.candidates.front().rank, 0U);
}

TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesBodyBooleanContract)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_TRUE(result.bodies.empty());
}

TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesBodyBooleanIssueContract)
{
    const PolyhedronBody first;
    const PolyhedronBody second;

    const auto intersection = IntersectBodies(first, second);
    const auto difference = DifferenceBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_FALSE(intersection.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

// 验证聚合入口 Geometry.h 暴露 ClassifyParallelSegmentProjection。
TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesParallelSegmentProjectionClassification)
{
    const SCParallelSegmentProjectionTolerance2d tolerance;
    const auto relation = ClassifyParallelSegmentProjection(
        SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 0.0}},
        SCLineSegment2d{SCPoint2d{2.0, 0.0}, SCPoint2d{8.0, 0.0}},
        tolerance);
    EXPECT_EQ(relation, SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection);
}

// 验证聚合入口 Geometry.h 暴露 QueryPolygonPositiveAreaIntersection。
TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesPolygonPositiveAreaIntersectionQuery)
{
    const SCPolygon2d first(SCPolyline2d({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}},
                                          SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d({{5.0, 5.0}, {15.0, 5.0}, {15.0, 15.0}, {5.0, 15.0}},
                                           SCPolylineClosure::Closed));
    const auto result = QueryPolygonPositiveAreaIntersection(first, second);
    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.hasPositiveAreaIntersection);
}

// 验证聚合入口 Geometry.h 暴露 Contains(box, polyline)。
TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesBoxPolylineContains)
{
    const SCBox2d box = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 10.0});
    const SCPolyline2d polyline({{2.0, 2.0}, {8.0, 8.0}}, SCPolylineClosure::Open);
    EXPECT_TRUE(Contains(box, polyline));
}

// 验证聚合入口 Geometry.h 暴露 Intersects(box, polyline)。
TEST(UmbrellaHeaderTest, GeometryUmbrellaExposesBoxPolylineIntersects)
{
    const SCBox2d box = SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 10.0});
    const SCPolyline2d polyline({{-5.0, 5.0}, {15.0, 5.0}}, SCPolylineClosure::Open);
    EXPECT_TRUE(Intersects(box, polyline));
    EXPECT_FALSE(Contains(box, polyline));
}

