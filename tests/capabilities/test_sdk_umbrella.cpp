#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::BodyBooleanIssue3d;
using geometry::sdk::DifferenceBodies;
using geometry::sdk::IntersectBodies;
using geometry::sdk::Point2d;
using geometry::sdk::Point3d;
using geometry::sdk::Plane;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronFace3d;
using geometry::sdk::PolyhedronLoop3d;
using geometry::sdk::SearchPolyIssue2d;
using geometry::sdk::SearchPolygons;
using geometry::sdk::UnionBodies;
using geometry::sdk::Vector3d;

namespace
{
[[nodiscard]] PolyhedronBody BuildAxisAlignedBoxBody(
    double minX,
    double minY,
    double minZ,
    double maxX,
    double maxY,
    double maxZ)
{
    return PolyhedronBody(
        {
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{0.0, 0.0, -1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{minX, maxY, minZ},
                        Point3d{maxX, maxY, minZ},
                        Point3d{maxX, minY, minZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, maxZ}, Vector3d{0.0, 0.0, 1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, maxZ},
                        Point3d{maxX, minY, maxZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{minX, maxY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{0.0, -1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{maxX, minY, minZ},
                        Point3d{maxX, minY, maxZ},
                        Point3d{minX, minY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{maxX, minY, minZ}, Vector3d{1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{maxX, minY, minZ},
                        Point3d{maxX, maxY, minZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{maxX, minY, maxZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, maxY, minZ}, Vector3d{0.0, 1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, maxY, minZ},
                        Point3d{minX, maxY, maxZ},
                        Point3d{maxX, maxY, maxZ},
                        Point3d{maxX, maxY, minZ},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{minX, minY, minZ}, Vector3d{-1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{minX, minY, minZ},
                        Point3d{minX, minY, maxZ},
                        Point3d{minX, maxY, maxZ},
                        Point3d{minX, maxY, minZ},
                    })),
        });
}
} // namespace

TEST(SdkUmbrellaHeaderTest, GeometryUmbrellaExposesSearchPolyContract)
{
    const geometry::sdk::MultiPolyline2d lines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 0.0}}, PolylineClosure::Open)};

    const auto result = SearchPolygons(lines);

    ASSERT_EQ(result.issue, SearchPolyIssue2d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_EQ(result.candidates.front().rank, 0U);
}

TEST(SdkUmbrellaHeaderTest, GeometryUmbrellaExposesBodyBooleanContract)
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

TEST(SdkUmbrellaHeaderTest, GeometryUmbrellaExposesBodyBooleanIssueContract)
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
