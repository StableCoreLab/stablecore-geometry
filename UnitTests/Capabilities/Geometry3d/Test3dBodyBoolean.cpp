#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "Brep/BodyBoolean.h"
#include "Brep/BrepConversion.h"
#include "support/Fixtures3d.h"

using Geometry::BodyBooleanIssue3d;
using Geometry::BodyBooleanOptions3d;
using Geometry::DifferenceBodies;
using Geometry::IntersectBodies;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronFace3d;
using Geometry::PolyhedronLoop3d;
using Geometry::SCPlane;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;
using Geometry::UnionBodies;

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

    [[nodiscard]] PolyhedronBody BuildTranslatedUnitCubeBody(double dx, double dy, double dz)
    {
        return BuildAxisAlignedBoxBody(dx + 0.0, dy + 0.0, dz + 0.0, dx + 1.0, dy + 1.0, dz + 1.0);
    }

    [[nodiscard]] SCPoint3d RotatePointAroundZ(const SCPoint3d& point, const SCPoint3d& origin, double angleRadians)
    {
        const double cosine = std::cos(angleRadians);
        const double sine = std::sin(angleRadians);
        const double dx = point.x - origin.x;
        const double dy = point.y - origin.y;
        return SCPoint3d{origin.x + dx * cosine - dy * sine, origin.y + dx * sine + dy * cosine, point.z};
    }

    [[nodiscard]] SCVector3d RotateVectorAroundZ(const SCVector3d& vector, double angleRadians)
    {
        const double cosine = std::cos(angleRadians);
        const double sine = std::sin(angleRadians);
        return SCVector3d{vector.x * cosine - vector.y * sine, vector.x * sine + vector.y * cosine, vector.z};
    }

    [[nodiscard]] PolyhedronLoop3d RotateLoopAroundZ(const PolyhedronLoop3d& loop,
                                                     const SCPoint3d& origin,
                                                     double angleRadians)
    {
        std::vector<SCPoint3d> vertices;
        vertices.reserve(loop.VertexCount());
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            vertices.push_back(RotatePointAroundZ(loop.VertexAt(i), origin, angleRadians));
        }
        return PolyhedronLoop3d(std::move(vertices));
    }

    [[nodiscard]] PolyhedronFace3d RotateFaceAroundZ(const PolyhedronFace3d& face,
                                                     const SCPoint3d& origin,
                                                     double angleRadians)
    {
        const SCPlane rotatedPlane =
            SCPlane::FromPointAndNormal(RotatePointAroundZ(face.SupportPlane().origin, origin, angleRadians),
                                        RotateVectorAroundZ(face.SupportPlane().normal, angleRadians));

        PolyhedronLoop3d outer = RotateLoopAroundZ(face.OuterLoop(), origin, angleRadians);
        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            holes.push_back(RotateLoopAroundZ(face.HoleAt(i), origin, angleRadians));
        }

        return PolyhedronFace3d(rotatedPlane, std::move(outer), std::move(holes));
    }

    [[nodiscard]] PolyhedronBody BuildRotatedOverlapBoxBody()
    {
        const PolyhedronBody box = BuildAxisAlignedBoxBody(0.25, 0.0, 0.0, 1.25, 1.0, 1.0);
        std::vector<PolyhedronFace3d> faces;
        faces.reserve(box.FaceCount());

        const SCPoint3d rotationOrigin{0.75, 0.5, 0.0};
        const double angleRadians = std::acos(-1.0) * 0.25;
        for (const PolyhedronFace3d& face : box.Faces())
        {
            faces.push_back(RotateFaceAroundZ(face, rotationOrigin, angleRadians));
        }

        return PolyhedronBody(std::move(faces));
    }
}  // namespace

TEST(BodyBooleanCapabilityTest, InvalidInputContractIsStableForEmptyBodies)
{
    const PolyhedronBody first;
    const PolyhedronBody second;
    const BodyBooleanOptions3d options{};

    const auto intersection = IntersectBodies(first, second, options);
    const auto united = UnionBodies(first, second, options);
    const auto difference = DifferenceBodies(first, second, options);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(united.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_FALSE(intersection.IsSuccess());
    EXPECT_FALSE(united.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

TEST(BodyBooleanCapabilityTest, IdenticalPolyhedronIntersectionReturnsClosedBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBooleanCapabilityTest, IdenticalPolyhedronUnionReturnsSingleClosedBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, IdenticalPolyhedronDifferenceReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, DisjointPolyhedronUnionReturnsTwoBodies)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
}

TEST(BodyBooleanCapabilityTest, DisjointPolyhedronUnionNormalizesBodyOrder)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = UnionBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 3.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 4.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, DisjointPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.VertexCount(), 8U);
    EXPECT_EQ(result.body.EdgeCount(), 12U);
}

TEST(BodyBooleanCapabilityTest, OverlappingPolyhedronIntersectionReturnsSingleOverlapBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, OverlappingPolyhedronUnionReturnsSingleAxisAlignedBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingPolyhedronUnionReturnsSingleAxisAlignedBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, ContainedPolyhedronIntersectionReturnsInnerBody)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, ContainedPolyhedronUnionReturnsOuterBody)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.5, 1e-12);
}

TEST(BodyBooleanCapabilityTest, DisjointPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(3.0, 0.0, 0.0, 4.0, 1.0, 1.0);

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, EdgeTouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, VertexTouchingPolyhedronIntersectionReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

    const auto intersection = IntersectBodies(first, second);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(intersection.IsSuccess());
    EXPECT_TRUE(intersection.producedEmptyResult);
    EXPECT_EQ(intersection.body.FaceCount(), 0U);
    EXPECT_TRUE(intersection.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, EdgeTouchingPolyhedronUnionReturnsDeterministicOrderedMultiBodyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0);

    const auto result = UnionBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_FALSE(result.producedEmptyResult);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 2.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, VertexTouchingPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingPolyhedronDifferenceReturnsOriginalBody)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, OverlappingPolyhedronDifferenceReturnsRemainingSingleBox)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.0, 1.0, 1.0);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 0.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, ContainedPolyhedronDifferenceReturnsDeterministicEmptyResult)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);

    const auto result = DifferenceBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, NonBoxOverlapUnionAndDifferenceRemainUnsupported)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.5, 0.0, 1.5, 1.5, 1.0);

    const auto united = UnionBodies(first, second);
    const auto difference = DifferenceBodies(first, second);

    EXPECT_EQ(united.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(united.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

TEST(BodyBooleanCapabilityTest, FaceTouchingLShapeUnionReturnsExplicitUnsupportedOperation)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0);
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);

    const auto result = UnionBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, BrepFaceTouchingLShapeUnionReturnsExplicitUnsupportedOperation)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, RotatedBoxIntersectionReturnsExplicitUnsupportedOperation)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedOverlapBoxBody();

    const auto result = IntersectBodies(first, second);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, BrepRotatedBoxIntersectionReturnsExplicitUnsupportedOperation)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildRotatedOverlapBoxBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::UnsupportedOperation);
    EXPECT_FALSE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, IdenticalBrepIntersectionReturnsClosedBody)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    const auto result = IntersectBodies(converted.body, converted.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
}

TEST(BodyBooleanCapabilityTest, IdenticalBrepDifferenceReturnsDeterministicEmptyResult)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    const auto result = DifferenceBodies(converted.body, converted.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, ContainedBrepIntersectionReturnsInnerBody)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    const auto second = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, ContainedBrepUnionReturnsOuterBody)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    const auto second = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, -0.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.5, 1e-12);
}

TEST(BodyBooleanCapabilityTest, DisjointBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(3.0, 0.0, 0.0, 4.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, DisjointBrepUnionNormalizesBodyOrder)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildTranslatedUnitCubeBody(3.0, 0.0, 0.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.bodies[0].FaceCount(), 6U);
    EXPECT_EQ(result.bodies[1].FaceCount(), 6U);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 3.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 4.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, OverlappingBrepUnionReturnsSingleAxisAlignedBox)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.5, 0.0, 0.0, 1.5, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.5, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingBrepUnionReturnsSingleAxisAlignedBox)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingBrepDifferenceReturnsOriginalBody)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, TouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, EdgeTouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, VertexTouchingBrepIntersectionReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}

TEST(BodyBooleanCapabilityTest, EdgeTouchingBrepUnionReturnsDeterministicOrderedMultiBodyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 0.0, 2.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_EQ(result.bodies.size(), 2U);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_FALSE(result.producedEmptyResult);
    const auto firstBounds = result.bodies[0].Bounds();
    const auto secondBounds = result.bodies[1].Bounds();
    EXPECT_NEAR(firstBounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(firstBounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MinPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(secondBounds.MaxPoint().x, 2.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, VertexTouchingBrepDifferenceReturnsOriginalBody)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(1.0, 1.0, 1.0, 2.0, 2.0, 2.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.bodies.empty());
    EXPECT_FALSE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 6U);
    EXPECT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    const auto bounds = result.body.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 1.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().z, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().z, 1.0, 1e-12);
}

TEST(BodyBooleanCapabilityTest, ContainedBrepDifferenceReturnsDeterministicEmptyResult)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = DifferenceBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.producedEmptyResult);
    EXPECT_EQ(result.body.FaceCount(), 0U);
    EXPECT_TRUE(result.bodies.empty());
}
