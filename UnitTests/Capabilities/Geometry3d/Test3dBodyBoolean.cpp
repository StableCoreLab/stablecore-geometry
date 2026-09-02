#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "Brep/BodyBoolean.h"
#include "Brep/BrepConversion.h"
#include "Core/Measure.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
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
    struct Point2d
    {
        double x{};
        double y{};
    };

    void ExpectUnsupportedOperation(const Geometry::BodyBooleanResult3d& result)
    {
        EXPECT_EQ(result.issue, BodyBooleanIssue3d::UnsupportedOperation);
        EXPECT_FALSE(result.IsSuccess());
        EXPECT_EQ(result.body.FaceCount(), 0U);
        EXPECT_TRUE(result.bodies.empty());
        EXPECT_FALSE(result.producedEmptyResult);
    }

    void ExpectInvalidInput(const Geometry::BodyBooleanResult3d& result)
    {
        EXPECT_EQ(result.issue, BodyBooleanIssue3d::InvalidInput);
        EXPECT_FALSE(result.IsSuccess());
        EXPECT_EQ(result.body.FaceCount(), 0U);
        EXPECT_TRUE(result.bodies.empty());
        EXPECT_FALSE(result.producedEmptyResult);
    }

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
        const double angleRadians = Geometry::kPi * 0.25;
        for (const PolyhedronFace3d& face : box.Faces())
        {
            faces.push_back(RotateFaceAroundZ(face, rotationOrigin, angleRadians));
        }

        return PolyhedronBody(std::move(faces));
    }

    [[nodiscard]] PolyhedronBody BuildNonCoplanarRotatedOverlapBoxBody()
    {
        const PolyhedronBody box = BuildAxisAlignedBoxBody(0.2, -0.1, 0.2, 1.2, 0.9, 1.2);
        std::vector<PolyhedronFace3d> faces;
        faces.reserve(box.FaceCount());

        const SCPoint3d rotationOrigin{0.7, 0.4, 0.7};
        const double angleRadians = Geometry::kPi / 6.0;
        for (const PolyhedronFace3d& face : box.Faces())
        {
            faces.push_back(RotateFaceAroundZ(face, rotationOrigin, angleRadians));
        }

        return PolyhedronBody(std::move(faces));
    }

    [[nodiscard]] std::vector<Point2d> ClipPolygonAgainstAxisAlignedHalfSpace(
        const std::vector<Point2d>& polygon, const int axis, const double boundary, const bool keepGreater)
    {
        std::vector<Point2d> result;
        if (polygon.empty())
        {
            return result;
        }

        const auto coordinate = [axis](const Point2d& point) { return axis == 0 ? point.x : point.y; };
        const auto inside = [&](const Point2d& point) {
            return keepGreater ? coordinate(point) >= boundary : coordinate(point) <= boundary;
        };
        for (std::size_t index = 0; index < polygon.size(); ++index)
        {
            const Point2d current = polygon[index];
            const Point2d previous = polygon[(index + polygon.size() - 1) % polygon.size()];
            const bool currentInside = inside(current);
            const bool previousInside = inside(previous);
            if (currentInside != previousInside)
            {
                const double previousCoordinate = coordinate(previous);
                const double currentCoordinate = coordinate(current);
                const double ratio = (boundary - previousCoordinate) / (currentCoordinate - previousCoordinate);
                result.push_back(Point2d{previous.x + ratio * (current.x - previous.x),
                                         previous.y + ratio * (current.y - previous.y)});
            }
            if (currentInside)
            {
                result.push_back(current);
            }
        }
        return result;
    }

    [[nodiscard]] double PolygonArea(const std::vector<Point2d>& polygon)
    {
        double doubledArea = 0.0;
        for (std::size_t index = 0; index < polygon.size(); ++index)
        {
            const Point2d& current = polygon[index];
            const Point2d& next = polygon[(index + 1) % polygon.size()];
            doubledArea += current.x * next.y - current.y * next.x;
        }
        return 0.5 * std::abs(doubledArea);
    }

    [[nodiscard]] double RotatedPrismaticBoxIntersectionVolumeOracle(
        const double minX, const double minY, const double minZ, const double maxX, const double maxY, const double maxZ,
        const double angleRadians)
    {
        const double centerX = 0.5 * (minX + maxX);
        const double centerY = 0.5 * (minY + maxY);
        const double cosine = std::cos(angleRadians);
        const double sine = std::sin(angleRadians);
        const auto rotate = [&](const Point2d& point) {
            const double dx = point.x - centerX;
            const double dy = point.y - centerY;
            return Point2d{centerX + dx * cosine - dy * sine, centerY + dx * sine + dy * cosine};
        };

        std::vector<Point2d> clipped{
            rotate(Point2d{minX, minY}), rotate(Point2d{maxX, minY}),
            rotate(Point2d{maxX, maxY}), rotate(Point2d{minX, maxY})};
        clipped = ClipPolygonAgainstAxisAlignedHalfSpace(clipped, 0, 0.0, true);
        clipped = ClipPolygonAgainstAxisAlignedHalfSpace(clipped, 0, 1.0, false);
        clipped = ClipPolygonAgainstAxisAlignedHalfSpace(clipped, 1, 0.0, true);
        clipped = ClipPolygonAgainstAxisAlignedHalfSpace(clipped, 1, 1.0, false);
        const double overlapHeight = std::max(0.0, std::min(1.0, maxZ) - std::max(0.0, minZ));
        return PolygonArea(clipped) * overlapHeight;
    }

    [[nodiscard]] PolyhedronBody BuildRotatedPrismaticBox(
        const double minX, const double minY, const double minZ, const double maxX, const double maxY, const double maxZ,
        const double angleRadians)
    {
        const PolyhedronBody box = BuildAxisAlignedBoxBody(minX, minY, minZ, maxX, maxY, maxZ);
        std::vector<PolyhedronFace3d> faces;
        faces.reserve(box.FaceCount());
        const SCPoint3d rotationOrigin{0.5 * (minX + maxX), 0.5 * (minY + maxY), 0.5 * (minZ + maxZ)};
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

TEST(BodyBooleanCapabilityTest, OpenBrepInputReturnsInvalidInputInsteadOfForcingClosedShell)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    const Geometry::SCBrepBody openBody(converted.body.Vertices(), converted.body.Edges(),
                                        {Geometry::SCBrepShell(shell.Faces(), false)});

    EXPECT_TRUE(openBody.IsValid());
    EXPECT_FALSE(openBody.ShellAt(0).IsClosed());
    ExpectInvalidInput(IntersectBodies(openBody, converted.body));
    ExpectInvalidInput(UnionBodies(openBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(openBody, converted.body));
}

TEST(BodyBooleanCapabilityTest, NonManifoldBrepInputReturnsInvalidInputEvenWhenClosedFlagIsSet)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces = shell.Faces();
    faces.push_back(faces.front());
    const Geometry::SCBrepBody nonManifold(converted.body.Vertices(), converted.body.Edges(),
                                           {Geometry::SCBrepShell(std::move(faces), true)});

    EXPECT_TRUE(nonManifold.IsValid());
    EXPECT_TRUE(nonManifold.ShellAt(0).IsClosed());
    ExpectInvalidInput(IntersectBodies(nonManifold, converted.body));
    ExpectInvalidInput(UnionBodies(nonManifold, converted.body));
    ExpectInvalidInput(DifferenceBodies(nonManifold, converted.body));
}

TEST(BodyBooleanCapabilityTest, CurvedBrepEdgeReturnsUnsupportedInsteadOfUsingItsChord)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    const SCPoint3d start = converted.body.VertexAt(original.StartVertexIndex()).Point();
    const SCPoint3d end = converted.body.VertexAt(original.EndVertexIndex()).Point();
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCNurbsCurve3d>(
            1, std::vector<SCPoint3d>{start, end}, std::vector<double>{0.0, 0.0, 1.0, 1.0}),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody curvedBody(converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(curvedBody.IsValid());
    ExpectUnsupportedOperation(IntersectBodies(curvedBody, converted.body));
    ExpectUnsupportedOperation(UnionBodies(curvedBody, converted.body));
    ExpectUnsupportedOperation(DifferenceBodies(curvedBody, converted.body));
}

TEST(BodyBooleanCapabilityTest, EdgeCurveEndpointMismatchReturnsInvalidInput)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCLineCurve3d>(Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection(SCPoint3d{10.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
            Geometry::SCIntervald{0.0, 1.0})),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody inconsistentBody(converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(inconsistentBody.IsValid());
    ExpectInvalidInput(IntersectBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(UnionBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(inconsistentBody, converted.body));
}

TEST(BodyBooleanCapabilityTest, ExplicitZeroDistanceToleranceRemainsStrictAndValid)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    const SCPoint3d start = converted.body.VertexAt(original.StartVertexIndex()).Point();
    const SCPoint3d end = converted.body.VertexAt(original.EndVertexIndex()).Point();
    const SCPoint3d shiftedStart{start.x + 0.5 * Geometry::kDefaultEpsilon, start.y, start.z};
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCLineCurve3d>(Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection(shiftedStart, end - shiftedStart), Geometry::SCIntervald{0.0, 1.0})),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody nearlyConsistentBody(
        converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(nearlyConsistentBody.IsValid());
    EXPECT_EQ(IntersectBodies(nearlyConsistentBody, converted.body).issue, BodyBooleanIssue3d::None);
    BodyBooleanOptions3d strictOptions;
    strictOptions.tolerance.distanceEpsilon = 0.0;
    ExpectInvalidInput(IntersectBodies(nearlyConsistentBody, converted.body, strictOptions));
}

TEST(BodyBooleanCapabilityTest, PlaneSupportMismatchReturnsInvalidInput)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces = shell.Faces();
    const Geometry::SCBrepFace original = faces.front();
    const auto* originalSurface = dynamic_cast<const Geometry::SCPlaneSurface*>(original.SupportSurface());
    ASSERT_NE(originalSurface, nullptr);
    SCPlane shiftedPlane = originalSurface->SupportPlane();
    shiftedPlane.origin = shiftedPlane.origin + shiftedPlane.UnitNormal() * 0.25;
    faces.front() = Geometry::SCBrepFace(std::make_shared<Geometry::SCPlaneSurface>(
                                              Geometry::SCPlaneSurface::FromPlane(shiftedPlane)),
                                          original.OuterLoop(), original.HoleLoops(), original.OuterTrim(),
                                          original.HoleTrims());
    const Geometry::SCBrepBody inconsistentBody(converted.body.Vertices(), converted.body.Edges(),
                                                {Geometry::SCBrepShell(std::move(faces), true)});

    ASSERT_TRUE(inconsistentBody.IsValid());
    ExpectInvalidInput(IntersectBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(UnionBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(inconsistentBody, converted.body));
}

TEST(BodyBooleanCapabilityTest, EquivalentPlaneFacesIgnoreParameterDomainBounds)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces;
    faces.reserve(shell.FaceCount());
    for (const Geometry::SCBrepFace& original : shell.Faces())
    {
        const auto* originalSurface = dynamic_cast<const Geometry::SCPlaneSurface*>(original.SupportSurface());
        ASSERT_NE(originalSurface, nullptr);
        faces.emplace_back(std::make_shared<Geometry::SCPlaneSurface>(Geometry::SCPlaneSurface::FromPlane(
                               originalSurface->SupportPlane(), Geometry::SCIntervald{-10.0, 10.0},
                               Geometry::SCIntervald{-10.0, 10.0})),
                           original.OuterLoop(), original.HoleLoops());
    }
    const Geometry::SCBrepBody reparameterizedBody(converted.body.Vertices(), converted.body.Edges(),
                                                   {Geometry::SCBrepShell(std::move(faces), true)});

    ASSERT_TRUE(reparameterizedBody.IsValid());
    EXPECT_FALSE(reparameterizedBody.Bounds().MinPoint().AlmostEquals(converted.body.Bounds().MinPoint(), 1e-9));
    EXPECT_EQ(IntersectBodies(converted.body, reparameterizedBody).issue, BodyBooleanIssue3d::None);
    EXPECT_EQ(UnionBodies(converted.body, reparameterizedBody).issue, BodyBooleanIssue3d::None);
    const auto difference = DifferenceBodies(converted.body, reparameterizedBody);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(difference.producedEmptyResult);
}

TEST(BodyBooleanCapabilityTest, AngularToleranceControlsNearCoplanarSupportClassification)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedPrismaticBox(0.0, 0.0, 0.2, 1.0, 1.0, 1.2, 5e-10);
    BodyBooleanOptions3d defaultAngleOptions;
    defaultAngleOptions.tolerance.distanceEpsilon = 1e-12;
    const auto defaultAngleResult = IntersectBodies(first, second, defaultAngleOptions);

    BodyBooleanOptions3d strictAngleOptions = defaultAngleOptions;
    strictAngleOptions.tolerance.angleEpsilon = 0.0;
    const auto strictAngleResult = IntersectBodies(first, second, strictAngleOptions);

    ExpectUnsupportedOperation(defaultAngleResult);
    EXPECT_EQ(strictAngleResult.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(strictAngleResult.IsSuccess());
}

TEST(BodyBooleanCapabilityTest, InvalidToleranceReturnsInvalidInput)
{
    BodyBooleanOptions3d options;
    options.tolerance.angleEpsilon = -1.0;

    ExpectInvalidInput(IntersectBodies(Geometry::Test::BuildUnitCubeBody(), Geometry::Test::BuildUnitCubeBody(), options));
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

TEST(BodyBooleanCapabilityTest, OverlappingAxisAlignedBoxUnionReturnsClosedOrthogonalSolid)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody contained = BuildAxisAlignedBoxBody(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
    const auto united = UnionBodies(first, contained);

    ASSERT_EQ(united.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(united.IsSuccess());
    EXPECT_TRUE(united.bodies.empty());
    ASSERT_TRUE(united.body.IsValid());
    ASSERT_EQ(united.body.ShellCount(), 1U);
    EXPECT_TRUE(united.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(united.body), 1.875, 1e-9);
}

TEST(BodyBooleanCapabilityTest, PartialAxisAlignedBoxDifferenceRemainsUnsupported)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.5, 0.5, 0.0, 1.5, 1.5, 1.0);

    const auto difference = DifferenceBodies(first, second);

    ExpectUnsupportedOperation(difference);
}

TEST(BodyBooleanCapabilityTest, FaceTouchingLShapePolyhedronUnionReturnsClosedOrthogonalSolid)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0);
    const PolyhedronBody second = BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);

    const auto result = UnionBodies(first, second);
    const auto reversed = UnionBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.bodies.empty());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(result.body), 3.0, 1e-9);
    const auto bounds = result.body.Bounds();
    EXPECT_TRUE(bounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    EXPECT_TRUE(bounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 1.0}, 1e-12));
    ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(reversed.IsSuccess());
    EXPECT_EQ(reversed.body.FaceCount(), result.body.FaceCount());
    EXPECT_NEAR(Geometry::Volume(reversed.body), Geometry::Volume(result.body), 1e-9);
}

TEST(BodyBooleanCapabilityTest, FaceTouchingLShapeBrepUnionPreservesTopologyAndVolume)
{
    const auto first = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 0.0, 0.0, 2.0, 1.0, 1.0));
    const auto second = ConvertToBrepBody(BuildAxisAlignedBoxBody(0.0, 1.0, 0.0, 1.0, 2.0, 1.0));
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = UnionBodies(second.body, first.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_TRUE(result.bodies.empty());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_NEAR(Geometry::Volume(result.body), 3.0, 1e-9);
}

TEST(BodyBooleanCapabilityTest, RotatedBoxIntersectionReturnsExplicitUnsupportedOperation)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedOverlapBoxBody();

    const auto result = IntersectBodies(first, second);

    ExpectUnsupportedOperation(result);
}

TEST(BodyBooleanCapabilityTest, RotatedBoxUnionAndDifferenceRemainExplicitlyUnsupported)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedOverlapBoxBody();

    const auto united = UnionBodies(first, second);
    const auto difference = DifferenceBodies(first, second);

    ExpectUnsupportedOperation(united);
    ExpectUnsupportedOperation(difference);
}

TEST(BodyBooleanCapabilityTest, EqualBoundsButDifferentRotatedBoxesDoNotUseIdenticalBodyFastPath)
{
    const PolyhedronBody first = BuildRotatedPrismaticBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.23);
    const PolyhedronBody second = BuildRotatedPrismaticBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, -0.23);

    ASSERT_TRUE(first.Bounds().MinPoint().AlmostEquals(second.Bounds().MinPoint(), 1e-12));
    ASSERT_TRUE(first.Bounds().MaxPoint().AlmostEquals(second.Bounds().MaxPoint(), 1e-12));
    ExpectUnsupportedOperation(IntersectBodies(first, second));
    ExpectUnsupportedOperation(UnionBodies(first, second));
    ExpectUnsupportedOperation(DifferenceBodies(first, second));
}

TEST(BodyBooleanCapabilityTest, NonCoplanarRotatedConvexBoxesIntersectAsClosedSingleShell)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildNonCoplanarRotatedOverlapBoxBody();

    const auto result = IntersectBodies(first, second);
    const auto reversed = IntersectBodies(second, first);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_GT(Geometry::Volume(result.body), 0.0);
    ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(reversed.IsSuccess());
    EXPECT_NEAR(Geometry::Volume(reversed.body), Geometry::Volume(result.body), 1e-9);
    EXPECT_TRUE(reversed.body.Bounds().MinPoint().AlmostEquals(result.body.Bounds().MinPoint(), 1e-9));
    EXPECT_TRUE(reversed.body.Bounds().MaxPoint().AlmostEquals(result.body.Bounds().MaxPoint(), 1e-9));
}

TEST(BodyBooleanCapabilityTest, NonCoplanarRotatedConvexBrepBoxesIntersectAsClosedSingleShell)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildNonCoplanarRotatedOverlapBoxBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    const auto result = IntersectBodies(first.body, second.body);

    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    ASSERT_TRUE(result.body.IsValid());
    ASSERT_EQ(result.body.ShellCount(), 1U);
    EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
    EXPECT_GT(Geometry::Volume(result.body), 0.0);
}

TEST(BodyBooleanCapabilityTest, RotatedPrismaticBoxIntersectionMatchesIndependentHalfSpaceClippingOracle)
{
    std::mt19937 generator(20260902U);
    std::uniform_real_distribution<double> centerOffset(-0.25, 0.25);
    std::uniform_real_distribution<double> angleDistribution(0.12, 0.68);
    std::uniform_real_distribution<double> zDistribution(0.08, 0.32);

    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    for (int sample = 0; sample < 24; ++sample)
    {
        const double centerX = 0.5 + centerOffset(generator);
        const double centerY = 0.5 + centerOffset(generator);
        const double minZ = zDistribution(generator);
        const double angleRadians = angleDistribution(generator);
        const double minX = centerX - 0.5;
        const double minY = centerY - 0.5;
        const double maxX = centerX + 0.5;
        const double maxY = centerY + 0.5;
        const double maxZ = minZ + 1.0;
        const PolyhedronBody second =
            BuildRotatedPrismaticBox(minX, minY, minZ, maxX, maxY, maxZ, angleRadians);
        const double expectedVolume =
            RotatedPrismaticBoxIntersectionVolumeOracle(minX, minY, minZ, maxX, maxY, maxZ, angleRadians);

        ASSERT_GT(expectedVolume, 1e-5);
        const auto result = IntersectBodies(first, second);
        const auto reversed = IntersectBodies(second, first);
        ASSERT_EQ(result.issue, BodyBooleanIssue3d::None) << "sample=" << sample;
        ASSERT_EQ(reversed.issue, BodyBooleanIssue3d::None) << "sample=" << sample;
        ASSERT_TRUE(result.body.IsValid());
        ASSERT_EQ(result.body.ShellCount(), 1U);
        EXPECT_TRUE(result.body.ShellAt(0).IsClosed());
        EXPECT_NEAR(Geometry::Volume(result.body), expectedVolume, 1e-8) << "sample=" << sample;
        EXPECT_NEAR(Geometry::Volume(reversed.body), expectedVolume, 1e-8) << "sample=" << sample;
    }
}

TEST(BodyBooleanCapabilityTest, BrepRotatedBoxOperationsReturnExplicitUnsupportedOperation)
{
    const auto first = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    const auto second = ConvertToBrepBody(BuildRotatedOverlapBoxBody());
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);

    ExpectUnsupportedOperation(IntersectBodies(first.body, second.body));
    ExpectUnsupportedOperation(UnionBodies(first.body, second.body));
    ExpectUnsupportedOperation(DifferenceBodies(first.body, second.body));
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

TEST(BodyBooleanCapabilityTest, SupportedResultsPreserveClosedValidTopology)
{
    const PolyhedronBody first = BuildAxisAlignedBoxBody(-0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
    const PolyhedronBody second = Geometry::Test::BuildUnitCubeBody();

    const auto intersection = IntersectBodies(first, second);
    ASSERT_EQ(intersection.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(intersection.IsSuccess());
    ASSERT_TRUE(intersection.body.IsValid());
    ASSERT_EQ(intersection.body.ShellCount(), 1U);
    EXPECT_TRUE(intersection.body.ShellAt(0).IsClosed());

    const auto united = UnionBodies(first, second);
    ASSERT_EQ(united.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(united.IsSuccess());
    ASSERT_TRUE(united.body.IsValid());
    ASSERT_EQ(united.body.ShellCount(), 1U);
    EXPECT_TRUE(united.body.ShellAt(0).IsClosed());
}
