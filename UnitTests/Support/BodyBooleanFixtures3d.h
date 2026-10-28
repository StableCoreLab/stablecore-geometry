#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Core/BodyBoolean.h"
#include "Brep/BrepConversion.h"
#include "Core/Measure.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Fixtures3d.h"

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
