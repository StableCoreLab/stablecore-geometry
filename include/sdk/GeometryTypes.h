#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "common/Epsilon.h"
#include "export/GeometryExport.h"
#include "types/Box2.h"
#include "types/Box3.h"
#include "types/Direction3.h"
#include "types/Interval.h"
#include "types/Line3.h"
#include "types/LineSegment3.h"
#include "types/Matrix3.h"
#include "types/Plane.h"
#include "types/Point2.h"
#include "types/Point3.h"
#include "types/Ray3.h"
#include "types/Transform3.h"
#include "types/Triangle3.h"
#include "types/Vector2.h"
#include "types/Vector3.h"

namespace geometry::sdk
{
using geometry::Box2d;
using geometry::Box3d;
using geometry::Direction3d;
using geometry::Intervald;
using geometry::Line3d;
using geometry::LineSegment3d;
using geometry::Matrix3d;
using geometry::Plane;
using geometry::Point2d;
using geometry::Point3d;
using geometry::Ray3d;
using geometry::Transform3d;
using geometry::Triangle3d;
using geometry::Vector2d;
using geometry::Vector3d;

struct GEOMETRY_API SegmentProjection2d
{
    Point2d point{};
    double parameter{0.0};
    double distanceSquared{0.0};
    bool isOnSegment{false};

    [[nodiscard]] bool IsValid() const
    {
        return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
               distanceSquared >= 0.0;
    }

    [[nodiscard]] bool AlmostEquals(
        const SegmentProjection2d& other,
        double eps = geometry::kDefaultEpsilon) const
    {
        return point.AlmostEquals(other.point, eps) &&
               std::abs(parameter - other.parameter) <= eps &&
               std::abs(distanceSquared - other.distanceSquared) <= eps &&
               isOnSegment == other.isOnSegment;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "SegmentProjection2d{point=" << point.DebugString()
               << ", parameter=" << parameter
               << ", distanceSquared=" << distanceSquared
               << ", isOnSegment=" << (isOnSegment ? "true" : "false") << "}";
        return stream.str();
    }
};

struct GEOMETRY_API GeometryTolerance3d
{
    double distanceEpsilon{geometry::kDefaultEpsilon};
    double angleEpsilon{geometry::kDefaultEpsilon};
    double parameterEpsilon{geometry::kDefaultEpsilon};
    double boxPadding{geometry::kDefaultEpsilon};

    [[nodiscard]] bool IsValid() const
    {
        return std::isfinite(distanceEpsilon) && distanceEpsilon >= 0.0 &&
               std::isfinite(angleEpsilon) && angleEpsilon >= 0.0 &&
               std::isfinite(parameterEpsilon) && parameterEpsilon >= 0.0 &&
               std::isfinite(boxPadding) && boxPadding >= 0.0;
    }
};

struct GEOMETRY_API GeometryContext3d
{
    GeometryTolerance3d tolerance{};
    bool enableDiagnostics{false};

    [[nodiscard]] bool IsValid() const
    {
        return tolerance.IsValid();
    }
};

struct GEOMETRY_API LineProjection3d
{
    Point3d point{};
    double parameter{0.0};
    double distanceSquared{0.0};
    bool isOnLine{false};

    [[nodiscard]] bool IsValid() const
    {
        return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
               distanceSquared >= 0.0;
    }
};

struct GEOMETRY_API PlaneProjection3d
{
    Point3d point{};
    double signedDistance{0.0};
    double distanceSquared{0.0};

    [[nodiscard]] bool IsValid() const
    {
        return point.IsValid() && std::isfinite(signedDistance) && std::isfinite(distanceSquared) &&
               distanceSquared >= 0.0;
    }
};

struct GEOMETRY_API SurfaceProjection3d
{
    bool success{false};
    Point3d point{};
    double u{0.0};
    double v{0.0};
    double distanceSquared{0.0};

    [[nodiscard]] bool IsValid() const
    {
        return !success || (point.IsValid() && std::isfinite(u) && std::isfinite(v) &&
                            std::isfinite(distanceSquared) && distanceSquared >= 0.0);
    }
};

struct GEOMETRY_API BrepFaceProjection3d
{
    bool success{false};
    bool onTrimmedFace{false};
    bool onBoundary{false};
    Point3d point{};
    double u{0.0};
    double v{0.0};
    double distanceSquared{0.0};

    [[nodiscard]] bool IsValid() const
    {
        return !success || (point.IsValid() && std::isfinite(u) && std::isfinite(v) &&
                            std::isfinite(distanceSquared) && distanceSquared >= 0.0);
    }
};

struct GEOMETRY_API LinePlaneIntersection3d
{
    bool intersects{false};
    bool isParallel{false};
    bool liesOnPlane{false};
    double parameter{0.0};
    Point3d point{};

    [[nodiscard]] bool IsValid() const
    {
        return !intersects || point.IsValid();
    }
};

struct GEOMETRY_API LineSurfaceIntersection3d
{
    bool intersects{false};
    bool isParallel{false};
    bool liesOnSurface{false};
    double lineParameter{0.0};
    double u{0.0};
    double v{0.0};
    Point3d point{};

    [[nodiscard]] bool IsValid() const
    {
        return !intersects || (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(u) &&
                               std::isfinite(v));
    }
};

struct GEOMETRY_API LineBrepFaceIntersection3d
{
    bool intersects{false};
    bool onBoundary{false};
    double lineParameter{0.0};
    double u{0.0};
    double v{0.0};
    Point3d point{};

    [[nodiscard]] bool IsValid() const
    {
        return !intersects || (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(u) &&
                               std::isfinite(v));
    }
};

struct GEOMETRY_API LineBrepBodyIntersection3d
{
    bool intersects{false};
    std::vector<std::size_t> faceIndices{};
    std::vector<LineBrepFaceIntersection3d> hits{};

    [[nodiscard]] bool IsValid() const
    {
        if (!intersects)
        {
            return true;
        }

        if (faceIndices.size() != hits.size())
        {
            return false;
        }

        for (const LineBrepFaceIntersection3d& hit : hits)
        {
            if (!hit.IsValid())
            {
                return false;
            }
        }
        return true;
    }
};

struct GEOMETRY_API PlanePlaneIntersection3d
{
    bool intersects{false};
    bool isParallel{false};
    bool isCoincident{false};
    Line3d line{};

    [[nodiscard]] bool IsValid() const
    {
        return !intersects || line.IsValid();
    }
};

struct GEOMETRY_API CurveEval3d
{
    Point3d point{};
    Vector3d firstDerivative{};
    Vector3d secondDerivative{};
    int derivativeOrder{0};

    [[nodiscard]] bool IsValid() const
    {
        if (!point.IsValid() || derivativeOrder < 0 || derivativeOrder > 2)
        {
            return false;
        }

        if (derivativeOrder >= 1 && !firstDerivative.IsValid())
        {
            return false;
        }

        if (derivativeOrder >= 2 && !secondDerivative.IsValid())
        {
            return false;
        }

        return true;
    }
};

struct GEOMETRY_API SurfaceEval3d
{
    Point3d point{};
    Vector3d derivativeU{};
    Vector3d derivativeV{};
    Vector3d normal{};
    int derivativeOrder{0};

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (!point.IsValid() || derivativeOrder < 0 || derivativeOrder > 1)
        {
            return false;
        }

        if (derivativeOrder >= 1)
        {
            if (!derivativeU.IsValid() || !derivativeV.IsValid() || !normal.IsValid())
            {
                return false;
            }

            if (normal.Length() <= eps)
            {
                return false;
            }
        }

        return true;
    }
};

} // namespace geometry::sdk
