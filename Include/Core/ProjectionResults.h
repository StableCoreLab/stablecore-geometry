#pragma once

#include <cstddef>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Core/GeometryTypesPrimitives.h"

namespace Geometry
{
    struct GEOMETRY_API SCSegmentProjection3d
    {
        SCPoint3d point{};
        double parameter{0.0};
        double distanceSquared{0.0};
        bool isOnSegment{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                   distanceSquared >= 0.0;
        }

        [[nodiscard]] bool AlmostEquals(const SCSegmentProjection3d& other, double eps = Geometry::kDefaultEpsilon) const
        {
            return point.AlmostEquals(other.point, eps) && std::abs(parameter - other.parameter) <= eps &&
                   std::abs(distanceSquared - other.distanceSquared) <= eps && isOnSegment == other.isOnSegment;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCSegmentProjection3d{point=" << point.DebugString() << ", parameter=" << parameter
                   << ", distanceSquared=" << distanceSquared << ", isOnSegment=" << (isOnSegment ? "true" : "false")
                   << "}";
            return stream.str();
        }
    };

    struct GEOMETRY_API SCGeometryContext3d
    {
        SCGeometryTolerance3d tolerance{};
        bool enableDiagnostics{false};

        [[nodiscard]] bool IsValid() const
        {
            return tolerance.IsValid();
        }
    };

    struct GEOMETRY_API SCLineProjection3d
    {
        SCPoint3d point{};
        double parameter{0.0};
        double distanceSquared{0.0};
        bool isOnLine{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                   distanceSquared >= 0.0;
        }
    };

    struct GEOMETRY_API SCPlaneProjection3d
    {
        SCPoint3d point{};
        double signedDistance{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && std::isfinite(signedDistance) && std::isfinite(distanceSquared) &&
                   distanceSquared >= 0.0;
        }
    };

    struct GEOMETRY_API SCSurfaceProjection3d
    {
        bool success{false};
        SCPoint3d point{};
        double u{0.0};
        double v{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(u) && std::isfinite(v) &&
                                std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCCurveProjection3d
    {
        bool success{false};
        SCPoint3d point{};
        double parameter{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                                distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCCurveOnSurfaceProjection3d
    {
        bool success{false};
        SCPoint3d point{};
        std::size_t segmentIndex{0};
        double segmentParameter{0.0};
        SCPoint2d uv{};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && uv.IsValid() && std::isfinite(segmentParameter) &&
                                std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCBrepFaceProjection3d
    {
        bool success{false};
        bool onTrimmedFace{false};
        bool onBoundary{false};
        SCPoint3d point{};
        double u{0.0};
        double v{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(u) && std::isfinite(v) &&
                                std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCBrepBodyProjection3d
    {
        bool success{false};
        std::size_t faceIndex{0};
        SCBrepFaceProjection3d projection{};

        [[nodiscard]] bool IsValid() const
        {
            return !success || projection.IsValid();
        }
    };

    struct GEOMETRY_API SCBrepEdgeProjection3d
    {
        bool success{false};
        double parameter{0.0};
        SCPoint3d point{};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                                distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCBrepVertexProjection3d
    {
        bool success{false};
        SCPoint3d point{};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCPolyhedronFaceProjection3d
    {
        bool success{false};
        bool onFace{false};
        bool onBoundary{false};
        SCPoint3d point{};
        double u{0.0};
        double v{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(u) && std::isfinite(v) &&
                                std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

    struct GEOMETRY_API SCPolyhedronBodyProjection3d
    {
        bool success{false};
        std::size_t faceIndex{0};
        SCPolyhedronFaceProjection3d projection{};

        [[nodiscard]] bool IsValid() const
        {
            return !success || projection.IsValid();
        }
    };

    struct GEOMETRY_API SCTriangleMeshProjection3d
    {
        bool success{false};
        std::size_t triangleIndex{0};
        SCPoint3d point{};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !success || (point.IsValid() && std::isfinite(distanceSquared) && distanceSquared >= 0.0);
        }
    };

}  // namespace Geometry
