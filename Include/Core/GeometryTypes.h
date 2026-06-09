#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Core/GeometryTypesPrimitives.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCDirection3d.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCMatrix3d.h"
#include "Types/Geometry3d/SCPlane.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCRay3d.h"
#include "Types/Geometry3d/SCTransform3d.h"
#include "Types/Geometry3d/SCTriangle3d.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry
{
    struct SCCurveEval3d;

    using Geometry::SCBox2d;
    using Geometry::SCBox3d;
    using Geometry::SCDirection3d;
    using Geometry::SCIntervald;
    using Geometry::SCLine3d;
    using Geometry::SCLineSegment3d;
    using Geometry::SCMatrix3d;
    using Geometry::SCPlane;
    using Geometry::SCPoint2d;
    using Geometry::SCPoint3d;
    using Geometry::SCRay3d;
    using Geometry::SCTransform3d;
    using Geometry::SCTriangle3d;
    using Geometry::SCVector2d;
    using Geometry::SCVector3d;

    struct GEOMETRY_API SCSegmentProjection2d
    {
        SCPoint2d point{};
        double parameter{0.0};
        double distanceSquared{0.0};
        bool isOnSegment{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && std::isfinite(parameter) && std::isfinite(distanceSquared) &&
                   distanceSquared >= 0.0;
        }

        [[nodiscard]] bool AlmostEquals(const SCSegmentProjection2d& other, double eps = Geometry::kDefaultEpsilon) const
        {
            return point.AlmostEquals(other.point, eps) && std::abs(parameter - other.parameter) <= eps &&
                   std::abs(distanceSquared - other.distanceSquared) <= eps && isOnSegment == other.isOnSegment;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCSegmentProjection2d{point=" << point.DebugString() << ", parameter=" << parameter
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

    struct GEOMETRY_API SCLinePlaneIntersection3d
    {
        bool intersects{false};
        bool isParallel{false};
        bool liesOnPlane{false};
        double parameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || point.IsValid();
        }
    };

    struct GEOMETRY_API SCLineSurfaceIntersection3d
    {
        bool intersects{false};
        bool isParallel{false};
        bool liesOnSurface{false};
        double lineParameter{0.0};
        double u{0.0};
        double v{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects ||
                   (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(u) && std::isfinite(v));
        }
    };

    struct GEOMETRY_API SCLineBrepFaceIntersection3d
    {
        bool intersects{false};
        bool onBoundary{false};
        double lineParameter{0.0};
        double u{0.0};
        double v{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects ||
                   (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(u) && std::isfinite(v));
        }
    };

    struct GEOMETRY_API SCLineBrepBodyIntersection3d
    {
        bool intersects{false};
        std::vector<std::size_t> faceIndices{};
        std::vector<SCLineBrepFaceIntersection3d> hits{};

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

            for (const SCLineBrepFaceIntersection3d& hit : hits)
            {
                if (!hit.IsValid())
                {
                    return false;
                }
            }
            return true;
        }
    };

    struct GEOMETRY_API SCLineBrepEdgeIntersection3d
    {
        bool intersects{false};
        double lineParameter{0.0};
        double edgeParameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(edgeParameter));
        }
    };

    struct GEOMETRY_API SCLineBrepVertexIntersection3d
    {
        bool intersects{false};
        double lineParameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && std::isfinite(lineParameter));
        }
    };

    struct GEOMETRY_API SCLineCurveIntersection3d
    {
        bool intersects{false};
        double lineParameter{0.0};
        double curveParameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(curveParameter));
        }
    };

    struct GEOMETRY_API SCLineCurveOnSurfaceIntersection3d
    {
        bool intersects{false};
        std::size_t segmentIndex{0};
        double segmentParameter{0.0};
        double lineParameter{0.0};
        SCPoint2d uv{};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects ||
                   (point.IsValid() && uv.IsValid() && std::isfinite(segmentParameter) && std::isfinite(lineParameter));
        }
    };

    struct GEOMETRY_API SCLinePolyhedronFaceIntersection3d
    {
        bool intersects{false};
        bool onBoundary{false};
        double lineParameter{0.0};
        double u{0.0};
        double v{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects ||
                   (point.IsValid() && std::isfinite(lineParameter) && std::isfinite(u) && std::isfinite(v));
        }
    };

    struct GEOMETRY_API SCLinePolyhedronBodyIntersection3d
    {
        bool intersects{false};
        std::vector<std::size_t> faceIndices{};
        std::vector<SCLinePolyhedronFaceIntersection3d> hits{};

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

            for (const SCLinePolyhedronFaceIntersection3d& hit : hits)
            {
                if (!hit.IsValid())
                {
                    return false;
                }
            }
            return true;
        }
    };

    struct GEOMETRY_API SCLineTriangleMeshIntersection3d
    {
        bool intersects{false};
        std::vector<std::size_t> triangleIndices{};
        std::vector<double> lineParameters{};
        std::vector<SCPoint3d> points{};

        [[nodiscard]] bool IsValid() const
        {
            if (!intersects)
            {
                return true;
            }

            if (triangleIndices.size() != lineParameters.size() || lineParameters.size() != points.size())
            {
                return false;
            }

            for (const SCPoint3d& point : points)
            {
                if (!point.IsValid())
                {
                    return false;
                }
            }
            return true;
        }
    };

    struct GEOMETRY_API SCPlaneCurveIntersection3d
    {
        bool intersects{false};
        double curveParameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && std::isfinite(curveParameter));
        }
    };

    struct GEOMETRY_API SCPlaneCurveOnSurfaceIntersection3d
    {
        bool intersects{false};
        std::size_t segmentIndex{0};
        double segmentParameter{0.0};
        SCPoint2d uv{};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && uv.IsValid() && std::isfinite(segmentParameter));
        }
    };

    struct GEOMETRY_API SCPlaneBrepEdgeIntersection3d
    {
        bool intersects{false};
        double edgeParameter{0.0};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || (point.IsValid() && std::isfinite(edgeParameter));
        }
    };

    struct GEOMETRY_API SCPlaneBrepVertexIntersection3d
    {
        bool intersects{false};
        SCPoint3d point{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || point.IsValid();
        }
    };

    struct GEOMETRY_API SCPlanePlaneIntersection3d
    {
        bool intersects{false};
        bool isParallel{false};
        bool isCoincident{false};
        SCLine3d line{};

        [[nodiscard]] bool IsValid() const
        {
            return !intersects || line.IsValid();
        }
    };

}  // namespace Geometry

namespace Geometry
{
    namespace Sdk = ::Geometry;
}









