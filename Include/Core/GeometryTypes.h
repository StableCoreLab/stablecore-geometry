#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Core/GeometryTypesPrimitives.h"
#include "Core/ProjectionResults.h"
#include "Core/Geometry2dResults.h"
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

    // Projection results are declared in ProjectionResults.h.
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








