#pragma once

#include <array>
#include <memory>

#include "Core/GeometryTypesPrimitives.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "Types/Geometry3d/SCPlane.h"

namespace Geometry
{
    enum class SCRingOrientation2d
    {
        Unknown,
        Clockwise,
        CounterClockwise
    };

    enum class SCIntersectionKind2d
    {
        None,
        Point,
        Overlap,
        Tangent
    };

    enum class SCExtensionPolicy
    {
        None,
        ExtendFirst,
        ExtendSecond,
        ExtendBoth
    };

    struct GEOMETRY_API SCIntersectionPoint2d
    {
        SCPoint2d point{};
        double parameterOnFirst{0.0};
        double parameterOnSecond{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid();
        }

        [[nodiscard]] bool AlmostEquals(const SCIntersectionPoint2d& other, double eps = Geometry::kDefaultEpsilon) const
        {
            return point.AlmostEquals(other.point, eps) && std::abs(parameterOnFirst - other.parameterOnFirst) <= eps &&
                   std::abs(parameterOnSecond - other.parameterOnSecond) <= eps;
        }
    };

    struct GEOMETRY_API SCSegmentIntersection2d
    {
        SCIntersectionKind2d kind{SCIntersectionKind2d::None};
        std::array<SCIntersectionPoint2d, 2> points{}; 
        std::size_t pointCount{0};

        [[nodiscard]] bool HasIntersection() const
        {
            return kind != SCIntersectionKind2d::None;
        }
    };

    struct GEOMETRY_API SCLineIntersection2d
    {
        SCIntersectionKind2d kind{SCIntersectionKind2d::None};
        std::array<SCIntersectionPoint2d, 2> points{};
        std::size_t pointCount{0};
        bool parallel{false};
        bool collinear{false};
        bool infiniteOverlap{false};

        [[nodiscard]] bool HasIntersection() const
        {
            return kind != SCIntersectionKind2d::None;
        }
    };

    struct GEOMETRY_API SCExtendedIntersection2d
    {
        SCIntersectionKind2d kind{SCIntersectionKind2d::None};
        std::array<SCIntersectionPoint2d, 2> points{};
        std::size_t pointCount{0};
        bool onFirstSegment{false};
        bool onSecondSegment{false};
        bool infiniteOverlap{false};

        [[nodiscard]] bool HasIntersection() const
        {
            return kind != SCIntersectionKind2d::None;
        }
    };

    struct GEOMETRY_API SCPolylineIntersectionPoint2d
    {
        SCIntersectionKind2d kind{SCIntersectionKind2d::None};
        SCPoint2d point{};
        std::size_t segmentIndexOnFirst{0};
        std::size_t segmentIndexOnSecond{0};
        double parameterOnFirstSegment{0.0};
        double parameterOnSecondSegment{0.0};
        double globalParameterOnFirst{0.0};
        double globalParameterOnSecond{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return kind != SCIntersectionKind2d::None && point.IsValid();
        }
    };

    struct GEOMETRY_API SCClosestPoints2d
    {
        SCPoint2d firstPoint{};
        SCPoint2d secondPoint{};
        double parameterOnFirst{0.0};
        double parameterOnSecond{0.0};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return firstPoint.IsValid() && secondPoint.IsValid() && distanceSquared >= 0.0;
        }
    };

    struct GEOMETRY_API SCAxisSample2d
    {
        SCPoint2d point{};
        SCVector2d tangent{};
        SCVector2d normal{};
        double parameter{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && tangent.IsValid() && normal.IsValid();
        }
    };

    struct GEOMETRY_API SCAxisProjection2d
    {
        SCSegmentProjection2d projection{};
        SCVector2d tangent{};
        SCVector2d normal{};

        [[nodiscard]] bool IsValid() const
        {
            return projection.IsValid() && tangent.IsValid() && normal.IsValid();
        }
    };

    struct GEOMETRY_API SCSegmentTrim2d
    {
        bool success{false};
        std::unique_ptr<ISCSegment2d> segment{};

        SCSegmentTrim2d() = default;
        SCSegmentTrim2d(SCSegmentTrim2d&&) noexcept = default;
        SCSegmentTrim2d& operator=(SCSegmentTrim2d&&) noexcept = default;
        SCSegmentTrim2d(const SCSegmentTrim2d&) = delete;
        SCSegmentTrim2d& operator=(const SCSegmentTrim2d&) = delete;
    };

    struct GEOMETRY_API SCSegmentSplit2d
    {
        bool success{false};
        std::unique_ptr<ISCSegment2d> first{};
        std::unique_ptr<ISCSegment2d> second{};

        SCSegmentSplit2d() = default;
        SCSegmentSplit2d(SCSegmentSplit2d&&) noexcept = default;
        SCSegmentSplit2d& operator=(SCSegmentSplit2d&&) noexcept = default;
        SCSegmentSplit2d(const SCSegmentSplit2d&) = delete;
        SCSegmentSplit2d& operator=(const SCSegmentSplit2d&) = delete;
    };

    struct GEOMETRY_API SCSnapResult2d
    {
        bool snapped{false};
        SCPoint2d point{};
        double distanceSquared{0.0};
        std::size_t segmentIndex{0};
        double parameter{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return !snapped || point.IsValid();
        }
    };
}  // namespace Geometry





