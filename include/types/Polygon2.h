#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include "algorithm/Predicate2.h"
#include "types/ArcSegment2.h"
#include "types/Box2.h"
#include "types/Polyline2.h"

namespace geometry
{
template <typename T>
using PolygonMeasureType = std::conditional_t<std::is_floating_point_v<T>, T, double>;

template <typename T>
class Polygon2
{
public:
    using ValueType = T;
    using PolylineType = Polyline2<T>;
    using PointType = Point2<T>;
    using MeasureType = PolygonMeasureType<T>;

    Polygon2() = default;

    explicit Polygon2(PolylineType outerRing)
        : outerRing_(std::move(outerRing))
    {
    }

    Polygon2(PolylineType outerRing, std::vector<PolylineType> holes)
        : outerRing_(std::move(outerRing)), holes_(std::move(holes))
    {
    }

    [[nodiscard]] const PolylineType& GetOuterRing() const
    {
        return outerRing_;
    }

    [[nodiscard]] std::size_t GetHoleCount() const
    {
        return holes_.size();
    }

    [[nodiscard]] const PolylineType& GetHole(std::size_t index) const
    {
        assert(index < holes_.size());
        return holes_[index];
    }

    [[nodiscard]] MeasureType Area() const
    {
        if (!IsValid())
        {
            return MeasureType{};
        }

        const MeasureType signedArea = RingSignedArea(outerRing_);
        MeasureType total = signedArea;
        for (const auto& hole : holes_)
        {
            total += RingSignedArea(hole);
        }

        return std::abs(total);
    }

    [[nodiscard]] MeasureType Perimeter() const
    {
        if (!IsValid())
        {
            return MeasureType{};
        }

        MeasureType total = static_cast<MeasureType>(outerRing_.Length());
        for (const auto& hole : holes_)
        {
            total += static_cast<MeasureType>(hole.Length());
        }

        return total;
    }

    [[nodiscard]] Point2<MeasureType> Centroid() const
    {
        if (!IsValid())
        {
            return Point2<MeasureType>{};
        }

        const auto outer = RingCentroidAndArea(outerRing_);
        MeasureType weightedX = outer.first.x * outer.second;
        MeasureType weightedY = outer.first.y * outer.second;
        MeasureType signedArea = outer.second;

        for (const auto& hole : holes_)
        {
            const auto ring = RingCentroidAndArea(hole);
            weightedX += ring.first.x * ring.second;
            weightedY += ring.first.y * ring.second;
            signedArea += ring.second;
        }

        if (IsZero(signedArea))
        {
            return Point2<MeasureType>{};
        }

        return Point2<MeasureType>(weightedX / signedArea, weightedY / signedArea);
    }

    [[nodiscard]] Box2<T> GetBoundingBox() const
    {
        if (!IsValid())
        {
            return Box2<T>();
        }

        Box2<T> box = outerRing_.GetBoundingBox();
        for (const auto& hole : holes_)
        {
            box.ExpandToInclude(hole.GetBoundingBox());
        }

        return box;
    }

    [[nodiscard]] bool IsValid() const
    {
        if (!outerRing_.IsValid() || !outerRing_.IsClosed())
        {
            return false;
        }

        const MeasureType outerArea = RingSignedArea(outerRing_);
        if (!(outerArea > MeasureType{}))
        {
            return false;
        }

        for (const auto& hole : holes_)
        {
            if (!hole.IsValid() || !hole.IsClosed())
            {
                return false;
            }

            const MeasureType holeArea = RingSignedArea(hole);
            if (!(holeArea < MeasureType{}))
            {
                return false;
            }
        }

        return true;
    }

private:
    struct RingMoment
    {
        MeasureType signedArea{};
        MeasureType firstMomentX{};
        MeasureType firstMomentY{};
    };

    [[nodiscard]] static MeasureType RingSignedArea(const PolylineType& ring)
    {
        return RingMomentIntegral(ring).signedArea;
    }

    [[nodiscard]] static std::pair<Point2<MeasureType>, MeasureType> RingCentroidAndArea(
        const PolylineType& ring)
    {
        const RingMoment moment = RingMomentIntegral(ring);
        if (IsZero(moment.signedArea))
        {
            return {Point2<MeasureType>{}, MeasureType{}};
        }

        return {
            Point2<MeasureType>(
                moment.firstMomentX / moment.signedArea,
                moment.firstMomentY / moment.signedArea),
            moment.signedArea};
    }

    [[nodiscard]] static RingMoment RingMomentIntegral(const PolylineType& ring)
    {
        RingMoment total{};
        const std::size_t segmentCount = ring.GetSegmentCount();
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const auto& segment = ring.GetSegment(i);
            switch (segment.GetKind())
            {
            case SegmentKind2::Line:
                AccumulateLineSegmentMoment(segment.GetStartPoint(), segment.GetEndPoint(), total);
                break;
            case SegmentKind2::Arc:
                AccumulateArcSegmentMoment(segment, total);
                break;
            default:
                break;
            }
        }

        return total;
    }

    static void AccumulateLineSegmentMoment(
        const PointType& startPoint,
        const PointType& endPoint,
        RingMoment& total)
    {
        const MeasureType x0 = static_cast<MeasureType>(startPoint.x);
        const MeasureType y0 = static_cast<MeasureType>(startPoint.y);
        const MeasureType x1 = static_cast<MeasureType>(endPoint.x);
        const MeasureType y1 = static_cast<MeasureType>(endPoint.y);
        const MeasureType cross = x0 * y1 - x1 * y0;

        total.signedArea += cross / static_cast<MeasureType>(2);
        total.firstMomentX += (y1 - y0) * (x0 * x0 + x0 * x1 + x1 * x1) / static_cast<MeasureType>(6);
        total.firstMomentY += (x0 - x1) * (y0 * y0 + y0 * y1 + y1 * y1) / static_cast<MeasureType>(6);
    }

    static void AccumulateArcSegmentMoment(const Segment2<T>& segment, RingMoment& total)
    {
        const auto* arc = dynamic_cast<const ArcSegment2<T>*>(&segment);
        assert(arc != nullptr);
        if (arc == nullptr)
        {
            return;
        }

        const MeasureType cx = static_cast<MeasureType>(arc->GetCenter().x);
        const MeasureType cy = static_cast<MeasureType>(arc->GetCenter().y);
        const MeasureType radius = static_cast<MeasureType>(arc->GetRadius());
        const MeasureType startAngle = static_cast<MeasureType>(arc->GetStartAngle());
        const MeasureType sweep = static_cast<MeasureType>(arc->GetSignedSweep());
        const MeasureType endAngle = startAngle + sweep;

        const MeasureType sinStart = static_cast<MeasureType>(std::sin(static_cast<double>(startAngle)));
        const MeasureType cosStart = static_cast<MeasureType>(std::cos(static_cast<double>(startAngle)));
        const MeasureType sinEnd = static_cast<MeasureType>(std::sin(static_cast<double>(endAngle)));
        const MeasureType cosEnd = static_cast<MeasureType>(std::cos(static_cast<double>(endAngle)));

        total.signedArea += (cx * radius * (sinEnd - sinStart) -
                             cy * radius * (cosEnd - cosStart) +
                             radius * radius * sweep) /
                            static_cast<MeasureType>(2);

        total.firstMomentX += ArcFirstMomentX(cx, radius, startAngle, endAngle);
        total.firstMomentY += ArcFirstMomentY(cy, radius, startAngle, endAngle);
    }

    [[nodiscard]] static MeasureType ArcFirstMomentX(
        MeasureType centerX,
        MeasureType radius,
        MeasureType startAngle,
        MeasureType endAngle)
    {
        return ArcFirstMomentXPrimitive(centerX, radius, endAngle) -
               ArcFirstMomentXPrimitive(centerX, radius, startAngle);
    }

    [[nodiscard]] static MeasureType ArcFirstMomentXPrimitive(
        MeasureType centerX,
        MeasureType radius,
        MeasureType angle)
    {
        const MeasureType sinAngle = static_cast<MeasureType>(std::sin(static_cast<double>(angle)));
        const MeasureType cosAngle = static_cast<MeasureType>(std::cos(static_cast<double>(angle)));
        return (centerX * centerX * radius * sinAngle +
                centerX * radius * radius * (angle + sinAngle * cosAngle) +
                radius * radius * radius * (sinAngle - sinAngle * sinAngle * sinAngle / static_cast<MeasureType>(3))) /
               static_cast<MeasureType>(2);
    }

    [[nodiscard]] static MeasureType ArcFirstMomentY(
        MeasureType centerY,
        MeasureType radius,
        MeasureType startAngle,
        MeasureType endAngle)
    {
        return ArcFirstMomentYPrimitive(centerY, radius, endAngle) -
               ArcFirstMomentYPrimitive(centerY, radius, startAngle);
    }

    [[nodiscard]] static MeasureType ArcFirstMomentYPrimitive(
        MeasureType centerY,
        MeasureType radius,
        MeasureType angle)
    {
        const MeasureType sinAngle = static_cast<MeasureType>(std::sin(static_cast<double>(angle)));
        const MeasureType cosAngle = static_cast<MeasureType>(std::cos(static_cast<double>(angle)));
        return (-centerY * centerY * radius * cosAngle +
                centerY * radius * radius * (angle - sinAngle * cosAngle) +
                radius * radius * radius * (-cosAngle + cosAngle * cosAngle * cosAngle / static_cast<MeasureType>(3))) /
               static_cast<MeasureType>(2);
    }

    PolylineType outerRing_{};
    std::vector<PolylineType> holes_{};
};

using Polygon2d = Polygon2<double>;
using Polygon2i = Polygon2<int>;
}
