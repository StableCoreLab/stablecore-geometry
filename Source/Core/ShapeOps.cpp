#include "Core/ShapeOps.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "RingIntegral2d.h"
#include "Core/Algorithms.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::shared_ptr<ISCSegment2d> CloneSharedSegment(const ISCSegment2d& segment)
        {
            return std::shared_ptr<ISCSegment2d>(segment.Clone().release());
        }

        [[nodiscard]] double SignedArea(const SCPolyline2d& ring)
        {
            return Detail::ComputeSignedArea(ring);
        }

        [[nodiscard]] SCPoint2d RingCentroid(const SCPolyline2d& ring)
        {
            const Detail::RingMoment2d moment = Detail::ComputeRingMoment(ring);
            if (std::abs(moment.signedArea) <= Geometry::kShapeOpsDefaultEpsilon)
            {
                return SCPoint2d{};
            }

            return SCPoint2d{moment.firstMomentX / moment.signedArea, moment.firstMomentY / moment.signedArea};
        }

    }  // namespace

    double Perimeter(const SCPolygon2d& polygon)
    {
        return polygon.Perimeter();
    }

    SCRingOrientation2d Orientation(const SCPolyline2d& ring)
    {
        const double signedArea = SignedArea(ring);
        if (std::abs(signedArea) <= Geometry::kShapeOpsDefaultEpsilon)
        {
            return SCRingOrientation2d::Unknown;
        }

        return signedArea > 0.0 ? SCRingOrientation2d::CounterClockwise : SCRingOrientation2d::Clockwise;
    }

    bool IsClockwise(const SCPolyline2d& ring)
    {
        return Orientation(ring) == SCRingOrientation2d::Clockwise;
    }

    bool IsCounterClockwise(const SCPolyline2d& ring)
    {
        return Orientation(ring) == SCRingOrientation2d::CounterClockwise;
    }

    SCPoint2d Centroid(const SCPolygon2d& polygon)
    {
        return polygon.Centroid();
    }

    SCLineSegment2d Reverse(const SCLineSegment2d& segment)
    {
        return SCLineSegment2d(segment.endPoint, segment.startPoint);
    }

    SCArcSegment2d Reverse(const SCArcSegment2d& segment)
    {
        return SCArcSegment2d(segment.center, segment.radius, segment.EndAngle(), -segment.sweepAngle);
    }

    SCPolyline2d Reverse(const SCPolyline2d& polyline)
    {
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.reserve(polyline.SegmentCount());
        for (std::size_t i = polyline.SegmentCount(); i > 0; --i)
        {
            std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i - 1);
            if (segment == nullptr)
            {
                segments.push_back(nullptr);
                continue;
            }

            switch (segment->Kind())
            {
                case SCSegmentKind2::Line:
                    segments.push_back(
                        std::make_shared<SCLineSegment2d>(Reverse(static_cast<const SCLineSegment2d&>(*segment))));
                    break;
                case SCSegmentKind2::Arc:
                    segments.push_back(
                        std::make_shared<SCArcSegment2d>(Reverse(static_cast<const SCArcSegment2d&>(*segment))));
                    break;
                default:
                    segments.push_back(std::shared_ptr<ISCSegment2d>(std::move(segment)));
                    break;
            }
        }

        return SCPolyline2d(std::move(segments), polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
    }

    SCPolyline2d Close(const SCPolyline2d& polyline)
    {
        if (polyline.IsClosed())
        {
            return polyline;
        }

        if (polyline.SegmentCount() == 0)
        {
            return SCPolyline2d(SCPolylineClosure::Closed);
        }

        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.reserve(polyline.SegmentCount() + 1);
        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
            if (segment == nullptr)
            {
                continue;
            }
            segments.push_back(CloneSharedSegment(*segment));
        }

        if (segments.empty())
        {
            return SCPolyline2d(SCPolylineClosure::Closed);
        }

        if (!segments.back()->EndPoint().AlmostEquals(segments.front()->StartPoint()))
        {
            segments.push_back(
                std::make_shared<SCLineSegment2d>(segments.back()->EndPoint(), segments.front()->StartPoint()));
        }

        return SCPolyline2d(std::move(segments), SCPolylineClosure::Closed);
    }
}  // namespace Geometry

