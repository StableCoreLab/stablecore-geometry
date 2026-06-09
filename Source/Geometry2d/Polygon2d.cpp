#include "Geometry2d/SCPolygon2d.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "../Core/RingIntegral2d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Core/Validation.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::vector<std::unique_ptr<ISCSegment2d>> CollectSegments(const SCPolyline2d& ring)
        {
            std::vector<std::unique_ptr<ISCSegment2d>> segments;
            segments.reserve(ring.SegmentCount());
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                segments.push_back(ring.SegmentAt(i));
            }
            return segments;
        }

        [[nodiscard]] bool HasBoundaryIntersection(const SCPolyline2d& first, const SCPolyline2d& second)
        {
            const std::vector<std::unique_ptr<ISCSegment2d>> firstSegments = CollectSegments(first);
            const std::vector<std::unique_ptr<ISCSegment2d>> secondSegments = CollectSegments(second);

            for (const auto& lhs : firstSegments)
            {
                if (lhs == nullptr)
                {
                    continue;
                }

                for (const auto& rhs : secondSegments)
                {
                    if (rhs == nullptr)
                    {
                        continue;
                    }

                    if (HasIntersection(*lhs, *rhs, Geometry::kDefaultEpsilon))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] double RingSignedArea(const SCPolyline2d& ring)
        {
            return Detail::ComputeSignedArea(ring);
        }

        [[nodiscard]] std::pair<SCPoint2d, double> RingCentroidAndArea(const SCPolyline2d& ring)
        {
            const Detail::RingMoment2d moment = Detail::ComputeRingMoment(ring);
            if (IsZero(moment.signedArea))
            {
                return {SCPoint2d{}, 0.0};
            }

            return {SCPoint2d(moment.firstMomentX / moment.signedArea, moment.firstMomentY / moment.signedArea),
                    moment.signedArea};
        }

    }  // namespace

    struct SCPolygon2d::Impl
    {
        SCPolyline2d outerRing{};
        std::vector<SCPolyline2d> holes{};

        Impl() = default;

        explicit Impl(SCPolyline2d outerRing) : outerRing(std::move(outerRing))
        {
        }

        Impl(SCPolyline2d outerRing, std::vector<SCPolyline2d> holes)
            : outerRing(std::move(outerRing)), holes(std::move(holes))
        {
        }
    };

    SCPolygon2d::SCPolygon2d() : impl_(std::make_unique<Impl>())
    {
    }

    SCPolygon2d::SCPolygon2d(SCPolyline2d outerRing) : impl_(std::make_unique<Impl>(std::move(outerRing)))
    {
    }

    SCPolygon2d::SCPolygon2d(SCPolyline2d outerRing, std::vector<SCPolyline2d> holes)
        : impl_(std::make_unique<Impl>(std::move(outerRing), std::move(holes)))
    {
    }

    SCPolygon2d::SCPolygon2d(const SCPolygon2d& other)
        : impl_(std::make_unique<Impl>(other.impl_->outerRing, other.impl_->holes))
    {
    }

    SCPolygon2d::SCPolygon2d(SCPolygon2d&& other) noexcept = default;

    SCPolygon2d& SCPolygon2d::operator=(const SCPolygon2d& other)
    {
        if (this != &other)
        {
            impl_ = std::make_unique<Impl>(other.impl_->outerRing, other.impl_->holes);
        }
        return *this;
    }

    SCPolygon2d& SCPolygon2d::operator=(SCPolygon2d&& other) noexcept = default;

    SCPolygon2d::~SCPolygon2d() = default;

    bool SCPolygon2d::IsValid() const
    {
        if (!impl_->outerRing.IsValid() || !impl_->outerRing.IsClosed())
        {
            return false;
        }

        if (HasSelfIntersection(impl_->outerRing, Geometry::kDefaultEpsilon))
        {
            return false;
        }

        const double outerArea = RingSignedArea(impl_->outerRing);
        if (!(outerArea > 0.0))
        {
            return false;
        }

        for (const auto& hole : impl_->holes)
        {
            if (!hole.IsValid() || !hole.IsClosed())
            {
                return false;
            }

            if (HasSelfIntersection(hole, Geometry::kDefaultEpsilon))
            {
                return false;
            }

            const double holeArea = RingSignedArea(hole);
            if (!(holeArea < 0.0))
            {
                return false;
            }

            if (HasBoundaryIntersection(impl_->outerRing, hole))
            {
                return false;
            }

            if (hole.PointCount() == 0)
            {
                return false;
            }

            if (LocatePoint(hole.PointAt(0), impl_->outerRing, Geometry::kDefaultEpsilon) != SCPointContainment2d::Inside)
            {
                return false;
            }
        }

        for (std::size_t i = 0; i < impl_->holes.size(); ++i)
        {
            for (std::size_t j = i + 1; j < impl_->holes.size(); ++j)
            {
                if (HasBoundaryIntersection(impl_->holes[i], impl_->holes[j]))
                {
                    return false;
                }

                if (impl_->holes[i].PointCount() == 0 || impl_->holes[j].PointCount() == 0)
                {
                    return false;
                }

                if (LocatePoint(impl_->holes[i].PointAt(0), impl_->holes[j], Geometry::kDefaultEpsilon) ==
                        SCPointContainment2d::Inside ||
                    LocatePoint(impl_->holes[j].PointAt(0), impl_->holes[i], Geometry::kDefaultEpsilon) ==
                        SCPointContainment2d::Inside)
                {
                    return false;
                }
            }
        }

        return true;
    }

    std::size_t SCPolygon2d::PointCount() const
    {
        std::size_t count = impl_->outerRing.VertexCount();
        for (const auto& hole : impl_->holes)
        {
            count += hole.VertexCount();
        }
        return count;
    }

    std::size_t SCPolygon2d::SegmentCount() const
    {
        std::size_t count = impl_->outerRing.SegmentCount();
        for (const auto& hole : impl_->holes)
        {
            count += hole.SegmentCount();
        }
        return count;
    }

    std::size_t SCPolygon2d::HoleCount() const
    {
        return impl_->holes.size();
    }

    SCPolyline2d SCPolygon2d::OuterRing() const
    {
        return impl_->outerRing;
    }

    SCPolyline2d SCPolygon2d::HoleAt(std::size_t index) const
    {
        if (index >= impl_->holes.size())
        {
            throw std::out_of_range("SCPolygon2d::HoleAt index out of range");
        }

        return impl_->holes[index];
    }

    double SCPolygon2d::Area() const
    {
        if (!IsValid())
        {
            return 0.0;
        }

        double total = RingSignedArea(impl_->outerRing);
        for (const auto& hole : impl_->holes)
        {
            total += RingSignedArea(hole);
        }
        return std::abs(total);
    }

    double SCPolygon2d::Perimeter() const
    {
        if (!IsValid())
        {
            return 0.0;
        }

        double total = impl_->outerRing.Length();
        for (const auto& hole : impl_->holes)
        {
            total += hole.Length();
        }
        return total;
    }

    SCPoint2d SCPolygon2d::Centroid() const
    {
        if (!IsValid())
        {
            return SCPoint2d{};
        }

        const auto outer = RingCentroidAndArea(impl_->outerRing);
        double weightedX = outer.first.x * outer.second;
        double weightedY = outer.first.y * outer.second;
        double signedArea = outer.second;

        for (const auto& hole : impl_->holes)
        {
            const auto ring = RingCentroidAndArea(hole);
            weightedX += ring.first.x * ring.second;
            weightedY += ring.first.y * ring.second;
            signedArea += ring.second;
        }

        if (IsZero(signedArea))
        {
            return SCPoint2d{};
        }

        return SCPoint2d(weightedX / signedArea, weightedY / signedArea);
    }

    SCBox2d SCPolygon2d::Bounds() const
    {
        if (!IsValid())
        {
            return SCBox2d{};
        }

        SCBox2d box = impl_->outerRing.Bounds();
        for (const auto& hole : impl_->holes)
        {
            box.ExpandToInclude(hole.Bounds());
        }
        return box;
    }

    std::string SCPolygon2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCPolygon2d{holeCount=" << HoleCount() << ", pointCount=" << PointCount()
               << ", segmentCount=" << SegmentCount() << ", valid=" << (IsValid() ? "true" : "false")
               << ", bounds=" << Bounds().DebugString() << "}";
        return stream.str();
    }
}  // namespace Geometry

