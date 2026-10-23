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

        [[nodiscard]] bool HasBoundaryIntersection(const SCPolyline2d& first, const SCPolyline2d& second,
                                                   double eps)
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

                    if (HasIntersection(*lhs, *rhs, eps))
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
            if (!std::isfinite(moment.signedArea) || moment.signedArea == 0.0)
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
        return IsValid(Geometry::kDefaultEpsilon);
    }

    bool SCPolygon2d::IsValid(double eps) const
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return false;
        }
        if (!impl_->outerRing.IsValid() || !impl_->outerRing.IsClosed())
        {
            return false;
        }

        if (HasSelfIntersection(impl_->outerRing, eps))
        {
            return false;
        }

        const double outerArea = RingSignedArea(impl_->outerRing);
        // 环方向不是结构有效性的前置条件；NormalizePolygon 会在需要方向语义的
        // 算法入口统一规范化。这里仅排除零面积退化环。
        if (!(std::abs(outerArea) > 0.0))
        {
            return false;
        }

        for (const auto& hole : impl_->holes)
        {
            if (!hole.IsValid() || !hole.IsClosed())
            {
                return false;
            }

            if (HasSelfIntersection(hole, eps))
            {
                return false;
            }

            const double holeArea = RingSignedArea(hole);
            if (!(std::abs(holeArea) > 0.0))
            {
                return false;
            }

            if (HasBoundaryIntersection(impl_->outerRing, hole, eps))
            {
                return false;
            }

            if (hole.PointCount() == 0)
            {
                return false;
            }

            if (LocatePoint(hole.VertexAt(0), impl_->outerRing, eps) != SCPointContainment2d::Inside)
            {
                return false;
            }
        }

        for (std::size_t i = 0; i < impl_->holes.size(); ++i)
        {
            for (std::size_t j = i + 1; j < impl_->holes.size(); ++j)
            {
                if (HasBoundaryIntersection(impl_->holes[i], impl_->holes[j], eps))
                {
                    return false;
                }

                if (impl_->holes[i].PointCount() == 0 || impl_->holes[j].PointCount() == 0)
                {
                    return false;
                }

                if (LocatePoint(impl_->holes[i].VertexAt(0), impl_->holes[j], eps) ==
                        SCPointContainment2d::Inside ||
                    LocatePoint(impl_->holes[j].VertexAt(0), impl_->holes[i], eps) ==
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
        return Area(Geometry::kDefaultEpsilon);
    }

    double SCPolygon2d::Area(double eps) const
    {
        if (!IsValid(eps))
        {
            return 0.0;
        }

        // IsValid 接受可由 NormalizePolygon 规范化的环方向，因此面积必须按环角色
        // 计算，不能再依赖外环为正、洞环为负的方向约定。
        double total = std::abs(RingSignedArea(impl_->outerRing));
        for (const auto& hole : impl_->holes)
        {
            total -= std::abs(RingSignedArea(hole));
        }
        return total >= 0.0 ? total : 0.0;
    }

    double SCPolygon2d::Perimeter() const
    {
        return Perimeter(Geometry::kDefaultEpsilon);
    }

    double SCPolygon2d::Perimeter(double eps) const
    {
        if (!IsValid(eps))
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
        return Centroid(Geometry::kDefaultEpsilon);
    }

    SCPoint2d SCPolygon2d::Centroid(double eps) const
    {
        if (!IsValid(eps))
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

        if (!std::isfinite(signedArea) || signedArea == 0.0 ||
            !std::isfinite(weightedX) || !std::isfinite(weightedY))
        {
            return SCPoint2d{};
        }

        return SCPoint2d(weightedX / signedArea, weightedY / signedArea);
    }

    SCBox2d SCPolygon2d::Bounds() const
    {
        return Bounds(Geometry::kDefaultEpsilon);
    }

    SCBox2d SCPolygon2d::Bounds(double eps) const
    {
        if (!IsValid(eps))
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
