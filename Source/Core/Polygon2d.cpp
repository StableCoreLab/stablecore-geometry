#include "Geometry2d/SCPolygon2d.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "../Detail/RingIntegral2d.h"
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

        [[nodiscard]] double RingSignedArea(const SCPolyline2d& ring, double eps)
        {
            return Detail::ComputeSignedArea(ring, eps);
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
        if (!impl_->outerRing.IsValid(eps) || !impl_->outerRing.IsClosed())
        {
            return false;
        }

        if (HasSelfIntersection(impl_->outerRing, eps))
        {
            return false;
        }

        const double outerArea = RingSignedArea(impl_->outerRing, eps);
        // 环方向不是结构有效性的前置条件；NormalizePolygon 会在需要方向语义的
        // 算法入口统一规范化。这里排除零面积或面积不可表示的环。
        if (!std::isfinite(outerArea) || outerArea == 0.0)
        {
            return false;
        }

        double totalPerimeter = impl_->outerRing.Length(eps);
        for (const auto& hole : impl_->holes)
        {
            if (!hole.IsValid(eps) || !hole.IsClosed())
            {
                return false;
            }
            totalPerimeter += hole.Length(eps);

            if (HasSelfIntersection(hole, eps))
            {
                return false;
            }

            const double holeArea = RingSignedArea(hole, eps);
            if (!std::isfinite(holeArea) || holeArea == 0.0)
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

        return std::isfinite(totalPerimeter);
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
        double total = std::abs(RingSignedArea(impl_->outerRing, eps));
        for (const auto& hole : impl_->holes)
        {
            total -= std::abs(RingSignedArea(hole, eps));
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

        double total = impl_->outerRing.Length(eps);
        for (const auto& hole : impl_->holes)
        {
            total += hole.Length(eps);
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

        const Detail::RingMoment2d outer = Detail::ComputeRingMoment(impl_->outerRing);
        const double outerSign = outer.signedArea > 0.0 ? 1.0 : -1.0;
        double weightedX = outerSign * outer.firstMomentX;
        double weightedY = outerSign * outer.firstMomentY;
        double area = std::abs(outer.signedArea);

        // 与 Area 一样按外环／孔角色加权；各孔矩平移到外环基点后再相减。
        for (const auto& hole : impl_->holes)
        {
            const Detail::RingMoment2d ring = Detail::ComputeRingMoment(hole);
            const double holeArea = std::abs(ring.signedArea);
            const double holeSign = ring.signedArea > 0.0 ? 1.0 : -1.0;
            weightedX -= holeSign * ring.firstMomentX +
                         (ring.referencePoint.x - outer.referencePoint.x) * holeArea;
            weightedY -= holeSign * ring.firstMomentY +
                         (ring.referencePoint.y - outer.referencePoint.y) * holeArea;
            area -= holeArea;
        }

        if (!std::isfinite(area) || area <= 0.0 ||
            !std::isfinite(weightedX) || !std::isfinite(weightedY))
        {
            return SCPoint2d{};
        }

        return SCPoint2d(outer.referencePoint.x + weightedX / area,
                         outer.referencePoint.y + weightedY / area);
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

        SCBox2d box = impl_->outerRing.Bounds(eps);
        for (const auto& hole : impl_->holes)
        {
            box.ExpandToInclude(hole.Bounds(eps));
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
