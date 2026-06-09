#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Core/Intersection.h"
#include "Core/Metrics.h"
#include "Core/Projection.h"
#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API SCSegmentSearchEntry2d
    {
        std::size_t id{0};
        std::shared_ptr<const ISCSegment2d> segment{};
        SCBox2d box{};

        [[nodiscard]] bool IsValid() const
        {
            return segment != nullptr && segment->IsValid() && box.IsValid();
        }
    };

    struct GEOMETRY_API SCSegmentSearchHit2d
    {
        std::size_t id{0};
        SCPoint2d point{};
        double distanceSquared{0.0};
        double parameter{0.0};
        bool isOnSegment{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && distanceSquared >= 0.0;
        }
    };

    class GEOMETRY_API SCSegmentSearch2d
    {
    public:
        SCSegmentSearch2d() = default;
        explicit SCSegmentSearch2d(std::vector<SCSegmentSearchEntry2d> entries);

        void Clear();
        std::size_t Add(std::shared_ptr<const ISCSegment2d> segment);
        std::size_t Add(const ISCSegment2d& segment);
        std::size_t Add(const SCLineSegment2d& segment);
        std::size_t Add(const SCArcSegment2d& segment);
        bool Remove(std::size_t id);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCSegmentSearchEntry2d* Find(std::size_t id) const;
        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const SCBox2d& box,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const ISCSegment2d& segment,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit2d> QueryWithinDistance(const SCPoint2d& point,
                                                                          double maxDistance) const;
        [[nodiscard]] std::optional<SCSegmentSearchHit2d> Nearest(const SCPoint2d& point) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCSegmentSearchEntry2d>& Entries() const;
        [[nodiscard]] std::vector<SCSegmentSearchEntry2d>& Entries();
        [[nodiscard]] const std::vector<SCSegmentSearchEntry2d>& Data() const;
        [[nodiscard]] std::vector<SCSegmentSearchEntry2d>& Data();

    private:
        std::size_t nextId_{0};
        std::vector<SCSegmentSearchEntry2d> entries_{};
    };

}  // namespace Geometry
