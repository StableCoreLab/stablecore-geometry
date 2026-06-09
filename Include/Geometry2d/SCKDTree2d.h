#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry
{
    struct GEOMETRY_API SCKDTreeEntry2d
    {
        std::size_t id{0};
        SCPoint2d point{};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid();
        }
    };

    struct GEOMETRY_API SCKDTreeHit2d
    {
        std::size_t id{0};
        SCPoint2d point{};
        double distanceSquared{0.0};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && distanceSquared >= 0.0;
        }
    };

    class GEOMETRY_API SCKDTree2d
    {
    public:
        SCKDTree2d() = default;
        explicit SCKDTree2d(std::vector<SCKDTreeEntry2d> entries);

        void Clear();
        void Add(std::size_t id, const SCPoint2d& point);
        bool Remove(std::size_t id);
        void Update(std::size_t id, const SCPoint2d& point);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCKDTreeEntry2d* Find(std::size_t id) const;
        [[nodiscard]] std::vector<std::size_t> Query(const SCPoint2d& point,
                                                     double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryWithinDistance(const SCPoint2d& point, double maxDistance) const;
        [[nodiscard]] std::optional<SCKDTreeHit2d> Nearest(const SCPoint2d& point) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCKDTreeEntry2d>& Entries() const;
        [[nodiscard]] std::vector<SCKDTreeEntry2d>& Entries();
        [[nodiscard]] const std::vector<SCKDTreeEntry2d>& Data() const;
        [[nodiscard]] std::vector<SCKDTreeEntry2d>& Data();

    private:
        std::vector<SCKDTreeEntry2d> entries_{};
    };

}  // namespace Geometry
