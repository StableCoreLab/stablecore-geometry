#pragma once

#include <string>
#include <vector>

#include "Core/Metrics.h"
#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCBoxTreeEntry3d
    {
        std::size_t id{0};
        SCBox3d box{};

        [[nodiscard]] bool IsValid() const
        {
            return box.IsValid();
        }
    };

    struct GEOMETRY_API SCBoxTreeKnnHit3d
    {
        std::size_t id{0};
        SCBox3d box{};
        double distanceSquared{0.0};
    };

    class GEOMETRY_API SCBoxTree3d
    {
    public:
        SCBoxTree3d() = default;
        explicit SCBoxTree3d(std::vector<SCBoxTreeEntry3d> entries);

        void Clear();
        void Add(std::size_t id, const SCBox3d& box);
        bool Remove(std::size_t id);
        void Update(std::size_t id, const SCBox3d& box);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCBoxTreeEntry3d* Find(std::size_t id) const;
        [[nodiscard]] std::vector<std::size_t> Query(const SCBox3d& box, double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryContaining(const SCPoint3d& point,
                                                               double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<SCBoxTreeKnnHit3d> QueryKNearest(const SCPoint3d& point,
                                                                   std::size_t k,
                                                                   double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCBoxTreeEntry3d>& Entries() const;
        [[nodiscard]] std::vector<SCBoxTreeEntry3d>& Entries();

    private:
        std::vector<SCBoxTreeEntry3d> entries_{};
    };
}  // namespace Geometry
