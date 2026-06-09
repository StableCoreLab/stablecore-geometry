#pragma once
#include <string>
#include <vector>

#include "Core/Metrics.h"
#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API SCBoxTreeEntry2d
    {
        std::size_t id{0};
        SCBox2d box{};

        [[nodiscard]] bool IsValid() const
        {
            return box.IsValid();
        }
    };

    struct GEOMETRY_API SCBoxTreeHit2d
    {
        std::size_t id{0};
        SCBox2d box{};
    };

    class GEOMETRY_API SCBoxTree2d
    {
    public:
        SCBoxTree2d() = default;
        explicit SCBoxTree2d(std::vector<SCBoxTreeEntry2d> entries);

        void Clear();
        void Add(std::size_t id, const SCBox2d& box);
        bool Remove(std::size_t id);
        void Update(std::size_t id, const SCBox2d& box);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCBoxTreeEntry2d* Find(std::size_t id) const;
        [[nodiscard]] std::vector<std::size_t> Query(const SCBox2d& box, double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryContaining(const SCPoint2d& point,
                                                               double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCBoxTreeEntry2d>& Entries() const;
        [[nodiscard]] std::vector<SCBoxTreeEntry2d>& Entries();
        [[nodiscard]] const std::vector<SCBoxTreeEntry2d>& Data() const;
        [[nodiscard]] std::vector<SCBoxTreeEntry2d>& Data();

    private:
        std::vector<SCBoxTreeEntry2d> entries_{};
    };

}  // namespace Geometry
