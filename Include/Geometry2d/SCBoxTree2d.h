#pragma once
#include <memory>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Support/Epsilon.h"
#include "Types/Geometry2d/SCBox2.h"

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

    struct GEOMETRY_API SCBoxTreeKnnHit2d
    {
        std::size_t id{0};
        SCBox2d box{};
        double distanceSquared{0.0};
    };

    class GEOMETRY_API SCBoxTree2d
    {
    public:
        struct Node;
        SCBoxTree2d();
        explicit SCBoxTree2d(std::vector<SCBoxTreeEntry2d> entries);
        ~SCBoxTree2d();
        SCBoxTree2d(const SCBoxTree2d& other);
        SCBoxTree2d& operator=(const SCBoxTree2d& other);
        SCBoxTree2d(SCBoxTree2d&&) noexcept;
        SCBoxTree2d& operator=(SCBoxTree2d&&) noexcept;

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
        [[nodiscard]] std::vector<SCBoxTreeKnnHit2d> QueryKNearest(const SCPoint2d& point,
                                                                   std::size_t k,
                                                                   double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCBoxTreeEntry2d>& Entries() const;
        [[nodiscard]] const std::vector<SCBoxTreeEntry2d>& Data() const;

    private:
        void Rebuild();
        std::vector<SCBoxTreeEntry2d> entries_{};
        std::unique_ptr<Node> root_{};
    };

}  // namespace Geometry
