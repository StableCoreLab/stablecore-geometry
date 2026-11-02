#include "Geometry3d/SCBoxTree3d.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <memory>
#include <sstream>
#include <utility>

#include "Core/Metrics.h"
#include "../Detail/BoxTreeCore.h"
#include "../Detail/GeometryContainerOps.h"

namespace Geometry
{
    struct SCBoxTree3d::Node
    {
        SCBox3d bounds{};
        std::vector<std::size_t> leafIndices{};
        std::unique_ptr<Node> left{};
        std::unique_ptr<Node> right{};

        [[nodiscard]] bool IsLeaf() const
        {
            return left == nullptr && right == nullptr;
        }
    };

    namespace
    {
        struct BoxTree3dTraits
        {
            static constexpr int AxisCount = 3;
            static SCBox3d EmptyBox() { return {}; }
            static void ExpandBounds(SCBox3d& bounds, const SCBox3d& box)
            {
                bounds.ExpandToInclude(box.MinPoint());
                bounds.ExpandToInclude(box.MaxPoint());
            }
            static double CenterCoordinate(const SCBox3d& box, int axis)
            {
                const SCPoint3d center = box.Center();
                return axis == 0 ? center.x : (axis == 1 ? center.y : center.z);
            }
            static double DistanceSquared(const SCPoint3d& point, const SCBox3d& box)
            {
                return Geometry::DistanceSquared(point, box);
            }
            static double Distance(const SCPoint3d& point, const SCBox3d& box)
            {
                const double dx = std::max({box.MinPoint().x - point.x, 0.0, point.x - box.MaxPoint().x});
                const double dy = std::max({box.MinPoint().y - point.y, 0.0, point.y - box.MaxPoint().y});
                const double dz = std::max({box.MinPoint().z - point.z, 0.0, point.z - box.MaxPoint().z});
                return std::hypot(dx, dy, dz);
            }
            static double QuarterDistance(const SCPoint3d& point, const SCBox3d& box)
            {
                const double dx = std::max({box.MinPoint().x * 0.25 - point.x * 0.25,
                                            0.0, point.x * 0.25 - box.MaxPoint().x * 0.25});
                const double dy = std::max({box.MinPoint().y * 0.25 - point.y * 0.25,
                                            0.0, point.y * 0.25 - box.MaxPoint().y * 0.25});
                const double dz = std::max({box.MinPoint().z * 0.25 - point.z * 0.25,
                                            0.0, point.z * 0.25 - box.MaxPoint().z * 0.25});
                return std::hypot(dx, dy, dz);
            }
            static bool Intersects(const SCBox3d& lhs, const SCBox3d& rhs, double eps)
            {
                return Geometry::Intersects(lhs, rhs, eps);
            }
            static bool Contains(const SCBox3d& box, const SCPoint3d& point, double eps)
            {
                return Geometry::Contains(box, point, eps);
            }
        };

        [[nodiscard]] std::unique_ptr<SCBoxTree3d::Node> BuildBoxTree(const std::vector<SCBoxTreeEntry3d>& entries,
                                                                      std::vector<std::size_t> indices,
                                                                      int depth)
        {
            return Detail::BuildBoxTree<SCBoxTree3d::Node, SCBoxTreeEntry3d, BoxTree3dTraits>(
                entries, std::move(indices), depth);
        }

        [[nodiscard]] int InsertIndex(std::unique_ptr<SCBoxTree3d::Node>& node,
                                      const std::vector<SCBoxTreeEntry3d>& entries,
                                      std::size_t index,
                                      int depth)
        {
            return Detail::InsertBoxTreeIndex<SCBoxTree3d::Node, SCBoxTreeEntry3d, BoxTree3dTraits>(
                node, entries, index, depth);
        }

        [[nodiscard]] bool FindIndexPath(std::unique_ptr<SCBoxTree3d::Node>& node,
                                         std::size_t index,
                                         const SCBox3d& box,
                                         std::vector<std::unique_ptr<SCBoxTree3d::Node>*>& path)
        {
            return Detail::FindBoxTreeIndexPath<SCBoxTree3d::Node, SCBox3d, BoxTree3dTraits>(node, index, box, path);
        }

        void RefreshPath(const std::vector<std::unique_ptr<SCBoxTree3d::Node>*>& path,
                         const std::vector<SCBoxTreeEntry3d>& entries,
                         bool prune)
        {
            Detail::RefreshBoxTreePath<SCBoxTree3d::Node, SCBoxTreeEntry3d, BoxTree3dTraits>(path, entries, prune);
        }

        void RemapIndices(std::unique_ptr<SCBoxTree3d::Node>& node,
                          const std::vector<SCBoxTreeEntry3d>& entries,
                          const std::vector<std::size_t>& mapping)
        {
            Detail::RemapBoxTreeIndices<SCBoxTree3d::Node, SCBoxTreeEntry3d, BoxTree3dTraits>(node, entries, mapping);
        }

    }  // namespace

    SCBoxTree3d::SCBoxTree3d() = default;

    SCBoxTree3d::SCBoxTree3d(std::vector<SCBoxTreeEntry3d> entries) : entries_(std::move(entries))
    {
        Rebuild();
    }

    SCBoxTree3d::~SCBoxTree3d() = default;
    SCBoxTree3d::SCBoxTree3d(SCBoxTree3d&&) noexcept = default;
    SCBoxTree3d& SCBoxTree3d::operator=(SCBoxTree3d&&) noexcept = default;

    SCBoxTree3d::SCBoxTree3d(const SCBoxTree3d& other) : entries_(other.entries_)
    {
        Rebuild();
    }

    SCBoxTree3d& SCBoxTree3d::operator=(const SCBoxTree3d& other)
    {
        if (this != &other)
        {
            SCBoxTree3d copy(other);
            entries_.swap(copy.entries_);
            root_.swap(copy.root_);
        }
        return *this;
    }

    void SCBoxTree3d::Rebuild()
    {
        std::vector<std::size_t> indices;
        indices.reserve(entries_.size());
        for (std::size_t index = 0; index < entries_.size(); ++index)
        {
            if (entries_[index].IsValid())
            {
                indices.push_back(index);
            }
        }
        auto newRoot = BuildBoxTree(entries_, std::move(indices), 0);
        root_ = std::move(newRoot);
    }

    void SCBoxTree3d::Clear()
    {
        entries_.clear();
        root_.reset();
    }

    void SCBoxTree3d::Add(std::size_t id, const SCBox3d& box)
    {
        entries_.push_back(SCBoxTreeEntry3d{id, box});
        if (!entries_.back().IsValid())
        {
            return;
        }
        int depth;
        try
        {
            depth = InsertIndex(root_, entries_, entries_.size() - 1, 0);
        }
        catch (...)
        {
            entries_.pop_back();
            throw;
        }
        if (depth > static_cast<int>(2 * std::bit_width(entries_.size()) + 8))
        {
            Rebuild();
        }
    }

    bool SCBoxTree3d::Remove(std::size_t id)
    {
        std::vector<std::size_t> mapping(entries_.size());
        std::size_t retained = 0;
        for (std::size_t index = 0; index < entries_.size(); ++index)
        {
            mapping[index] = entries_[index].id == id ? entries_.size() : retained++;
        }
        if (retained == entries_.size())
        {
            return false;
        }
        entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                      [id](const SCBoxTreeEntry3d& entry) { return entry.id == id; }),
                       entries_.end());
        RemapIndices(root_, entries_, mapping);
        return true;
    }

    void SCBoxTree3d::Update(std::size_t id, const SCBox3d& box)
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCBoxTreeEntry3d& entry) { return entry.id == id; });
        if (it != entries_.end())
        {
            const std::size_t index = static_cast<std::size_t>(it - entries_.begin());
            const SCBox3d oldBox = it->box;
            std::vector<std::unique_ptr<Node>*> path;
            if (oldBox.IsValid())
            {
                (void)FindIndexPath(root_, index, oldBox, path);
                auto& indices = path.back()->get()->leafIndices;
                indices.erase(std::remove(indices.begin(), indices.end(), index), indices.end());
                RefreshPath(path, entries_, false);
            }
            it->box = box;
            if (box.IsValid())
            {
                int depth;
                try
                {
                    depth = InsertIndex(root_, entries_, index, 0);
                }
                catch (...)
                {
                    it->box = oldBox;
                    if (!path.empty())
                    {
                        path.back()->get()->leafIndices.push_back(index);
                        RefreshPath(path, entries_, false);
                    }
                    throw;
                }
                if (!path.empty())
                {
                    RefreshPath(path, entries_, true);
                }
                if (depth > static_cast<int>(2 * std::bit_width(entries_.size()) + 8))
                {
                    Rebuild();
                }
            }
            else if (!path.empty())
            {
                RefreshPath(path, entries_, true);
            }
            return;
        }

        Add(id, box);
    }

    std::size_t SCBoxTree3d::Size() const
    {
        return Detail::ContainerCount(entries_);
    }

    bool SCBoxTree3d::IsEmpty() const
    {
        return Detail::ContainerIsEmpty(entries_);
    }

    bool SCBoxTree3d::IsValid() const
    {
        return Detail::ContainerIsValid(entries_);
    }

    bool SCBoxTree3d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCBoxTreeEntry3d* SCBoxTree3d::Find(std::size_t id) const
    {
        return Detail::FindEntryById(entries_, id);
    }

    std::vector<std::size_t> SCBoxTree3d::Query(const SCBox3d& box, double eps) const
    {
        std::vector<std::size_t> result;
        Detail::QueryBoxTree<Node, SCBoxTreeEntry3d, SCBox3d, BoxTree3dTraits>(root_.get(), entries_, box, eps, result);
        return result;
    }

    std::vector<std::size_t> SCBoxTree3d::QueryContaining(const SCPoint3d& point, double eps) const
    {
        std::vector<std::size_t> result;
        if (!point.IsValid())
        {
            return result;
        }
        Detail::QueryPointTree<Node, SCBoxTreeEntry3d, SCPoint3d, BoxTree3dTraits>(root_.get(), entries_, point, eps, result);
        return result;
    }

    std::vector<SCBoxTreeKnnHit3d> SCBoxTree3d::QueryKNearest(const SCPoint3d& point, std::size_t k, double eps) const
    {
        std::vector<SCBoxTreeKnnHit3d> hits;
        if (!point.IsValid())
        {
            return hits;
        }
        hits.reserve(std::min(k, entries_.size()));
        Detail::QueryBoxTreeKnn<Node, SCBoxTreeEntry3d, SCPoint3d, SCBoxTreeKnnHit3d, BoxTree3dTraits>(
            root_.get(), entries_, point, k, eps, hits);
        std::sort(hits.begin(), hits.end(), [&point](const auto& lhs, const auto& rhs) {
            return Detail::CompareBoxTreeHits<SCBoxTreeKnnHit3d, SCPoint3d, BoxTree3dTraits>(lhs, rhs, point);
        });
        return hits;
    }

    std::string SCBoxTree3d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCBoxTree3d{size=" << Size() << ", valid=" << (IsValid() ? "true" : "false") << "}";
        return stream.str();
    }

    const std::vector<SCBoxTreeEntry3d>& SCBoxTree3d::Entries() const
    {
        return entries_;
    }

}  // namespace Geometry
