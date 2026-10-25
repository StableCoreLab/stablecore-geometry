#include "Geometry2d/SCBoxTree2d.h"

#include <algorithm>
#include <bit>
#include <memory>
#include <sstream>
#include <utility>

#include "Core/Metrics.h"
#include "../Detail/BoxTreeCore.h"
#include "../Detail/GeometryContainerOps.h"

namespace Geometry
{
    struct SCBoxTree2d::Node
    {
        SCBox2d bounds{};
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
        struct BoxTree2dTraits
        {
            static constexpr int AxisCount = 2;
            static SCBox2d EmptyBox() { return {}; }
            static void ExpandBounds(SCBox2d& bounds, const SCBox2d& box) { bounds.ExpandToInclude(box); }
            static double CenterCoordinate(const SCBox2d& box, int axis)
            {
                const SCPoint2d center = box.Center();
                return axis == 0 ? center.x : center.y;
            }
            static double DistanceSquared(const SCPoint2d& point, const SCBox2d& box)
            {
                return Geometry::DistanceSquared(point, box);
            }
            static bool Intersects(const SCBox2d& lhs, const SCBox2d& rhs, double eps)
            {
                return Geometry::Intersects(lhs, rhs, eps);
            }
            static bool Contains(const SCBox2d& box, const SCPoint2d& point, double eps)
            {
                return Geometry::Contains(box, point, eps);
            }
        };

        [[nodiscard]] std::unique_ptr<SCBoxTree2d::Node> BuildBoxTree(const std::vector<SCBoxTreeEntry2d>& entries,
                                                                      std::vector<std::size_t> indices,
                                                                      int depth)
        {
            return Detail::BuildBoxTree<SCBoxTree2d::Node, SCBoxTreeEntry2d, BoxTree2dTraits>(
                entries, std::move(indices), depth);
        }

        [[nodiscard]] int InsertIndex(std::unique_ptr<SCBoxTree2d::Node>& node,
                                      const std::vector<SCBoxTreeEntry2d>& entries,
                                      std::size_t index,
                                      int depth)
        {
            return Detail::InsertBoxTreeIndex<SCBoxTree2d::Node, SCBoxTreeEntry2d, BoxTree2dTraits>(
                node, entries, index, depth);
        }

        [[nodiscard]] bool FindIndexPath(std::unique_ptr<SCBoxTree2d::Node>& node,
                                         std::size_t index,
                                         const SCBox2d& box,
                                         std::vector<std::unique_ptr<SCBoxTree2d::Node>*>& path)
        {
            return Detail::FindBoxTreeIndexPath<SCBoxTree2d::Node, SCBox2d, BoxTree2dTraits>(node, index, box, path);
        }

        void RefreshPath(const std::vector<std::unique_ptr<SCBoxTree2d::Node>*>& path,
                         const std::vector<SCBoxTreeEntry2d>& entries,
                         bool prune)
        {
            Detail::RefreshBoxTreePath<SCBoxTree2d::Node, SCBoxTreeEntry2d, BoxTree2dTraits>(path, entries, prune);
        }

        void RemapIndices(std::unique_ptr<SCBoxTree2d::Node>& node,
                          const std::vector<SCBoxTreeEntry2d>& entries,
                          const std::vector<std::size_t>& mapping)
        {
            Detail::RemapBoxTreeIndices<SCBoxTree2d::Node, SCBoxTreeEntry2d, BoxTree2dTraits>(node, entries, mapping);
        }

    }  // namespace

    SCBoxTree2d::SCBoxTree2d() = default;

    SCBoxTree2d::SCBoxTree2d(std::vector<SCBoxTreeEntry2d> entries) : entries_(std::move(entries))
    {
        Rebuild();
    }

    SCBoxTree2d::~SCBoxTree2d() = default;
    SCBoxTree2d::SCBoxTree2d(SCBoxTree2d&&) noexcept = default;
    SCBoxTree2d& SCBoxTree2d::operator=(SCBoxTree2d&&) noexcept = default;

    SCBoxTree2d::SCBoxTree2d(const SCBoxTree2d& other) : entries_(other.entries_)
    {
        Rebuild();
    }

    SCBoxTree2d& SCBoxTree2d::operator=(const SCBoxTree2d& other)
    {
        if (this != &other)
        {
            SCBoxTree2d copy(other);
            entries_.swap(copy.entries_);
            root_.swap(copy.root_);
        }
        return *this;
    }

    void SCBoxTree2d::Rebuild()
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

    void SCBoxTree2d::Clear()
    {
        entries_.clear();
        root_.reset();
    }

    void SCBoxTree2d::Add(std::size_t id, const SCBox2d& box)
    {
        entries_.push_back(SCBoxTreeEntry2d{id, box});
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

    bool SCBoxTree2d::Remove(std::size_t id)
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
                                      [id](const SCBoxTreeEntry2d& entry) { return entry.id == id; }),
                       entries_.end());
        RemapIndices(root_, entries_, mapping);
        return true;
    }

    void SCBoxTree2d::Update(std::size_t id, const SCBox2d& box)
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCBoxTreeEntry2d& entry) { return entry.id == id; });
        if (it != entries_.end())
        {
            const std::size_t index = static_cast<std::size_t>(it - entries_.begin());
            const SCBox2d oldBox = it->box;
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

    std::size_t SCBoxTree2d::Size() const
    {
        return Detail::ContainerCount(entries_);
    }

    bool SCBoxTree2d::IsEmpty() const
    {
        return Detail::ContainerIsEmpty(entries_);
    }

    bool SCBoxTree2d::IsValid() const
    {
        return Detail::ContainerIsValid(entries_);
    }

    bool SCBoxTree2d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCBoxTreeEntry2d* SCBoxTree2d::Find(std::size_t id) const
    {
        return Detail::FindEntryById(entries_, id);
    }

    std::vector<std::size_t> SCBoxTree2d::Query(const SCBox2d& box, double eps) const
    {
        std::vector<std::size_t> result;
        Detail::QueryBoxTree<Node, SCBoxTreeEntry2d, SCBox2d, BoxTree2dTraits>(root_.get(), entries_, box, eps, result);
        return result;
    }

    std::vector<std::size_t> SCBoxTree2d::QueryContaining(const SCPoint2d& point, double eps) const
    {
        std::vector<std::size_t> result;
        Detail::QueryPointTree<Node, SCBoxTreeEntry2d, SCPoint2d, BoxTree2dTraits>(root_.get(), entries_, point, eps, result);
        return result;
    }

    std::vector<SCBoxTreeKnnHit2d> SCBoxTree2d::QueryKNearest(const SCPoint2d& point, std::size_t k, double eps) const
    {
        std::vector<SCBoxTreeKnnHit2d> hits;
        hits.reserve(std::min(k, entries_.size()));
        Detail::QueryBoxTreeKnn<Node, SCBoxTreeEntry2d, SCPoint2d, SCBoxTreeKnnHit2d, BoxTree2dTraits>(
            root_.get(), entries_, point, k, eps, hits);
        std::sort(hits.begin(), hits.end(), Detail::CompareBoxTreeHits<SCBoxTreeKnnHit2d>);
        return hits;
    }

    std::string SCBoxTree2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCBoxTree2d{size=" << Size() << ", valid=" << (IsValid() ? "true" : "false") << "}";
        return stream.str();
    }

    const std::vector<SCBoxTreeEntry2d>& SCBoxTree2d::Entries() const
    {
        return entries_;
    }

    const std::vector<SCBoxTreeEntry2d>& SCBoxTree2d::Data() const
    {
        return Entries();
    }

}  // namespace Geometry
