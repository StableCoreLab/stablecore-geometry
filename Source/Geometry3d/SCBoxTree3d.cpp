#include "Geometry3d/SCBoxTree3d.h"

#include <algorithm>
#include <bit>
#include <memory>
#include <sstream>
#include <utility>

#include "Core/Metrics.h"

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
        [[nodiscard]] double CenterCoordinate(const SCBox3d& box, int axis)
        {
            const SCPoint3d center = box.Center();
            return axis == 0 ? center.x : (axis == 1 ? center.y : center.z);
        }

        [[nodiscard]] SCBox3d ComputeBounds(const std::vector<SCBoxTreeEntry3d>& entries,
                                            const std::vector<std::size_t>& indices)
        {
            SCBox3d bounds;
            for (std::size_t index : indices)
            {
                bounds.ExpandToInclude(entries[index].box.MinPoint());
                bounds.ExpandToInclude(entries[index].box.MaxPoint());
            }
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<SCBoxTree3d::Node> BuildBoxTree(const std::vector<SCBoxTreeEntry3d>& entries,
                                                            std::vector<std::size_t> indices,
                                                            int depth)
        {
            if (indices.empty())
            {
                return {};
            }

            auto node = std::make_unique<SCBoxTree3d::Node>();
            node->bounds = ComputeBounds(entries, indices);
            if (indices.size() <= 4)
            {
                node->leafIndices = std::move(indices);
                return node;
            }

            const int axis = depth % 3;
            std::sort(indices.begin(), indices.end(), [&entries, axis](std::size_t lhs, std::size_t rhs) {
                return CenterCoordinate(entries[lhs].box, axis) < CenterCoordinate(entries[rhs].box, axis);
            });

            const std::size_t mid = indices.size() / 2;
            std::vector<std::size_t> leftIndices(indices.begin(), indices.begin() + static_cast<std::ptrdiff_t>(mid));
            std::vector<std::size_t> rightIndices(indices.begin() + static_cast<std::ptrdiff_t>(mid), indices.end());
            node->left = BuildBoxTree(entries, std::move(leftIndices), depth + 1);
            node->right = BuildBoxTree(entries, std::move(rightIndices), depth + 1);
            return node;
        }

        void ExpandBounds(SCBox3d& bounds, const SCBox3d& box)
        {
            bounds.ExpandToInclude(box.MinPoint());
            bounds.ExpandToInclude(box.MaxPoint());
        }

        void RefreshBounds(SCBoxTree3d::Node& node, const std::vector<SCBoxTreeEntry3d>& entries)
        {
            node.bounds = {};
            if (node.IsLeaf())
            {
                node.bounds = ComputeBounds(entries, node.leafIndices);
                return;
            }
            if (node.left && node.left->bounds.IsValid())
            {
                ExpandBounds(node.bounds, node.left->bounds);
            }
            if (node.right && node.right->bounds.IsValid())
            {
                ExpandBounds(node.bounds, node.right->bounds);
            }
        }

        [[nodiscard]] int InsertIndex(std::unique_ptr<SCBoxTree3d::Node>& node,
                                      const std::vector<SCBoxTreeEntry3d>& entries,
                                      std::size_t index,
                                      int depth)
        {
            if (!node)
            {
                auto newNode = std::make_unique<SCBoxTree3d::Node>();
                newNode->leafIndices.push_back(index);
                RefreshBounds(*newNode, entries);
                node = std::move(newNode);
                return depth;
            }
            int insertedDepth = depth;
            if (node->IsLeaf())
            {
                if (node->leafIndices.size() == 4)
                {
                    auto indices = node->leafIndices;
                    indices.push_back(index);
                    auto replacement = BuildBoxTree(entries, std::move(indices), depth);
                    node = std::move(replacement);
                    insertedDepth = depth + 1;
                }
                else
                {
                    node->leafIndices.push_back(index);
                }
            }
            else
            {
                const double leftDistance = DistanceSquared(entries[index].box.Center(), node->left->bounds);
                const double rightDistance = DistanceSquared(entries[index].box.Center(), node->right->bounds);
                insertedDepth = InsertIndex(leftDistance <= rightDistance ? node->left : node->right,
                                            entries, index, depth + 1);
            }
            RefreshBounds(*node, entries);
            return insertedDepth;
        }

        [[nodiscard]] bool FindIndexPath(std::unique_ptr<SCBoxTree3d::Node>& node,
                                         std::size_t index,
                                         const SCBox3d& box,
                                         std::vector<std::unique_ptr<SCBoxTree3d::Node>*>& path)
        {
            if (!node || !Intersects(node->bounds, box, 0.0))
            {
                return false;
            }
            path.push_back(&node);
            if (node->IsLeaf())
            {
                if (std::find(node->leafIndices.begin(), node->leafIndices.end(), index) != node->leafIndices.end())
                {
                    return true;
                }
            }
            else if (FindIndexPath(node->left, index, box, path) || FindIndexPath(node->right, index, box, path))
            {
                return true;
            }
            path.pop_back();
            return false;
        }

        void RefreshPath(const std::vector<std::unique_ptr<SCBoxTree3d::Node>*>& path,
                         const std::vector<SCBoxTreeEntry3d>& entries,
                         bool prune)
        {
            for (auto it = path.rbegin(); it != path.rend(); ++it)
            {
                auto& node = **it;
                if (prune && node->IsLeaf() && node->leafIndices.empty())
                {
                    node.reset();
                }
                else if (prune && !node->IsLeaf() && (!node->left || !node->right))
                {
                    node = std::move(node->left ? node->left : node->right);
                }
                if (node)
                {
                    RefreshBounds(*node, entries);
                }
            }
        }

        void RemapIndices(std::unique_ptr<SCBoxTree3d::Node>& node,
                          const std::vector<SCBoxTreeEntry3d>& entries,
                          const std::vector<std::size_t>& mapping)
        {
            if (!node)
            {
                return;
            }
            if (node->IsLeaf())
            {
                auto& indices = node->leafIndices;
                std::size_t retained = 0;
                for (std::size_t index : indices)
                {
                    if (mapping[index] != mapping.size())
                    {
                        indices[retained++] = mapping[index];
                    }
                }
                indices.resize(retained);
                if (indices.empty())
                {
                    node.reset();
                    return;
                }
            }
            else
            {
                RemapIndices(node->left, entries, mapping);
                RemapIndices(node->right, entries, mapping);
                if (!node->left || !node->right)
                {
                    node = std::move(node->left ? node->left : node->right);
                }
            }
            if (node)
            {
                RefreshBounds(*node, entries);
            }
        }

        void QueryBox(const SCBoxTree3d::Node* node,
                      const std::vector<SCBoxTreeEntry3d>& entries,
                      const SCBox3d& box,
                      double eps,
                      std::vector<std::size_t>& result)
        {
            if (node == nullptr || !Intersects(node->bounds, box, eps))
            {
                return;
            }

            if (node->IsLeaf())
            {
                for (std::size_t index : node->leafIndices)
                {
                    if (Intersects(entries[index].box, box, eps))
                    {
                        result.push_back(entries[index].id);
                    }
                }
                return;
            }

            QueryBox(node->left.get(), entries, box, eps, result);
            QueryBox(node->right.get(), entries, box, eps, result);
        }

        void QueryPoint(const SCBoxTree3d::Node* node,
                        const std::vector<SCBoxTreeEntry3d>& entries,
                        const SCPoint3d& point,
                        double eps,
                        std::vector<std::size_t>& result)
        {
            if (node == nullptr || !Contains(node->bounds, point, eps))
            {
                return;
            }

            if (node->IsLeaf())
            {
                for (std::size_t index : node->leafIndices)
                {
                    if (Contains(entries[index].box, point, eps))
                    {
                        result.push_back(entries[index].id);
                    }
                }
                return;
            }

            QueryPoint(node->left.get(), entries, point, eps, result);
            QueryPoint(node->right.get(), entries, point, eps, result);
        }

        [[nodiscard]] bool CompareHits(const SCBoxTreeKnnHit3d& lhs, const SCBoxTreeKnnHit3d& rhs)
        {
            return lhs.distanceSquared == rhs.distanceSquared ? lhs.id < rhs.id
                                                                : lhs.distanceSquared < rhs.distanceSquared;
        }

        void QueryKnn(const SCBoxTree3d::Node* node,
                      const std::vector<SCBoxTreeEntry3d>& entries,
                      const SCPoint3d& point,
                      std::size_t k,
                      double eps,
                      std::vector<SCBoxTreeKnnHit3d>& hits)
        {
            if (node == nullptr || k == 0)
            {
                return;
            }
            if (hits.size() >= k)
            {
                const double factor = 1.0 + std::max(0.0, eps);
                const double lowerBound = DistanceSquared(point, node->bounds);
                if (lowerBound != hits.front().distanceSquared &&
                    lowerBound * factor * factor > hits.front().distanceSquared)
                {
                    return;
                }
            }
            if (node->IsLeaf())
            {
                for (std::size_t index : node->leafIndices)
                {
                    const auto& entry = entries[index];
                    SCBoxTreeKnnHit3d hit{entry.id, entry.box, DistanceSquared(point, entry.box)};
                    if (hits.size() < k)
                    {
                        hits.push_back(hit);
                        std::push_heap(hits.begin(), hits.end(), CompareHits);
                    }
                    else if (CompareHits(hit, hits.front()))
                    {
                        std::pop_heap(hits.begin(), hits.end(), CompareHits);
                        hits.back() = hit;
                        std::push_heap(hits.begin(), hits.end(), CompareHits);
                    }
                }
                return;
            }
            const auto* first = node->left.get();
            const auto* second = node->right.get();
            if (DistanceSquared(point, second->bounds) < DistanceSquared(point, first->bounds))
            {
                std::swap(first, second);
            }
            QueryKnn(first, entries, point, k, eps, hits);
            QueryKnn(second, entries, point, k, eps, hits);
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
        return entries_.size();
    }

    bool SCBoxTree3d::IsEmpty() const
    {
        return entries_.empty();
    }

    bool SCBoxTree3d::IsValid() const
    {
        for (const auto& entry : entries_)
        {
            if (!entry.IsValid())
            {
                return false;
            }
        }
        return true;
    }

    bool SCBoxTree3d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCBoxTreeEntry3d* SCBoxTree3d::Find(std::size_t id) const
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCBoxTreeEntry3d& entry) { return entry.id == id; });
        return it == entries_.end() ? nullptr : &*it;
    }

    std::vector<std::size_t> SCBoxTree3d::Query(const SCBox3d& box, double eps) const
    {
        std::vector<std::size_t> result;
        QueryBox(root_.get(), entries_, box, eps, result);
        return result;
    }

    std::vector<std::size_t> SCBoxTree3d::QueryContaining(const SCPoint3d& point, double eps) const
    {
        std::vector<std::size_t> result;
        QueryPoint(root_.get(), entries_, point, eps, result);
        return result;
    }

    std::vector<SCBoxTreeKnnHit3d> SCBoxTree3d::QueryKNearest(const SCPoint3d& point, std::size_t k, double eps) const
    {
        std::vector<SCBoxTreeKnnHit3d> hits;
        hits.reserve(std::min(k, entries_.size()));
        QueryKnn(root_.get(), entries_, point, k, eps, hits);
        std::sort(hits.begin(), hits.end(), CompareHits);
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
