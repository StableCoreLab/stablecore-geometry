#include "Geometry3d/SCBoxTree3d.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <utility>

#include "Core/Metrics.h"

namespace Geometry
{
    namespace
    {
        struct BoxNode
        {
            SCBox3d bounds{};
            std::vector<std::size_t> leafIndices{};
            std::unique_ptr<BoxNode> left{};
            std::unique_ptr<BoxNode> right{};

            [[nodiscard]] bool IsLeaf() const
            {
                return left == nullptr && right == nullptr;
            }
        };

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

        [[nodiscard]] std::unique_ptr<BoxNode> BuildBoxTree(const std::vector<SCBoxTreeEntry3d>& entries,
                                                            std::vector<std::size_t> indices,
                                                            int depth)
        {
            if (indices.empty())
            {
                return {};
            }

            auto node = std::make_unique<BoxNode>();
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

        void QueryBox(const BoxNode* node,
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

        void QueryPoint(const BoxNode* node,
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
    }  // namespace

    SCBoxTree3d::SCBoxTree3d(std::vector<SCBoxTreeEntry3d> entries) : entries_(std::move(entries))
    {
    }

    void SCBoxTree3d::Clear()
    {
        entries_.clear();
    }

    void SCBoxTree3d::Add(std::size_t id, const SCBox3d& box)
    {
        entries_.push_back(SCBoxTreeEntry3d{id, box});
    }

    bool SCBoxTree3d::Remove(std::size_t id)
    {
        const auto it = std::remove_if(
            entries_.begin(), entries_.end(), [id](const SCBoxTreeEntry3d& entry) { return entry.id == id; });
        if (it == entries_.end())
        {
            return false;
        }

        entries_.erase(it, entries_.end());
        return true;
    }

    void SCBoxTree3d::Update(std::size_t id, const SCBox3d& box)
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCBoxTreeEntry3d& entry) { return entry.id == id; });
        if (it != entries_.end())
        {
            it->box = box;
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
        std::vector<std::size_t> indices(entries_.size());
        for (std::size_t i = 0; i < entries_.size(); ++i)
        {
            indices[i] = i;
        }
        const std::unique_ptr<BoxNode> root = BuildBoxTree(entries_, std::move(indices), 0);

        std::vector<std::size_t> result;
        QueryBox(root.get(), entries_, box, eps, result);
        return result;
    }

    std::vector<std::size_t> SCBoxTree3d::QueryContaining(const SCPoint3d& point, double eps) const
    {
        std::vector<std::size_t> indices(entries_.size());
        for (std::size_t i = 0; i < entries_.size(); ++i)
        {
            indices[i] = i;
        }
        const std::unique_ptr<BoxNode> root = BuildBoxTree(entries_, std::move(indices), 0);

        std::vector<std::size_t> result;
        QueryPoint(root.get(), entries_, point, eps, result);
        return result;
    }

    std::vector<SCBoxTreeKnnHit3d> SCBoxTree3d::QueryKNearest(const SCPoint3d& point, std::size_t k, double) const
    {
        const auto compareHits = [](const auto& lhs, const auto& rhs) {
            if (lhs.distanceSquared != rhs.distanceSquared)
            {
                return lhs.distanceSquared < rhs.distanceSquared;
            }
            return lhs.id < rhs.id;
        };

        std::vector<SCBoxTreeKnnHit3d> hits;
        hits.reserve(entries_.size());
        for (const auto& entry : entries_)
        {
            hits.push_back(SCBoxTreeKnnHit3d{entry.id, entry.box, DistanceSquared(point, entry.box)});
        }

        if (k < hits.size())
        {
            std::nth_element(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(k), hits.end(), compareHits);
            hits.resize(k);
        }

        std::sort(hits.begin(), hits.end(), compareHits);
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

    std::vector<SCBoxTreeEntry3d>& SCBoxTree3d::Entries()
    {
        return entries_;
    }
}  // namespace Geometry
