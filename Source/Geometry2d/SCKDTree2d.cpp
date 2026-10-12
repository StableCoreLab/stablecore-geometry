#include "Geometry2d/SCKDTree2d.h"

#include <algorithm>
#include <bit>
#include <memory>
#include <sstream>
#include <utility>

#include "Core/Metrics.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    struct SCKDTree2d::Node
    {
        std::size_t entryIndex{0};
        SCPoint2d splitPoint{};
        bool active{true};
        int axis{0};
        std::unique_ptr<Node> left{};
        std::unique_ptr<Node> right{};
    };

    namespace
    {
        [[nodiscard]] double Coordinate(const SCPoint2d& point, int axis)
        {
            return axis == 0 ? point.x : point.y;
        }

        [[nodiscard]] std::unique_ptr<SCKDTree2d::Node> BuildKDTree(const std::vector<SCKDTreeEntry2d>& entries,
                                                          std::vector<std::size_t>::iterator begin,
                                                          std::vector<std::size_t>::iterator end,
                                                          int depth)
        {
            if (begin >= end)
            {
                return {};
            }

            const int axis = depth % 2;
            const auto mid = begin + (end - begin) / 2;
            std::nth_element(begin, mid, end, [&entries, axis](std::size_t lhs, std::size_t rhs) {
                return Coordinate(entries[lhs].point, axis) < Coordinate(entries[rhs].point, axis);
            });

            auto node = std::make_unique<SCKDTree2d::Node>();
            node->entryIndex = *mid;
            node->splitPoint = entries[*mid].point;
            node->axis = axis;
            node->left = BuildKDTree(entries, begin, mid, depth + 1);
            node->right = BuildKDTree(entries, mid + 1, end, depth + 1);
            return node;
        }

        [[nodiscard]] int InsertIndex(std::unique_ptr<SCKDTree2d::Node>& node,
                                      const std::vector<SCKDTreeEntry2d>& entries,
                                      std::size_t index,
                                      int depth)
        {
            if (!node)
            {
                node = std::make_unique<SCKDTree2d::Node>();
                node->entryIndex = index;
                node->splitPoint = entries[index].point;
                node->axis = depth % 2;
                return depth;
            }
            const bool left = Coordinate(entries[index].point, node->axis) < Coordinate(node->splitPoint, node->axis);
            return InsertIndex(left ? node->left : node->right, entries, index, depth + 1);
        }

        void RemapIndices(SCKDTree2d::Node* node, const std::vector<std::size_t>& mapping)
        {
            if (!node)
            {
                return;
            }
            if (node->active)
            {
                if (mapping[node->entryIndex] == mapping.size())
                {
                    node->active = false;
                }
                else
                {
                    node->entryIndex = mapping[node->entryIndex];
                }
            }
            RemapIndices(node->left.get(), mapping);
            RemapIndices(node->right.get(), mapping);
        }

        [[nodiscard]] SCKDTree2d::Node* DeactivateIndex(SCKDTree2d::Node* node,
                                                       std::size_t index,
                                                       const SCPoint2d& oldPoint)
        {
            if (!node)
            {
                return nullptr;
            }
            if (node->active && node->entryIndex == index)
            {
                node->active = false;
                return node;
            }
            const double delta = Coordinate(oldPoint, node->axis) - Coordinate(node->splitPoint, node->axis);
            if (delta <= 0.0)
            {
                if (auto* found = DeactivateIndex(node->left.get(), index, oldPoint))
                {
                    return found;
                }
            }
            return delta >= 0.0 ? DeactivateIndex(node->right.get(), index, oldPoint) : nullptr;
        }

        void QueryExact(const SCKDTree2d::Node* node,
                        const std::vector<SCKDTreeEntry2d>& entries,
                        const SCPoint2d& point,
                        double eps,
                        std::vector<std::size_t>& result)
        {
            if (node == nullptr)
            {
                return;
            }

            if (node->active && IsEqual(entries[node->entryIndex].point, point, eps))
            {
                result.push_back(entries[node->entryIndex].id);
            }

            const double delta = Coordinate(point, node->axis) - Coordinate(node->splitPoint, node->axis);
            const SCKDTree2d::Node* nearChild = delta <= 0.0 ? node->left.get() : node->right.get();
            const SCKDTree2d::Node* farChild = delta <= 0.0 ? node->right.get() : node->left.get();
            QueryExact(nearChild, entries, point, eps, result);
            if (std::abs(delta) <= eps)
            {
                QueryExact(farChild, entries, point, eps, result);
            }
        }

        void QueryRadius(const SCKDTree2d::Node* node,
                         const std::vector<SCKDTreeEntry2d>& entries,
                         const SCPoint2d& point,
                         double maxDistanceSquared,
                         std::vector<std::size_t>& result)
        {
            if (node == nullptr)
            {
                return;
            }

            if (node->active && DistanceSquared(entries[node->entryIndex].point, point) <= maxDistanceSquared)
            {
                result.push_back(entries[node->entryIndex].id);
            }

            const double delta = Coordinate(point, node->axis) - Coordinate(node->splitPoint, node->axis);
            const SCKDTree2d::Node* nearChild = delta <= 0.0 ? node->left.get() : node->right.get();
            const SCKDTree2d::Node* farChild = delta <= 0.0 ? node->right.get() : node->left.get();
            QueryRadius(nearChild, entries, point, maxDistanceSquared, result);
            if (delta * delta <= maxDistanceSquared)
            {
                QueryRadius(farChild, entries, point, maxDistanceSquared, result);
            }
        }

        void QueryNearest(const SCKDTree2d::Node* node,
                          const std::vector<SCKDTreeEntry2d>& entries,
                          const SCPoint2d& point,
                          std::optional<SCKDTreeHit2d>& best)
        {
            if (node == nullptr)
            {
                return;
            }

            if (node->active)
            {
                const SCKDTreeEntry2d& entry = entries[node->entryIndex];
                const double distanceSquared = DistanceSquared(entry.point, point);
                if (!best || distanceSquared < best->distanceSquared)
                {
                    best = SCKDTreeHit2d{entry.id, entry.point, distanceSquared};
                }
            }

            const double delta = Coordinate(point, node->axis) - Coordinate(node->splitPoint, node->axis);
            const SCKDTree2d::Node* nearChild = delta <= 0.0 ? node->left.get() : node->right.get();
            const SCKDTree2d::Node* farChild = delta <= 0.0 ? node->right.get() : node->left.get();
            QueryNearest(nearChild, entries, point, best);
            if (!best || delta * delta < best->distanceSquared)
            {
                QueryNearest(farChild, entries, point, best);
            }
        }
    }  // namespace

    SCKDTree2d::SCKDTree2d() = default;

    SCKDTree2d::SCKDTree2d(std::vector<SCKDTreeEntry2d> entries) : entries_(std::move(entries))
    {
        Rebuild();
    }

    SCKDTree2d::~SCKDTree2d() = default;
    SCKDTree2d::SCKDTree2d(SCKDTree2d&& other) noexcept
        : entries_(std::move(other.entries_)), root_(std::move(other.root_)),
          nodeCount_(std::exchange(other.nodeCount_, 0))
    {
    }

    SCKDTree2d& SCKDTree2d::operator=(SCKDTree2d&& other) noexcept
    {
        if (this != &other)
        {
            entries_ = std::move(other.entries_);
            root_ = std::move(other.root_);
            nodeCount_ = std::exchange(other.nodeCount_, 0);
        }
        return *this;
    }

    SCKDTree2d::SCKDTree2d(const SCKDTree2d& other) : entries_(other.entries_)
    {
        Rebuild();
    }

    SCKDTree2d& SCKDTree2d::operator=(const SCKDTree2d& other)
    {
        if (this != &other)
        {
            SCKDTree2d copy(other);
            entries_.swap(copy.entries_);
            root_.swap(copy.root_);
            std::swap(nodeCount_, copy.nodeCount_);
        }
        return *this;
    }

    void SCKDTree2d::Rebuild()
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
        auto newRoot = BuildKDTree(entries_, indices.begin(), indices.end(), 0);
        root_ = std::move(newRoot);
        nodeCount_ = indices.size();
    }

    void SCKDTree2d::Clear()
    {
        entries_.clear();
        root_.reset();
        nodeCount_ = 0;
    }

    void SCKDTree2d::Add(std::size_t id, const SCPoint2d& point)
    {
        entries_.push_back(SCKDTreeEntry2d{id, point});
        if (entries_.back().IsValid())
        {
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
            ++nodeCount_;
            if (depth > static_cast<int>(2 * std::bit_width(entries_.size()) + 8))
            {
                Rebuild();
            }
        }
    }

    bool SCKDTree2d::Remove(std::size_t id)
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
                                      [id](const SCKDTreeEntry2d& entry) { return entry.id == id; }),
                       entries_.end());
        RemapIndices(root_.get(), mapping);
        if (entries_.empty())
        {
            root_.reset();
            nodeCount_ = 0;
        }
        else if (nodeCount_ > 2 * entries_.size() + 32)
        {
            Rebuild();
        }
        return true;
    }

    void SCKDTree2d::Update(std::size_t id, const SCPoint2d& point)
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCKDTreeEntry2d& entry) { return entry.id == id; });
        if (it != entries_.end())
        {
            const std::size_t index = static_cast<std::size_t>(it - entries_.begin());
            const SCPoint2d oldPoint = it->point;
            Node* oldNode = nullptr;
            if (it->point.IsValid())
            {
                oldNode = DeactivateIndex(root_.get(), index, oldPoint);
            }
            it->point = point;
            if (point.IsValid())
            {
                int depth;
                try
                {
                    depth = InsertIndex(root_, entries_, index, 0);
                }
                catch (...)
                {
                    it->point = oldPoint;
                    if (oldNode)
                    {
                        oldNode->active = true;
                    }
                    throw;
                }
                ++nodeCount_;
                if (depth > static_cast<int>(2 * std::bit_width(entries_.size()) + 8))
                {
                    Rebuild();
                    return;
                }
            }
            if (nodeCount_ > 2 * entries_.size() + 32)
            {
                Rebuild();
            }
            return;
        }

        Add(id, point);
    }

    std::size_t SCKDTree2d::Size() const
    {
        return entries_.size();
    }

    bool SCKDTree2d::IsEmpty() const
    {
        return entries_.empty();
    }

    bool SCKDTree2d::IsValid() const
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

    bool SCKDTree2d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCKDTreeEntry2d* SCKDTree2d::Find(std::size_t id) const
    {
        const auto it =
            std::find_if(entries_.begin(), entries_.end(), [id](const SCKDTreeEntry2d& entry) { return entry.id == id; });
        return it == entries_.end() ? nullptr : &*it;
    }

    std::vector<std::size_t> SCKDTree2d::Query(const SCPoint2d& point, double eps) const
    {
        std::vector<std::size_t> result;
        QueryExact(root_.get(), entries_, point, eps, result);
        return result;
    }

    std::vector<std::size_t> SCKDTree2d::QueryWithinDistance(const SCPoint2d& point, double maxDistance) const
    {
        std::vector<std::size_t> result;
        const double maxDistanceSquared = maxDistance * maxDistance;
        QueryRadius(root_.get(), entries_, point, maxDistanceSquared, result);
        return result;
    }

    std::optional<SCKDTreeHit2d> SCKDTree2d::Nearest(const SCPoint2d& point) const
    {
        std::optional<SCKDTreeHit2d> best;
        QueryNearest(root_.get(), entries_, point, best);
        return best;
    }

    std::string SCKDTree2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCKDTree2d{size=" << Size() << ", valid=" << (IsValid() ? "true" : "false") << "}";
        return stream.str();
    }

    const std::vector<SCKDTreeEntry2d>& SCKDTree2d::Entries() const
    {
        return entries_;
    }

    const std::vector<SCKDTreeEntry2d>& SCKDTree2d::Data() const
    {
        return Entries();
    }

}  // namespace Geometry
