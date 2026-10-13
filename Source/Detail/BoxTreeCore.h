#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace Geometry::Detail
{
    template <typename Entry, typename Traits>
    [[nodiscard]] auto ComputeBoxTreeBounds(const std::vector<Entry>& entries, const std::vector<std::size_t>& indices)
        -> decltype(Traits::EmptyBox())
    {
        auto bounds = Traits::EmptyBox();
        for (std::size_t index : indices)
        {
            Traits::ExpandBounds(bounds, entries[index].box);
        }
        return bounds;
    }

    template <typename Node, typename Entry, typename Traits>
    [[nodiscard]] std::unique_ptr<Node> BuildBoxTree(const std::vector<Entry>& entries,
                                                     std::vector<std::size_t> indices,
                                                     int depth)
    {
        if (indices.empty())
        {
            return {};
        }

        auto node = std::make_unique<Node>();
        node->bounds = ComputeBoxTreeBounds<Entry, Traits>(entries, indices);
        if (indices.size() <= 4)
        {
            node->leafIndices = std::move(indices);
            return node;
        }

        const int axis = depth % Traits::AxisCount;
        std::sort(indices.begin(), indices.end(), [&entries, axis](std::size_t lhs, std::size_t rhs) {
            return Traits::CenterCoordinate(entries[lhs].box, axis) <
                   Traits::CenterCoordinate(entries[rhs].box, axis);
        });

        const std::size_t mid = indices.size() / 2;
        std::vector<std::size_t> leftIndices(indices.begin(), indices.begin() + static_cast<std::ptrdiff_t>(mid));
        std::vector<std::size_t> rightIndices(indices.begin() + static_cast<std::ptrdiff_t>(mid), indices.end());
        node->left = BuildBoxTree<Node, Entry, Traits>(entries, std::move(leftIndices), depth + 1);
        node->right = BuildBoxTree<Node, Entry, Traits>(entries, std::move(rightIndices), depth + 1);
        return node;
    }

    template <typename Node, typename Entry, typename Traits>
    void RefreshBoxTreeBounds(Node& node, const std::vector<Entry>& entries)
    {
        node.bounds = Traits::EmptyBox();
        if (node.IsLeaf())
        {
            node.bounds = ComputeBoxTreeBounds<Entry, Traits>(entries, node.leafIndices);
            return;
        }
        if (node.left && node.left->bounds.IsValid())
        {
            Traits::ExpandBounds(node.bounds, node.left->bounds);
        }
        if (node.right && node.right->bounds.IsValid())
        {
            Traits::ExpandBounds(node.bounds, node.right->bounds);
        }
    }

    template <typename Node, typename Entry, typename Traits>
    [[nodiscard]] int InsertBoxTreeIndex(std::unique_ptr<Node>& node,
                                         const std::vector<Entry>& entries,
                                         std::size_t index,
                                         int depth)
    {
        if (!node)
        {
            auto newNode = std::make_unique<Node>();
            newNode->leafIndices.push_back(index);
            RefreshBoxTreeBounds<Node, Entry, Traits>(*newNode, entries);
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
                node = BuildBoxTree<Node, Entry, Traits>(entries, std::move(indices), depth);
                insertedDepth = depth + 1;
            }
            else
            {
                node->leafIndices.push_back(index);
            }
        }
        else
        {
            const double leftDistance = Traits::DistanceSquared(entries[index].box.Center(), node->left->bounds);
            const double rightDistance = Traits::DistanceSquared(entries[index].box.Center(), node->right->bounds);
            insertedDepth = InsertBoxTreeIndex<Node, Entry, Traits>(
                leftDistance <= rightDistance ? node->left : node->right, entries, index, depth + 1);
        }
        RefreshBoxTreeBounds<Node, Entry, Traits>(*node, entries);
        return insertedDepth;
    }

    template <typename Node, typename Box, typename Traits>
    [[nodiscard]] bool FindBoxTreeIndexPath(std::unique_ptr<Node>& node,
                                           std::size_t index,
                                           const Box& box,
                                           std::vector<std::unique_ptr<Node>*>& path)
    {
        if (!node || !Traits::Intersects(node->bounds, box, 0.0))
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
        else if (FindBoxTreeIndexPath<Node, Box, Traits>(node->left, index, box, path) ||
                 FindBoxTreeIndexPath<Node, Box, Traits>(node->right, index, box, path))
        {
            return true;
        }
        path.pop_back();
        return false;
    }

    template <typename Node, typename Entry, typename Traits>
    void RefreshBoxTreePath(const std::vector<std::unique_ptr<Node>*>& path,
                            const std::vector<Entry>& entries,
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
                RefreshBoxTreeBounds<Node, Entry, Traits>(*node, entries);
            }
        }
    }

    template <typename Node, typename Entry, typename Traits>
    void RemapBoxTreeIndices(std::unique_ptr<Node>& node,
                             const std::vector<Entry>& entries,
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
            RemapBoxTreeIndices<Node, Entry, Traits>(node->left, entries, mapping);
            RemapBoxTreeIndices<Node, Entry, Traits>(node->right, entries, mapping);
            if (!node->left || !node->right)
            {
                node = std::move(node->left ? node->left : node->right);
            }
        }
        if (node)
        {
            RefreshBoxTreeBounds<Node, Entry, Traits>(*node, entries);
        }
    }

    template <typename Node, typename Entry, typename Box, typename Traits>
    void QueryBoxTree(const Node* node,
                      const std::vector<Entry>& entries,
                      const Box& box,
                      double epsilon,
                      std::vector<std::size_t>& result)
    {
        if (node == nullptr || !Traits::Intersects(node->bounds, box, epsilon))
        {
            return;
        }
        if (node->IsLeaf())
        {
            for (std::size_t index : node->leafIndices)
            {
                if (Traits::Intersects(entries[index].box, box, epsilon))
                {
                    result.push_back(entries[index].id);
                }
            }
            return;
        }
        QueryBoxTree<Node, Entry, Box, Traits>(node->left.get(), entries, box, epsilon, result);
        QueryBoxTree<Node, Entry, Box, Traits>(node->right.get(), entries, box, epsilon, result);
    }

    template <typename Node, typename Entry, typename Point, typename Traits>
    void QueryPointTree(const Node* node,
                       const std::vector<Entry>& entries,
                       const Point& point,
                       double epsilon,
                       std::vector<std::size_t>& result)
    {
        if (node == nullptr || !Traits::Contains(node->bounds, point, epsilon))
        {
            return;
        }
        if (node->IsLeaf())
        {
            for (std::size_t index : node->leafIndices)
            {
                if (Traits::Contains(entries[index].box, point, epsilon))
                {
                    result.push_back(entries[index].id);
                }
            }
            return;
        }
        QueryPointTree<Node, Entry, Point, Traits>(node->left.get(), entries, point, epsilon, result);
        QueryPointTree<Node, Entry, Point, Traits>(node->right.get(), entries, point, epsilon, result);
    }

    template <typename Hit>
    [[nodiscard]] bool CompareBoxTreeHits(const Hit& lhs, const Hit& rhs)
    {
        return lhs.distanceSquared == rhs.distanceSquared ? lhs.id < rhs.id
                                                          : lhs.distanceSquared < rhs.distanceSquared;
    }

    template <typename Node, typename Entry, typename Point, typename Hit, typename Traits>
    void QueryBoxTreeKnn(const Node* node,
                         const std::vector<Entry>& entries,
                         const Point& point,
                         std::size_t k,
                         double epsilon,
                         std::vector<Hit>& hits)
    {
        if (node == nullptr || k == 0)
        {
            return;
        }
        if (hits.size() >= k)
        {
            const double factor = 1.0 + std::max(0.0, epsilon);
            const double lowerBound = Traits::DistanceSquared(point, node->bounds);
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
                Hit hit{entry.id, entry.box, Traits::DistanceSquared(point, entry.box)};
                if (hits.size() < k)
                {
                    hits.push_back(hit);
                    std::push_heap(hits.begin(), hits.end(), CompareBoxTreeHits<Hit>);
                }
                else if (CompareBoxTreeHits(hit, hits.front()))
                {
                    std::pop_heap(hits.begin(), hits.end(), CompareBoxTreeHits<Hit>);
                    hits.back() = hit;
                    std::push_heap(hits.begin(), hits.end(), CompareBoxTreeHits<Hit>);
                }
            }
            return;
        }

        const Node* first = node->left.get();
        const Node* second = node->right.get();
        if (Traits::DistanceSquared(point, second->bounds) < Traits::DistanceSquared(point, first->bounds))
        {
            std::swap(first, second);
        }
        QueryBoxTreeKnn<Node, Entry, Point, Hit, Traits>(first, entries, point, k, epsilon, hits);
        QueryBoxTreeKnn<Node, Entry, Point, Hit, Traits>(second, entries, point, k, epsilon, hits);
    }
}  // namespace Geometry::Detail
