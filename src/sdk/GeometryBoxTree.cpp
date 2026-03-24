#include "sdk/GeometryBoxTree.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <utility>

#include "sdk/GeometryMetrics.h"

namespace geometry::sdk
{
namespace
{
struct BoxNode
{
    Box2d bounds{};
    std::vector<std::size_t> leafIndices{};
    std::unique_ptr<BoxNode> left{};
    std::unique_ptr<BoxNode> right{};

    [[nodiscard]] bool IsLeaf() const
    {
        return left == nullptr && right == nullptr;
    }
};

[[nodiscard]] double CenterCoordinate(const Box2d& box, int axis)
{
    const Point2d center = box.Center();
    return axis == 0 ? center.x : center.y;
}

[[nodiscard]] Box2d ComputeBounds(
    const std::vector<BoxTreeEntry2d>& entries,
    const std::vector<std::size_t>& indices)
{
    Box2d bounds;
    for (std::size_t index : indices)
    {
        bounds.ExpandToInclude(entries[index].box);
    }
    return bounds;
}

[[nodiscard]] std::unique_ptr<BoxNode> BuildBoxTree(
    const std::vector<BoxTreeEntry2d>& entries,
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

    const int axis = depth % 2;
    std::sort(
        indices.begin(),
        indices.end(),
        [&entries, axis](std::size_t lhs, std::size_t rhs) {
            return CenterCoordinate(entries[lhs].box, axis) < CenterCoordinate(entries[rhs].box, axis);
        });

    const std::size_t mid = indices.size() / 2;
    std::vector<std::size_t> leftIndices(indices.begin(), indices.begin() + static_cast<std::ptrdiff_t>(mid));
    std::vector<std::size_t> rightIndices(indices.begin() + static_cast<std::ptrdiff_t>(mid), indices.end());
    node->left = BuildBoxTree(entries, std::move(leftIndices), depth + 1);
    node->right = BuildBoxTree(entries, std::move(rightIndices), depth + 1);
    return node;
}

void QueryBox(
    const BoxNode* node,
    const std::vector<BoxTreeEntry2d>& entries,
    const Box2d& box,
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

void QueryPoint(
    const BoxNode* node,
    const std::vector<BoxTreeEntry2d>& entries,
    const Point2d& point,
    double eps,
    std::vector<std::size_t>& result)
{
    if (node == nullptr || !geometry::sdk::Contains(node->bounds, point, eps))
    {
        return;
    }

    if (node->IsLeaf())
    {
        for (std::size_t index : node->leafIndices)
        {
            if (geometry::sdk::Contains(entries[index].box, point, eps))
            {
                result.push_back(entries[index].id);
            }
        }
        return;
    }

    QueryPoint(node->left.get(), entries, point, eps, result);
    QueryPoint(node->right.get(), entries, point, eps, result);
}
} // namespace

GeometryBoxTree2d::GeometryBoxTree2d(std::vector<BoxTreeEntry2d> entries)
    : entries_(std::move(entries))
{
}

void GeometryBoxTree2d::Clear()
{
    entries_.clear();
}

void GeometryBoxTree2d::Add(std::size_t id, const Box2d& box)
{
    entries_.push_back(BoxTreeEntry2d{id, box});
}

bool GeometryBoxTree2d::Remove(std::size_t id)
{
    const auto it = std::remove_if(entries_.begin(), entries_.end(), [id](const BoxTreeEntry2d& entry) {
        return entry.id == id;
    });
    if (it == entries_.end())
    {
        return false;
    }

    entries_.erase(it, entries_.end());
    return true;
}

void GeometryBoxTree2d::Update(std::size_t id, const Box2d& box)
{
    if (auto* entry = const_cast<BoxTreeEntry2d*>(Find(id)); entry != nullptr)
    {
        entry->box = box;
        return;
    }

    Add(id, box);
}

std::size_t GeometryBoxTree2d::Size() const
{
    return entries_.size();
}

bool GeometryBoxTree2d::IsEmpty() const
{
    return entries_.empty();
}

bool GeometryBoxTree2d::IsValid() const
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

bool GeometryBoxTree2d::Contains(std::size_t id) const
{
    return Find(id) != nullptr;
}

const BoxTreeEntry2d* GeometryBoxTree2d::Find(std::size_t id) const
{
    const auto it = std::find_if(entries_.begin(), entries_.end(), [id](const BoxTreeEntry2d& entry) {
        return entry.id == id;
    });
    return it == entries_.end() ? nullptr : &*it;
}

std::vector<std::size_t> GeometryBoxTree2d::Query(const Box2d& box, double eps) const
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

std::vector<std::size_t> GeometryBoxTree2d::QueryContaining(const Point2d& point, double eps) const
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

std::string GeometryBoxTree2d::DebugString() const
{
    std::ostringstream stream;
    stream << "GeometryBoxTree2d{size=" << Size()
           << ", valid=" << (IsValid() ? "true" : "false") << "}";
    return stream.str();
}

const std::vector<BoxTreeEntry2d>& GeometryBoxTree2d::Data() const
{
    return entries_;
}

std::vector<BoxTreeEntry2d>& GeometryBoxTree2d::Data()
{
    return entries_;
}
} // namespace geometry::sdk
