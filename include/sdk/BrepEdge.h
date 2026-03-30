#pragma once

#include <memory>
#include <sstream>
#include <string>

#include "export/GeometryExport.h"
#include "sdk/Curve3d.h"
#include "sdk/GeometryTypes.h"

namespace geometry::sdk
{
class GEOMETRY_API BrepEdge
{
public:
    BrepEdge() = default;
    BrepEdge(std::shared_ptr<Curve3d> curve3d, std::size_t startVertexIndex, std::size_t endVertexIndex)
        : curve3d_(std::move(curve3d)), startVertexIndex_(startVertexIndex), endVertexIndex_(endVertexIndex)
    {
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const
    {
        return curve3d_ != nullptr && curve3d_->IsValid(tolerance) && startVertexIndex_ != endVertexIndex_;
    }

    [[nodiscard]] const Curve3d* Curve() const
    {
        return curve3d_.get();
    }

    [[nodiscard]] Box3d Bounds() const
    {
        return curve3d_ != nullptr ? curve3d_->Bounds() : Box3d{};
    }

    [[nodiscard]] std::size_t StartVertexIndex() const
    {
        return startVertexIndex_;
    }

    [[nodiscard]] std::size_t EndVertexIndex() const
    {
        return endVertexIndex_;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "BrepEdge{startVertexIndex=" << startVertexIndex_
               << ", endVertexIndex=" << endVertexIndex_ << "}";
        return stream.str();
    }

private:
    std::shared_ptr<Curve3d> curve3d_{};
    std::size_t startVertexIndex_{static_cast<std::size_t>(-1)};
    std::size_t endVertexIndex_{static_cast<std::size_t>(-1)};
};
} // namespace geometry::sdk
