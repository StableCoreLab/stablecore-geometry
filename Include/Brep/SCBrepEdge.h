#pragma once

#include <memory>
#include <sstream>
#include <string>

#include "Core/GeometryTypesPrimitives.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Types/Geometry3d/SCBox3.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepEdge
    {
    public:
        SCBrepEdge() = default;
        SCBrepEdge(std::shared_ptr<ISCCurve3d> curve3d, std::size_t startVertexIndex, std::size_t endVertexIndex)
            : curve3d_(std::move(curve3d)), startVertexIndex_(startVertexIndex), endVertexIndex_(endVertexIndex)
        {
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return curve3d_ != nullptr && curve3d_->IsValid(tolerance) && startVertexIndex_ != endVertexIndex_;
        }

        [[nodiscard]] const ISCCurve3d* Curve() const
        {
            return curve3d_.get();
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            return curve3d_ != nullptr ? curve3d_->Bounds() : SCBox3d{};
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
            stream << "SCBrepEdge{startVertexIndex=" << startVertexIndex_ << ", endVertexIndex=" << endVertexIndex_
                   << "}";
            return stream.str();
        }

    private:
        std::shared_ptr<ISCCurve3d> curve3d_{};
        std::size_t startVertexIndex_{static_cast<std::size_t>(-1)};
        std::size_t endVertexIndex_{static_cast<std::size_t>(-1)};
    };
}  // namespace Geometry


