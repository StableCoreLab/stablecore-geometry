#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>
#include "Brep/SCBrepCoedge.h"
#include "Export/GeometryExport.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepLoop
    {
    public:
        SCBrepLoop() = default;
        explicit SCBrepLoop(std::vector<SCBrepCoedge> coedges) : coedges_(std::move(coedges))
        {
        }

        [[nodiscard]] bool IsValid() const
        {
            if (coedges_.empty())
            {
                return false;
            }

            for (const SCBrepCoedge& coedge : coedges_)
            {
                if (!coedge.IsValid())
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] std::size_t CoedgeCount() const
        {
            return coedges_.size();
        }

        [[nodiscard]] SCBrepCoedge CoedgeAt(std::size_t index) const
        {
            return coedges_.at(index);
        }

        [[nodiscard]] const std::vector<SCBrepCoedge>& Coedges() const
        {
            return coedges_;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBrepLoop{coedgeCount=" << CoedgeCount() << "}";
            return stream.str();
        }

    private:
        std::vector<SCBrepCoedge> coedges_{};
    };
}  // namespace Geometry

