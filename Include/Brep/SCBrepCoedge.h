#pragma once

#include <cstddef>
#include <sstream>
#include <string>

#include "Export/GeometryExport.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepCoedge
    {
    public:
        SCBrepCoedge() = default;
        SCBrepCoedge(std::size_t edgeIndex, bool reversed = false) : edgeIndex_(edgeIndex), reversed_(reversed)
        {
        }

        [[nodiscard]] bool IsValid() const
        {
            return edgeIndex_ != static_cast<std::size_t>(-1);
        }

        [[nodiscard]] std::size_t EdgeIndex() const
        {
            return edgeIndex_;
        }

        [[nodiscard]] bool Reversed() const
        {
            return reversed_;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBrepCoedge{edgeIndex=" << edgeIndex_ << ", reversed=" << (reversed_ ? "true" : "false") << "}";
            return stream.str();
        }

    private:
        std::size_t edgeIndex_{static_cast<std::size_t>(-1)};
        bool reversed_{false};
    };
}  // namespace Geometry

