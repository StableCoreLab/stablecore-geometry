#pragma once

#include <cstddef>
#include <sstream>
#include <string>

#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    class GEOMETRY_API SCBrepVertex
    {
    public:
        SCBrepVertex() = default;
        explicit SCBrepVertex(SCPoint3d point) : point_(point)
        {
        }

        [[nodiscard]] bool IsValid() const
        {
            return point_.IsValid();
        }

        [[nodiscard]] SCPoint3d Point() const
        {
            return point_;
        }

        [[nodiscard]] std::string DebugString() const
        {
            std::ostringstream stream;
            stream << "SCBrepVertex{point=" << point_.DebugString() << "}";
            return stream.str();
        }

    private:
        SCPoint3d point_{};
    };
}  // namespace Geometry

