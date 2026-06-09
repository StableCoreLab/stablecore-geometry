#pragma once

#include <string>
#include <string_view>

#include "Core/Algorithms.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry::Serialize
{
    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCPoint2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCPoint2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCVector2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCVector2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCBox2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCBox2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCSegmentProjection2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCSegmentProjection2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCArcSegment2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCArcSegment2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCPolyline2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCPolyline2d& value);

    [[nodiscard]] GEOMETRY_API std::string ToText(const Geometry::SCPolygon2d& value);
    [[nodiscard]] GEOMETRY_API bool FromText(std::string_view text, Geometry::SCPolygon2d& value);
}  // namespace Geometry::Serialize
