#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "Export/GeometryExport.h"
#include "Core/PathGeometry.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    struct GEOMETRY_API SCOffsetSegmentMapping2d
    {
        std::size_t sourceSegmentIndex{0};
        std::size_t outputSegmentIndex{0};
        double sourceStartParameter{0.0};
        double sourceEndParameter{1.0};
        double sourceLength2d{0.0};
        double outputLength2d{0.0};
    };

    struct GEOMETRY_API SCMappedOffsetOptions2d
    {
        double miterLimit{0.0};
    };

    struct GEOMETRY_API SCMappedOffsetResult2d
    {
        bool success{false};
        SCPathGeometryFailure failure{SCPathGeometryFailure::None};
        std::optional<std::size_t> inputIndex{};
        std::optional<std::size_t> relatedIndex{};
        SCPolyline2d polyline{};
        std::vector<SCOffsetSegmentMapping2d> mappings{};
        std::vector<double> outputCumulativeLengths{};
        double inputTotalLength2d{0.0};
        double outputTotalLength2d{0.0};
    };

    [[nodiscard]] GEOMETRY_API SCMappedOffsetResult2d OffsetWithMapping(
        const SCPolyline2d& polyline, double signedDistance,
        const SCMappedOffsetOptions2d& options, const SCPathGeometryTolerance& tolerance);

    struct GEOMETRY_API SCOffsetOptions2d
    {
        double miterLimit{4.0};
    };

    [[nodiscard]] GEOMETRY_API SCLineSegment2d Offset(const SCLineSegment2d& segment, double distance);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Offset(const SCArcSegment2d& segment, double distance);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Offset(const SCPolyline2d& polyline,
                                                 double distance,
                                                 SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCPolygon2d Offset(const SCPolygon2d& polygon,
                                                double distance,
                                                SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d OffsetToMultiPolygon(const SCPolygon2d& polygon,
                                                                   double distance,
                                                                   SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolyline2d Offset(const SCMultiPolyline2d& polylines,
                                                      double distance,
                                                      SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Offset(const SCMultiPolygon2d& polygons,
                                                     double distance,
                                                     SCOffsetOptions2d options = {});
}  // namespace Geometry

