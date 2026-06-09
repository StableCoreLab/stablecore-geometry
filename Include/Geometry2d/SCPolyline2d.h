#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"

namespace Geometry
{
    enum class SCPolylineClosure
    {
        Open,
        Closed
    };

    class GEOMETRY_API SCPolyline2d
    {
    public:
        SCPolyline2d();
        explicit SCPolyline2d(SCPolylineClosure closure);
        SCPolyline2d(std::vector<SCPoint2d> points, SCPolylineClosure closure = SCPolylineClosure::Open);
        SCPolyline2d(std::vector<std::shared_ptr<ISCSegment2d>> segments, SCPolylineClosure closure);
        SCPolyline2d(const SCPolyline2d& other);
        SCPolyline2d(SCPolyline2d&& other) noexcept;
        SCPolyline2d& operator=(const SCPolyline2d& other);
        SCPolyline2d& operator=(SCPolyline2d&& other) noexcept;
        ~SCPolyline2d();

        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool IsClosed() const;
        [[nodiscard]] std::size_t PointCount() const;
        [[nodiscard]] std::size_t VertexCount() const;
        [[nodiscard]] std::size_t SegmentCount() const;
        [[nodiscard]] double Length() const;
        [[nodiscard]] SCPoint2d PointAt(std::size_t index) const;
        [[nodiscard]] SCPoint2d PointAt(int index) const
        {
            return PointAt(static_cast<std::size_t>(index));
        }
        [[nodiscard]] SCPoint2d PointAt(double parameter) const;
        [[nodiscard]] SCPoint2d VertexAt(std::size_t index) const;
        [[nodiscard]] SCPoint2d StartPoint() const;
        [[nodiscard]] SCPoint2d EndPoint() const;
        [[nodiscard]] double LengthAt(double parameter) const;
        [[nodiscard]] double ParameterAtLength(double distanceFromStart, bool clampToPath = false) const;
        [[nodiscard]] SCPoint2d PointAtLength(double distanceFromStart, bool clampToPath = false) const;
        [[nodiscard]] std::unique_ptr<ISCSegment2d> SegmentAt(std::size_t index) const;
        [[nodiscard]] SCBox2d Bounds() const;
        [[nodiscard]] std::string DebugString() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        explicit SCPolyline2d(std::unique_ptr<Impl> impl);
    };
}  // namespace Geometry
