#pragma once

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/PolyhedronLoop3d.h"

namespace geometry::sdk
{
class GEOMETRY_API PolyhedronFace3d
{
public:
    PolyhedronFace3d() = default;
    explicit PolyhedronFace3d(PolyhedronLoop3d outerLoop) : outerLoop_(std::move(outerLoop)) {}
    PolyhedronFace3d(Plane supportPlane, PolyhedronLoop3d outerLoop, std::vector<PolyhedronLoop3d> holes = {})
        : supportPlane_(supportPlane), outerLoop_(std::move(outerLoop)), holes_(std::move(holes))
    {
    }

    [[nodiscard]] bool IsValid(double eps = geometry::kDefaultEpsilon) const
    {
        if (!supportPlane_.IsValid(eps) || !outerLoop_.IsValid(eps))
        {
            return false;
        }

        auto loopOnPlane = [&](const PolyhedronLoop3d& loop) {
            for (const Point3d& vertex : loop.Vertices())
            {
                if (std::abs(supportPlane_.SignedDistanceTo(vertex, eps)) > eps)
                {
                    return false;
                }
            }
            return true;
        };

        if (!loopOnPlane(outerLoop_))
        {
            return false;
        }

        for (const PolyhedronLoop3d& hole : holes_)
        {
            if (!hole.IsValid(eps) || !loopOnPlane(hole))
            {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] Plane SupportPlane() const
    {
        return supportPlane_;
    }

    [[nodiscard]] PolyhedronLoop3d OuterLoop() const
    {
        return outerLoop_;
    }

    [[nodiscard]] std::size_t HoleCount() const
    {
        return holes_.size();
    }

    [[nodiscard]] PolyhedronLoop3d HoleAt(std::size_t index) const
    {
        return holes_.at(index);
    }

    [[nodiscard]] Box3d Bounds() const
    {
        Box3d bounds = outerLoop_.Bounds();
        for (const PolyhedronLoop3d& hole : holes_)
        {
            for (const Point3d& vertex : hole.Vertices())
            {
                bounds.ExpandToInclude(vertex);
            }
        }
        return bounds;
    }

    [[nodiscard]] std::string DebugString() const
    {
        std::ostringstream stream;
        stream << "PolyhedronFace3d{holeCount=" << HoleCount() << "}";
        return stream.str();
    }

private:
    Plane supportPlane_{};
    PolyhedronLoop3d outerLoop_{};
    std::vector<PolyhedronLoop3d> holes_{};
};
} // namespace geometry::sdk
