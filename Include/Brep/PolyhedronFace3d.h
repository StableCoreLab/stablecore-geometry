#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "Support/Epsilon.h"
#include "Brep/PolyhedronLoop3d.h"
#include "Export/GeometryExport.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCGeometryTypes3d.h"
#include "Types/Geometry3d/SCPlane.h"

namespace Geometry
{
    class GEOMETRY_API PolyhedronFace3d
    {
    public:
        PolyhedronFace3d() = default;
        explicit PolyhedronFace3d(PolyhedronLoop3d outerLoop) : outerLoop_(std::move(outerLoop))
        {
        }
        PolyhedronFace3d(SCPlane supportPlane, PolyhedronLoop3d outerLoop, std::vector<PolyhedronLoop3d> holes = {})
            : supportPlane_(supportPlane), outerLoop_(std::move(outerLoop)), holes_(std::move(holes))
        {
        }

        [[deprecated("Use IsValid(const SCGeometryTolerance3d&)")]]
        [[nodiscard]] bool IsValid(double eps) const
        {
            return IsValidWithDistanceEpsilon(eps);
        }

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const
        {
            return tolerance.IsValid() && IsValidWithDistanceEpsilon(tolerance.distanceEpsilon);
        }

        [[nodiscard]] SCPlane SupportPlane() const
        {
            return supportPlane_;
        }

        [[nodiscard]] PolyhedronLoop3d OuterLoop() const
        {
            return outerLoop_;
        }

        [[nodiscard]] const PolyhedronLoop3d& OuterLoopRef() const
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

        [[nodiscard]] const PolyhedronLoop3d& HoleAtRef(std::size_t index) const
        {
            return holes_.at(index);
        }

        [[nodiscard]] SCBox3d Bounds() const
        {
            SCBox3d bounds = outerLoop_.Bounds();
            for (const PolyhedronLoop3d& hole : holes_)
            {
                for (const SCPoint3d& vertex : hole.Vertices())
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
        [[nodiscard]] bool IsValidWithDistanceEpsilon(double eps) const
        {
            if (!supportPlane_.IsValid(eps) || !outerLoop_.IsValid(eps))
            {
                return false;
            }

            auto loopOnPlane = [&](const PolyhedronLoop3d& loop) {
                for (const SCPoint3d& vertex : loop.Vertices())
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
        SCPlane supportPlane_{};
        PolyhedronLoop3d outerLoop_{};
        std::vector<PolyhedronLoop3d> holes_{};
    };
}  // namespace Geometry



