#pragma once

#include <algorithm>
#include <memory>

#include "Geometry3d/ISCSurface.h"
#include "Types/Geometry3d/SCPlane.h"

namespace Geometry
{
    class GEOMETRY_API SCPlaneSurface final : public ISCSurface
    {
    public:
        SCPlaneSurface() = default;
        SCPlaneSurface(SCPlane plane, SCVector3d uAxis, SCVector3d vAxis, SCIntervald uRange, SCIntervald vRange)
            : plane_(plane), uAxis_(uAxis), vAxis_(vAxis), uRange_(uRange), vRange_(vRange)
        {
        }

        [[nodiscard]] static SCPlaneSurface FromPlane(const SCPlane& plane,
                                                    SCIntervald uRange = SCIntervald{-1.0, 1.0},
                                                    SCIntervald vRange = SCIntervald{-1.0, 1.0},
                                                    double eps = Geometry::kDefaultEpsilon);

        [[nodiscard]] bool IsValid(const SCGeometryTolerance3d& tolerance = {}) const override
        {
            if (!plane_.IsValid(tolerance.distanceEpsilon) || !uRange_.IsValid() || !vRange_.IsValid())
            {
                return false;
            }

            if (uAxis_.Length() <= tolerance.distanceEpsilon || vAxis_.Length() <= tolerance.distanceEpsilon)
            {
                return false;
            }

            const SCVector3d unitNormal = plane_.UnitNormal(tolerance.distanceEpsilon);
            const SCVector3d unitU = uAxis_.Normalized(tolerance.distanceEpsilon);
            const SCVector3d unitV = vAxis_.Normalized(tolerance.distanceEpsilon);
            return std::abs(Dot(unitU, unitNormal)) <= tolerance.angleEpsilon &&
                   std::abs(Dot(unitV, unitNormal)) <= tolerance.angleEpsilon &&
                   std::abs(Dot(unitU, unitV)) <= tolerance.angleEpsilon;
        }

        [[nodiscard]] SCIntervald URange() const override
        {
            return uRange_;
        }

        [[nodiscard]] SCIntervald VRange() const override
        {
            return vRange_;
        }

        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override
        {
            return plane_.origin + uAxis_ * u + vAxis_ * v;
        }

        [[nodiscard]] SCSurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
        {
            derivativeOrder = std::clamp(derivativeOrder, 0, 1);
            SCSurfaceEval3d eval{};
            eval.point = PointAt(u, v);
            eval.derivativeOrder = derivativeOrder;
            if (derivativeOrder >= 1)
            {
                eval.derivativeU = uAxis_;
                eval.derivativeV = vAxis_;
                eval.normal = Cross(uAxis_, vAxis_);
            }
            return eval;
        }

        [[nodiscard]] SCBox3d Bounds() const override
        {
            SCBox3d bounds{};
            bounds.ExpandToInclude(PointAt(uRange_.min, vRange_.min));
            bounds.ExpandToInclude(PointAt(uRange_.min, vRange_.max));
            bounds.ExpandToInclude(PointAt(uRange_.max, vRange_.min));
            bounds.ExpandToInclude(PointAt(uRange_.max, vRange_.max));
            return bounds;
        }

        [[nodiscard]] std::unique_ptr<ISCSurface> Clone() const override
        {
            return std::make_unique<SCPlaneSurface>(*this);
        }

        [[nodiscard]] SCPlane SupportPlane() const
        {
            return plane_;
        }

        [[nodiscard]] SCVector3d UAxis() const
        {
            return uAxis_;
        }

        [[nodiscard]] SCVector3d VAxis() const
        {
            return vAxis_;
        }

    private:
        SCPlane plane_{};
        SCVector3d uAxis_{1.0, 0.0, 0.0};
        SCVector3d vAxis_{0.0, 1.0, 0.0};
        SCIntervald uRange_{-1.0, 1.0};
        SCIntervald vRange_{-1.0, 1.0};
    };
}  // namespace Geometry




