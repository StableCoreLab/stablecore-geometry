#include "Geometry3d/SCOffsetSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Geometry3d/SCPlaneSurface.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool EstimateOffsetTangent(const SCOffsetSurface& surface,
                                                 double u, double v, bool alongU,
                                                 SCVector3d& tangent)
        {
            const SCIntervald range = alongU ? surface.URange() : surface.VRange();
            const double parameter = alongU ? u : v;
            if (!range.IsValid() || !range.Contains(parameter, 0.0))
            {
                return false;
            }

            const double span = range.max - range.min;
            if (!std::isfinite(span) || span <= 0.0)
            {
                return false;
            }
            const double roundoffStep = 64.0 * std::numeric_limits<double>::epsilon() *
                                        std::max(1.0, std::abs(parameter));
            const double step = std::min(span, std::max(span * 1e-5, roundoffStep));
            const double lower = std::max(range.min, parameter - step);
            const double upper = std::min(range.max, parameter + step);
            if (!(lower < upper))
            {
                return false;
            }

            const SCPoint3d first = alongU ? surface.PointAt(lower, v) : surface.PointAt(u, lower);
            const SCPoint3d second = alongU ? surface.PointAt(upper, v) : surface.PointAt(u, upper);
            if (!first.IsValid() || !second.IsValid())
            {
                return false;
            }
            tangent = (second - first) / (upper - lower);
            return tangent.IsValid();
        }
    }

    SCSurfaceEval3d SCOffsetSurface::Evaluate(double u, double v, int derivativeOrder) const
    {
        if (baseSurface_ == nullptr)
        {
            return {};
        }

        const int order = std::clamp(derivativeOrder, 0, 1);
        SCSurfaceEval3d eval = baseSurface_->Evaluate(u, v, order);
        if (offsetDistance_ == 0.0)
        {
            return eval;
        }

        eval.point = PointAt(u, v);
        eval.derivativeOrder = 0;
        if (order == 0 || !eval.point.IsValid())
        {
            return eval;
        }

        if (dynamic_cast<const SCPlaneSurface*>(baseSurface_.get()) != nullptr)
        {
            eval.derivativeOrder = order;
            return eval;
        }

        SCVector3d tangentU{};
        SCVector3d tangentV{};
        if (!EstimateOffsetTangent(*this, u, v, true, tangentU) ||
            !EstimateOffsetTangent(*this, u, v, false, tangentV))
        {
            return eval;
        }

        SCVector3d unitNormal{};
        const SCVector3d normal = Cross(tangentU, tangentV);
        if (!Detail::TryUnitVector(normal, unitNormal))
        {
            return eval;
        }
        eval.derivativeU = tangentU;
        eval.derivativeV = tangentV;
        eval.normal = normal;
        eval.derivativeOrder = 1;
        return eval;
    }
}  // namespace Geometry
