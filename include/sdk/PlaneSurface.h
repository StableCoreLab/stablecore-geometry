#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "sdk/Surface.h"

namespace geometry::sdk
{
namespace detail
{
[[nodiscard]] inline Vector3d MakePerpendicularVector(const Vector3d& normal, double eps)
{
    const Vector3d normalized = normal.Normalized(eps);
    if (normalized.Length() <= eps)
    {
        return Vector3d{};
    }

    // Prefer world-aligned in-plane axes when possible so plane UV coordinates
    // remain stable and intuitive, especially for axis-aligned support planes.
    Vector3d referenceAxis{1.0, 0.0, 0.0};
    if (std::abs(Dot(normalized, referenceAxis)) >= 1.0 - eps)
    {
        referenceAxis = Vector3d{0.0, 1.0, 0.0};
    }

    const Vector3d inPlane = referenceAxis - normalized * Dot(referenceAxis, normalized);
    return inPlane.Normalized(eps);
}
} // namespace detail

class GEOMETRY_API PlaneSurface final : public Surface
{
public:
    PlaneSurface() = default;
    PlaneSurface(Plane plane, Vector3d uAxis, Vector3d vAxis, Intervald uRange, Intervald vRange)
        : plane_(plane), uAxis_(uAxis), vAxis_(vAxis), uRange_(uRange), vRange_(vRange)
    {
    }

    [[nodiscard]] static PlaneSurface FromPlane(
        const Plane& plane,
        Intervald uRange = Intervald{-1.0, 1.0},
        Intervald vRange = Intervald{-1.0, 1.0},
        double eps = geometry::kDefaultEpsilon)
    {
        const Vector3d uAxis = detail::MakePerpendicularVector(plane.normal, eps);
        const Vector3d vAxis = Cross(plane.UnitNormal(eps), uAxis).Normalized(eps);
        return PlaneSurface(plane, uAxis, vAxis, uRange, vRange);
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const override
    {
        if (!plane_.IsValid(tolerance.distanceEpsilon) || !uRange_.IsValid() || !vRange_.IsValid())
        {
            return false;
        }

        if (uAxis_.Length() <= tolerance.distanceEpsilon || vAxis_.Length() <= tolerance.distanceEpsilon)
        {
            return false;
        }

        const Vector3d unitNormal = plane_.UnitNormal(tolerance.distanceEpsilon);
        const Vector3d unitU = uAxis_.Normalized(tolerance.distanceEpsilon);
        const Vector3d unitV = vAxis_.Normalized(tolerance.distanceEpsilon);
        return std::abs(Dot(unitU, unitNormal)) <= tolerance.angleEpsilon &&
               std::abs(Dot(unitV, unitNormal)) <= tolerance.angleEpsilon &&
               std::abs(Dot(unitU, unitV)) <= tolerance.angleEpsilon;
    }

    [[nodiscard]] Intervald URange() const override
    {
        return uRange_;
    }

    [[nodiscard]] Intervald VRange() const override
    {
        return vRange_;
    }

    [[nodiscard]] Point3d PointAt(double u, double v) const override
    {
        return plane_.origin + uAxis_ * u + vAxis_ * v;
    }

    [[nodiscard]] SurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
    {
        derivativeOrder = std::clamp(derivativeOrder, 0, 1);
        SurfaceEval3d eval{};
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

    [[nodiscard]] Box3d Bounds() const override
    {
        Box3d bounds{};
        bounds.ExpandToInclude(PointAt(uRange_.min, vRange_.min));
        bounds.ExpandToInclude(PointAt(uRange_.min, vRange_.max));
        bounds.ExpandToInclude(PointAt(uRange_.max, vRange_.min));
        bounds.ExpandToInclude(PointAt(uRange_.max, vRange_.max));
        return bounds;
    }

    [[nodiscard]] std::unique_ptr<Surface> Clone() const override
    {
        return std::make_unique<PlaneSurface>(*this);
    }

    [[nodiscard]] Plane SupportPlane() const
    {
        return plane_;
    }

    [[nodiscard]] Vector3d UAxis() const
    {
        return uAxis_;
    }

    [[nodiscard]] Vector3d VAxis() const
    {
        return vAxis_;
    }

private:
    Plane plane_{};
    Vector3d uAxis_{1.0, 0.0, 0.0};
    Vector3d vAxis_{0.0, 1.0, 0.0};
    Intervald uRange_{-1.0, 1.0};
    Intervald vRange_{-1.0, 1.0};
};
} // namespace geometry::sdk
