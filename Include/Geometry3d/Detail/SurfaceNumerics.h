#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "Types/Geometry3d/SCGeometryTypes3d.h"

namespace Geometry::Detail
{
    // 只接受坐标表示的舍入差；不引入几何容差或单位尺度绝对误差。
    [[nodiscard]] inline bool SameRoundedCoordinate(double a, double b)
    {
        if (!std::isfinite(a) || !std::isfinite(b)) return false;
        if (a == b) return true;
        const double magnitude = std::max(std::abs(a), std::abs(b));
        const double spacing = magnitude - std::nextafter(magnitude, 0.0);
        const double ulp = std::max(spacing, std::numeric_limits<double>::denorm_min());
        // 除以 ulp 后再作比较，避免差值和误差界溢出。
        return std::signbit(a) == std::signbit(b)
            ? (magnitude - std::min(std::abs(a), std::abs(b))) / ulp <= 8.0
            : std::abs(a) / ulp + std::abs(b) / ulp <= 8.0;
    }

    [[nodiscard]] inline bool SameRoundedPosition(const SCPoint3d& a, const SCPoint3d& b)
    {
        return SameRoundedCoordinate(a.x, b.x) && SameRoundedCoordinate(a.y, b.y) &&
               SameRoundedCoordinate(a.z, b.z);
    }
    [[nodiscard]] inline bool TryUnitVector(const SCVector3d& vector, SCVector3d& unit)
    {
        if (!vector.IsValid())
        {
            return false;
        }
        const double scale = std::max({std::abs(vector.x), std::abs(vector.y), std::abs(vector.z)});
        if (!(scale > 0.0))
        {
            return false;
        }
        const SCVector3d scaled = vector / scale;
        const double length = std::hypot(scaled.x, scaled.y, scaled.z);
        unit = scaled / length;
        return unit.IsValid();
    }

    [[nodiscard]] inline bool HasIndependentSurfaceTangents(const SCSurfaceEval3d& eval, double angleEpsilon)
    {
        if (eval.derivativeOrder < 1 || !eval.point.IsValid() || !eval.normal.IsValid() ||
            !std::isfinite(angleEpsilon) || angleEpsilon < 0.0)
        {
            return false;
        }
        SCVector3d normal, unitU, unitV;
        if (!TryUnitVector(eval.normal, normal) ||
            !TryUnitVector(eval.derivativeU, unitU) || !TryUnitVector(eval.derivativeV, unitV))
        {
            return false;
        }
        const SCVector3d cross = Cross(unitU, unitV);
        return std::hypot(cross.x, cross.y, cross.z) > angleEpsilon;
    }
}
