#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "sdk/Surface.h"

namespace geometry::sdk
{
class GEOMETRY_API NurbsSurface final : public Surface
{
public:
    NurbsSurface() = default;
    NurbsSurface(
        int degreeU,
        int degreeV,
        std::size_t controlPointCountU,
        std::size_t controlPointCountV,
        std::vector<Point3d> controlPoints,
        std::vector<double> knotsU,
        std::vector<double> knotsV)
        : degreeU_(degreeU),
          degreeV_(degreeV),
          controlPointCountU_(controlPointCountU),
          controlPointCountV_(controlPointCountV),
          controlPoints_(std::move(controlPoints)),
          knotsU_(std::move(knotsU)),
          knotsV_(std::move(knotsV))
    {
    }

    [[nodiscard]] bool IsValid(const GeometryTolerance3d& tolerance = {}) const override
    {
        if (degreeU_ < 1 || degreeV_ < 1 || controlPointCountU_ < 2 || controlPointCountV_ < 2 ||
            controlPoints_.size() != controlPointCountU_ * controlPointCountV_ ||
            knotsU_.size() != controlPointCountU_ + static_cast<std::size_t>(degreeU_) + 1 ||
            knotsV_.size() != controlPointCountV_ + static_cast<std::size_t>(degreeV_) + 1)
        {
            return false;
        }

        for (const Point3d& point : controlPoints_)
        {
            if (!point.IsValid())
            {
                return false;
            }
        }

        for (std::size_t i = 1; i < knotsU_.size(); ++i)
        {
            if (knotsU_[i] + tolerance.parameterEpsilon < knotsU_[i - 1])
            {
                return false;
            }
        }
        for (std::size_t i = 1; i < knotsV_.size(); ++i)
        {
            if (knotsV_[i] + tolerance.parameterEpsilon < knotsV_[i - 1])
            {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] Intervald URange() const override
    {
        return IsValid() ? Intervald{knotsU_[static_cast<std::size_t>(degreeU_)], knotsU_[controlPointCountU_]}
                         : Intervald{};
    }

    [[nodiscard]] Intervald VRange() const override
    {
        return IsValid() ? Intervald{knotsV_[static_cast<std::size_t>(degreeV_)], knotsV_[controlPointCountV_]}
                         : Intervald{};
    }

    [[nodiscard]] Point3d PointAt(double u, double v) const override
    {
        if (!IsValid())
        {
            return {};
        }

        const Intervald uRange = URange();
        const Intervald vRange = VRange();
        const double clampedU = std::clamp(u, uRange.min, uRange.max);
        const double clampedV = std::clamp(v, vRange.min, vRange.max);
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        for (std::size_t iv = 0; iv < controlPointCountV_; ++iv)
        {
            const double basisV = Basis(knotsV_, controlPointCountV_, iv, degreeV_, clampedV);
            for (std::size_t iu = 0; iu < controlPointCountU_; ++iu)
            {
                const double basisU = Basis(knotsU_, controlPointCountU_, iu, degreeU_, clampedU);
                const double weight = basisU * basisV;
                const Point3d& point = controlPoints_[iv * controlPointCountU_ + iu];
                x += point.x * weight;
                y += point.y * weight;
                z += point.z * weight;
            }
        }
        return Point3d{x, y, z};
    }

    [[nodiscard]] SurfaceEval3d Evaluate(double u, double v, int derivativeOrder = 1) const override
    {
        derivativeOrder = std::clamp(derivativeOrder, 0, 1);
        SurfaceEval3d eval{};
        eval.point = PointAt(u, v);
        eval.derivativeOrder = derivativeOrder;
        if (derivativeOrder >= 1)
        {
            const double du = std::max(
                URange().Length() / static_cast<double>(std::max<std::size_t>(controlPointCountU_ * 8, 8)),
                geometry::kDefaultEpsilon);
            const double dv = std::max(
                VRange().Length() / static_cast<double>(std::max<std::size_t>(controlPointCountV_ * 8, 8)),
                geometry::kDefaultEpsilon);
            const double prevU = std::max(URange().min, u - du);
            const double nextU = std::min(URange().max, u + du);
            const double prevV = std::max(VRange().min, v - dv);
            const double nextV = std::min(VRange().max, v + dv);
            eval.derivativeU = (PointAt(nextU, v) - PointAt(prevU, v)) /
                               std::max(nextU - prevU, geometry::kDefaultEpsilon);
            eval.derivativeV = (PointAt(u, nextV) - PointAt(u, prevV)) /
                               std::max(nextV - prevV, geometry::kDefaultEpsilon);
            eval.normal = Cross(eval.derivativeU, eval.derivativeV);
        }
        return eval;
    }

    [[nodiscard]] Box3d Bounds() const override
    {
        Box3d bounds{};
        for (const Point3d& point : controlPoints_)
        {
            bounds.ExpandToInclude(point);
        }
        return bounds;
    }

    [[nodiscard]] std::unique_ptr<Surface> Clone() const override
    {
        return std::make_unique<NurbsSurface>(*this);
    }

    [[nodiscard]] int DegreeU() const
    {
        return degreeU_;
    }

    [[nodiscard]] int DegreeV() const
    {
        return degreeV_;
    }

    [[nodiscard]] std::size_t ControlPointCountU() const
    {
        return controlPointCountU_;
    }

    [[nodiscard]] std::size_t ControlPointCountV() const
    {
        return controlPointCountV_;
    }

    [[nodiscard]] const std::vector<Point3d>& ControlPoints() const
    {
        return controlPoints_;
    }

    [[nodiscard]] const std::vector<double>& KnotsU() const
    {
        return knotsU_;
    }

    [[nodiscard]] const std::vector<double>& KnotsV() const
    {
        return knotsV_;
    }

private:
    [[nodiscard]] static double Basis(
        const std::vector<double>& knots,
        std::size_t pointCount,
        std::size_t index,
        int degree,
        double parameter)
    {
        if (degree == 0)
        {
            const bool inHalfOpen = knots[index] <= parameter && parameter < knots[index + 1];
            const bool isRightBoundary =
                std::abs(parameter - knots[pointCount]) <= geometry::kDefaultEpsilon && index + 1 == pointCount;
            return (inHalfOpen || isRightBoundary) ? 1.0 : 0.0;
        }

        double left = 0.0;
        const double leftDenom = knots[index + static_cast<std::size_t>(degree)] - knots[index];
        if (std::abs(leftDenom) > geometry::kDefaultEpsilon)
        {
            left = ((parameter - knots[index]) / leftDenom) *
                   Basis(knots, pointCount, index, degree - 1, parameter);
        }

        double right = 0.0;
        const double rightDenom =
            knots[index + static_cast<std::size_t>(degree) + 1] - knots[index + 1];
        if (std::abs(rightDenom) > geometry::kDefaultEpsilon)
        {
            right = ((knots[index + static_cast<std::size_t>(degree) + 1] - parameter) / rightDenom) *
                    Basis(knots, pointCount, index + 1, degree - 1, parameter);
        }

        return left + right;
    }

    int degreeU_{1};
    int degreeV_{1};
    std::size_t controlPointCountU_{0};
    std::size_t controlPointCountV_{0};
    std::vector<Point3d> controlPoints_{};
    std::vector<double> knotsU_{0.0, 1.0};
    std::vector<double> knotsV_{0.0, 1.0};
};
} // namespace geometry::sdk
