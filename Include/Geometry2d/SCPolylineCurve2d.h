#pragma once
#include <utility>
#include "Geometry2d/ISCCurve2d.h"
#include "Geometry2d/SCPolyline2d.h"
namespace Geometry
{
    // 拥有路径；每段占单位参数区间，Clone 不借用原路径生命周期。
    class GEOMETRY_API SCPolylineCurve2d final : public ISCCurve2d
    {
    public:
        SCPolylineCurve2d() = default;
        explicit SCPolylineCurve2d(SCPolyline2d path) : path_(std::move(path)) {}
        [[nodiscard]] bool IsValid(double epsilon = kDefaultEpsilon) const override;
        [[nodiscard]] SCIntervald ParameterRange() const override;
        [[nodiscard]] SCCurveEval2d Evaluate(double parameter, int derivativeOrder = 1) const override;
        [[nodiscard]] SCBox2d Bounds() const override;
        [[nodiscard]] std::unique_ptr<ISCCurve2d> Clone() const override;
        [[nodiscard]] std::size_t PieceCount() const override { return path_.SegmentCount(); }
        [[nodiscard]] SCIntervald PieceRange(std::size_t index) const override;
        [[nodiscard]] bool IsLinearPiece(std::size_t index) const override;
        [[nodiscard]] const SCPolyline2d& Polyline() const { return path_; }
    private:
        SCPolyline2d path_{};
    };
}
