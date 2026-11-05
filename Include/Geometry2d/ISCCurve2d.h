#pragma once
#include <memory>
#include <cstddef>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include "Export/GeometryExport.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCVector2.h"
#include "Types/Geometry3d/SCIntervald.h"
#include "Support/Epsilon.h"
namespace Geometry
{
    struct SCCurveEval2d
    {
        SCPoint2d point{};
        SCVector2d firstDerivative{};
        int derivativeOrder{0};
    };
    class GEOMETRY_API ISCCurve2d
    {
    public:
        virtual ~ISCCurve2d() = default;
        [[nodiscard]] virtual bool IsValid(double epsilon = kDefaultEpsilon) const = 0;
        [[nodiscard]] virtual SCIntervald ParameterRange() const = 0;
        [[nodiscard]] virtual SCCurveEval2d Evaluate(double parameter, int derivativeOrder = 1) const = 0;
        [[nodiscard]] virtual SCBox2d Bounds() const = 0;
        [[nodiscard]] virtual std::unique_ptr<ISCCurve2d> Clone() const = 0;
        // 分片区间按原参数排列；默认单片，非线性实现不得声明为直线。
        [[nodiscard]] virtual std::size_t PieceCount() const { return 1; }
        [[nodiscard]] virtual SCIntervald PieceRange(std::size_t index) const
        {
            if (index != 0) throw std::out_of_range("ISCCurve2d::PieceRange");
            return ParameterRange();
        }
        [[nodiscard]] virtual bool IsLinearPiece(std::size_t) const { return false; }
        [[nodiscard]] double ParameterAtPiece(std::size_t index, double local) const
        {
            const auto range = PieceRange(index);
            return std::lerp(range.min, range.max, local);
        }
        [[nodiscard]] SCPoint2d PointAt(double parameter) const { return Evaluate(parameter, 0).point; }
    };
}
