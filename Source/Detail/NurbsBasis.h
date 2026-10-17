#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCVector3.h"

namespace Geometry::Detail
{
    struct NurbsWeightedSum
    {
        struct DerivativeTerm
        {
            SCPoint3d point;
            double weight;
            double basis;
        };

        SCVector3d value{}, first{}, second{};
        double weight{0.0}, firstWeight{0.0}, secondWeight{0.0};
        double weightScale{0.0}, coordinateScale{1.0};
        bool constantGeometry{true};
        bool firstTermUnderflow{false};
        std::vector<DerivativeTerm> firstTerms{}, secondTerms{};

        [[nodiscard]] static double ProductRatio(double a, double b, double c, double divisor)
        {
            if (a == 0.0 || b == 0.0 || c == 0.0) return 0.0;
            int ea = 0, eb = 0, ec = 0, ed = 0;
            const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
            const double mc = std::frexp(c, &ec), md = std::frexp(divisor, &ed);
            return std::ldexp((ma * mb * mc) / md, ea + eb + ec - ed);
        }

        [[nodiscard]] static double ProductRatio(double a, double b, double c, double d,
                                                 double divisorA, double divisorB)
        {
            if (a == 0.0 || b == 0.0 || c == 0.0 || d == 0.0) return 0.0;
            int ea = 0, eb = 0, ec = 0, ed = 0, eDivisorA = 0, eDivisorB = 0;
            const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
            const double mc = std::frexp(c, &ec), md = std::frexp(d, &ed);
            const double mDivisorA = std::frexp(divisorA, &eDivisorA);
            const double mDivisorB = std::frexp(divisorB, &eDivisorB);
            return std::ldexp((ma * mb * mc * md) / (mDivisorA * mDivisorB),
                              ea + eb + ec + ed - eDivisorA - eDivisorB);
        }

        NurbsWeightedSum(const std::vector<SCPoint3d>& points, const std::vector<double>& weights,
                         const std::vector<double>& activeBasis)
            : NurbsWeightedSum(points, weights, activeBasis, 0, points.size())
        {
        }

        NurbsWeightedSum(const std::vector<SCPoint3d>& points, const std::vector<double>& weights,
                         const std::vector<double>& activeBasis, std::size_t firstIndex, std::size_t activeCount)
        {
            for (std::size_t i = 0; i < activeCount; ++i)
            {
                const std::size_t index = firstIndex + i;
                if (activeBasis[i] > 0.0)
                {
                    weightScale = std::max(weightScale, weights[index]);
                    coordinateScale = std::max({coordinateScale, std::abs(points[index].x),
                                                std::abs(points[index].y), std::abs(points[index].z)});
                }
                if (i > 0 && points[index] != points[firstIndex]) constantGeometry = false;
            }
        }

        NurbsWeightedSum(const std::vector<SCPoint3d>& points, const std::vector<double>& weights,
                         const std::vector<double>& activeBasis, const std::vector<std::size_t>& activeIndices)
            : NurbsWeightedSum(points, weights, std::span<const double>(activeBasis),
                               std::span<const std::size_t>(activeIndices))
        {
        }

        NurbsWeightedSum(const std::vector<SCPoint3d>& points, const std::vector<double>& weights,
                         std::span<const double> activeBasis, std::span<const std::size_t> activeIndices)
        {
            for (std::size_t i = 0; i < activeIndices.size(); ++i)
            {
                const std::size_t index = activeIndices[i];
                if (activeBasis[i] > 0.0)
                {
                    weightScale = std::max(weightScale, weights[index]);
                    coordinateScale = std::max({coordinateScale, std::abs(points[index].x),
                                                std::abs(points[index].y), std::abs(points[index].z)});
                }
                if (i > 0 && points[index] != points[activeIndices.front()]) constantGeometry = false;
            }
        }

        void Add(const SCPoint3d& point, double weightValue, double basis,
                 double basisFirst = 0.0, double basisSecond = 0.0)
        {
            if (basis == 0.0 && basisFirst == 0.0 && basisSecond == 0.0)
            {
                return;
            }
            const double scaledWeight = weightValue / weightScale;
            const double c = basis == 0.0 ? 0.0 : scaledWeight * basis;
            const double c1 = basisFirst == 0.0 ? 0.0 : scaledWeight * basisFirst;
            const double c2 = basisSecond == 0.0 ? 0.0 : scaledWeight * basisSecond;
            if (basisFirst != 0.0 && c1 == 0.0) firstTermUnderflow = true;
            const SCVector3d scaledPoint{point.x / coordinateScale, point.y / coordinateScale,
                                         point.z / coordinateScale};
            value = value + scaledPoint * c;
            first = first + scaledPoint * c1;
            second = second + scaledPoint * c2;
            weight += c;
            firstWeight += c1;
            secondWeight += c2;
            if (basisFirst != 0.0) firstTerms.push_back({point, weightValue, basisFirst});
            if (basisSecond != 0.0) secondTerms.push_back({point, weightValue, basisSecond});
        }

        [[nodiscard]] SCVector3d Point() const { return (value / weight) * coordinateScale; }
        [[nodiscard]] SCVector3d First() const
        {
            if (constantGeometry) return {};
            const SCVector3d normalizedPoint = value / weight;
            const SCVector3d weightedPointDerivative = normalizedPoint * firstWeight;
            const SCVector3d numerator = first - weightedPointDerivative;
            const auto cancellation = [](double a, double b, double difference) {
                const double scale = std::max(std::abs(a), std::abs(b));
                return scale > 0.0 && std::abs(difference) <=
                    8.0 * std::numeric_limits<double>::epsilon() * scale;
            };
            if (firstTermUnderflow || !std::isfinite(firstWeight) || !first.IsValid() ||
                !weightedPointDerivative.IsValid() || !numerator.IsValid() ||
                cancellation(first.x, weightedPointDerivative.x, numerator.x) ||
                cancellation(first.y, weightedPointDerivative.y, numerator.y) ||
                cancellation(first.z, weightedPointDerivative.z, numerator.z))
            {
                SCVector3d centered{};
                if (!firstTermUnderflow)
                {
                    for (const DerivativeTerm& term : firstTerms)
                    {
                        centered.x += ProductRatio(term.weight, term.basis,
                            term.point.x / coordinateScale - normalizedPoint.x, weightScale);
                        centered.y += ProductRatio(term.weight, term.basis,
                            term.point.y / coordinateScale - normalizedPoint.y, weightScale);
                        centered.z += ProductRatio(term.weight, term.basis,
                            term.point.z / coordinateScale - normalizedPoint.z, weightScale);
                    }
                    return {ProductRatio(centered.x, coordinateScale, 1.0, weight),
                            ProductRatio(centered.y, coordinateScale, 1.0, weight),
                            ProductRatio(centered.z, coordinateScale, 1.0, weight)};
                }
                for (const DerivativeTerm& term : firstTerms)
                {
                    centered.x += ProductRatio(term.weight, term.basis,
                        term.point.x / coordinateScale - normalizedPoint.x, coordinateScale, weightScale, weight);
                    centered.y += ProductRatio(term.weight, term.basis,
                        term.point.y / coordinateScale - normalizedPoint.y, coordinateScale, weightScale, weight);
                    centered.z += ProductRatio(term.weight, term.basis,
                        term.point.z / coordinateScale - normalizedPoint.z, coordinateScale, weightScale, weight);
                }
                return centered;
            }
            return (numerator / weight) * coordinateScale;
        }
        [[nodiscard]] SCVector3d Second(const SCVector3d& firstDerivative) const
        {
            if (constantGeometry) return {};
            const SCVector3d normalizedPoint = value / weight;
            const SCVector3d firstContribution = (firstDerivative / coordinateScale) * (2.0 * firstWeight);
            const SCVector3d pointContribution = normalizedPoint * secondWeight;
            const SCVector3d numerator = second - firstContribution - pointContribution;
            const SCVector3d result = (numerator / weight) * coordinateScale;
            const auto cancellation = [](double a, double b, double c, double difference) {
                const double scale = std::max({std::abs(a), std::abs(b), std::abs(c)});
                return scale > 0.0 && std::abs(difference) <=
                    8.0 * std::numeric_limits<double>::epsilon() * scale;
            };
            if (result.IsValid() &&
                !cancellation(second.x, firstContribution.x, pointContribution.x, numerator.x) &&
                !cancellation(second.y, firstContribution.y, pointContribution.y, numerator.y) &&
                !cancellation(second.z, firstContribution.z, pointContribution.z, numerator.z))
            {
                return result;
            }

            SCVector3d centered{};
            for (const DerivativeTerm& term : secondTerms)
            {
                centered.x += ProductRatio(term.weight, term.basis,
                    term.point.x / coordinateScale - normalizedPoint.x, coordinateScale, weightScale, weight);
                centered.y += ProductRatio(term.weight, term.basis,
                    term.point.y / coordinateScale - normalizedPoint.y, coordinateScale, weightScale, weight);
                centered.z += ProductRatio(term.weight, term.basis,
                    term.point.z / coordinateScale - normalizedPoint.z, coordinateScale, weightScale, weight);
            }
            for (const DerivativeTerm& term : firstTerms)
            {
                centered.x -= ProductRatio(term.weight, term.basis, firstDerivative.x,
                    2.0, weightScale, weight);
                centered.y -= ProductRatio(term.weight, term.basis, firstDerivative.y,
                    2.0, weightScale, weight);
                centered.z -= ProductRatio(term.weight, term.basis, firstDerivative.z,
                    2.0, weightScale, weight);
            }
            return centered;
        }
    };

    struct NurbsBasis
    {
        std::vector<double> value;
        std::vector<double> first;
        std::vector<double> second;
    };

    // 节点和参数有限，但差值的真实量级可能超过 double；先保留常规路径
    // 的舍入方式，仅在差值溢出时共同缩放，不能把无穷跨度当作零基函数。
    [[nodiscard]] inline double NurbsKnotRatio(double numeratorEnd,double numeratorStart,
                                               double spanEnd,double spanStart)
    {
        const double numerator=numeratorEnd-numeratorStart,span=spanEnd-spanStart;
        if (std::isfinite(numerator) && std::isfinite(span)) return numerator/span;
        const double scale=std::max({std::abs(numeratorEnd),std::abs(numeratorStart),std::abs(spanEnd),std::abs(spanStart)});
        if (!(scale>0) || !std::isfinite(scale)) return std::numeric_limits<double>::quiet_NaN();
        return (numeratorEnd/scale-numeratorStart/scale)/(spanEnd/scale-spanStart/scale);
    }

    [[nodiscard]] inline double NurbsKnotDerivativeScale(int degree,double last,double first)
    {
        const double width=last-first;
        if (std::isfinite(width)) return static_cast<double>(degree)/width;
        const double scale=std::max(std::abs(last),std::abs(first));
        return (static_cast<double>(degree)/scale)/(last/scale-first/scale);
    }

    [[nodiscard]] inline bool ValidKnots(const std::vector<double>& knots, std::size_t count, int degree)
    {
        if (degree < 1 || count <= static_cast<std::size_t>(degree) ||
            knots.size() != count + static_cast<std::size_t>(degree) + 1)
        {
            return false;
        }
        for (std::size_t i = 0; i < knots.size(); ++i)
        {
            if (!std::isfinite(knots[i]) || (i > 0 && knots[i] < knots[i - 1]))
            {
                return false;
            }
        }
        return knots[static_cast<std::size_t>(degree)] < knots[count];
    }

    [[nodiscard]] inline NurbsBasis EvaluateNurbsBasis(const std::vector<double>& knots,
                                                       std::size_t count,
                                                       int degree,
                                                       double parameter)
    {
        const double u = std::clamp(parameter, knots[static_cast<std::size_t>(degree)], knots[count]);
        const std::size_t span = u == knots[count]
                                     ? static_cast<std::size_t>(std::lower_bound(
                                           knots.begin(), knots.begin() + static_cast<std::ptrdiff_t>(count + 1), u) -
                                           knots.begin() - 1)
                                     : static_cast<std::size_t>(std::upper_bound(knots.begin(), knots.end(), u) - knots.begin() - 1);
        std::vector<std::vector<double>> levels(static_cast<std::size_t>(degree) + 1,
                                                std::vector<double>(knots.size() - 1, 0.0));
        levels[0][span] = 1.0;
        for (int d = 1; d <= degree; ++d)
        {
            for (std::size_t i = 0; i < count + static_cast<std::size_t>(degree - d); ++i)
            {
                const std::size_t order = static_cast<std::size_t>(d);
                const double left = knots[i + order] - knots[i];
                const double right = knots[i + order + 1] - knots[i + 1];
                if (left > 0.0 && levels[order-1][i]!=0.0)
                {
                    levels[order][i] += NurbsKnotRatio(u,knots[i],knots[i+order],knots[i]) * levels[order - 1][i];
                }
                if (right > 0.0 && levels[order-1][i+1]!=0.0)
                {
                    levels[order][i] += NurbsKnotRatio(knots[i+order+1],u,knots[i+order+1],knots[i+1]) * levels[order - 1][i + 1];
                }
            }
        }

        NurbsBasis result{std::vector<double>(count), std::vector<double>(count), std::vector<double>(count)};
        std::vector<double> lowerFirst(knots.size() - 1, 0.0);
        for (int d = 1; d <= degree; ++d)
        {
            std::vector<double> currentFirst(knots.size() - 1, 0.0);
            for (std::size_t i = 0; i < count + static_cast<std::size_t>(degree - d); ++i)
            {
                const std::size_t order = static_cast<std::size_t>(d);
                const double left = knots[i + order] - knots[i];
                const double right = knots[i + order + 1] - knots[i + 1];
                const double a = left > 0.0 ? NurbsKnotDerivativeScale(d,knots[i+order],knots[i]) : 0.0;
                const double b = right > 0.0 ? NurbsKnotDerivativeScale(d,knots[i+order+1],knots[i+1]) : 0.0;
                currentFirst[i] = (levels[order-1][i]==0?0:a*levels[order-1][i]) -
                    (levels[order-1][i+1]==0?0:b*levels[order-1][i+1]);
                if (d == degree)
                {
                    result.second[i] = (lowerFirst[i]==0?0:a*lowerFirst[i]) - (lowerFirst[i+1]==0?0:b*lowerFirst[i+1]);
                }
            }
            lowerFirst = std::move(currentFirst);
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            result.value[i] = levels[static_cast<std::size_t>(degree)][i];
            result.first[i] = lowerFirst[i];
        }
        return result;
    }

    struct LocalNurbsBasis
    {
        std::size_t firstControlIndex{0};
        std::vector<double> value;
        std::vector<double> first;
        std::vector<double> second;
    };

    [[nodiscard]] inline std::size_t FindValidatedNurbsSpan(const std::vector<double>& knots,
                                                           std::size_t count, double parameter)
    {
        const double* data = knots.data();
        const bool end = parameter == data[count];
        std::size_t first = 0, last = end ? count + 1 : knots.size();
        while (first < last) {
            const std::size_t middle = first + (last-first)/2;
            if (end ? data[middle] < parameter : data[middle] <= parameter) first = middle+1;
            else last = middle;
        }
        return first-1;
    }

    [[nodiscard]] inline std::size_t EvaluateValidatedLinearNurbsBasis(const std::vector<double>& knots,
        std::size_t count,double parameter,double* values)
    {
        const double t=std::clamp(parameter,knots[1],knots[count]);
        const auto span=FindValidatedNurbsSpan(knots,count,t);
        values[0]=NurbsKnotRatio(knots[span+1],t,knots[span+1],knots[span]);
        values[1]=NurbsKnotRatio(t,knots[span],knots[span+1],knots[span]);
        return span;
    }

    [[nodiscard]] inline LocalNurbsBasis EvaluateLocalNurbsBasis(const std::vector<double>& knots,
                                                                std::size_t count,
                                                                int degree,
                                                                double parameter)
    {
        const double u = std::clamp(parameter, knots[static_cast<std::size_t>(degree)], knots[count]);
        const std::size_t span = FindValidatedNurbsSpan(knots, count, u);
        const std::size_t order = static_cast<std::size_t>(degree);
        const std::size_t first = span - order;
        std::vector<std::vector<double>> levels(order + 1, std::vector<double>(order + 2, 0.0));
        levels[0][order] = 1.0;

        LocalNurbsBasis result{first, std::vector<double>(order + 1),
                              std::vector<double>(order + 1), std::vector<double>(order + 1)};
        std::vector<double> lowerFirst(order + 2, 0.0);
        for (int d = 1; d <= degree; ++d)
        {
            const std::size_t currentOrder = static_cast<std::size_t>(d);
            std::vector<double> currentFirst(order + 2, 0.0);
            for (std::size_t j = 0; j <= order; ++j)
            {
                const std::size_t i = first + j;
                const double left = knots[i + currentOrder] - knots[i];
                const double right = knots[i + currentOrder + 1] - knots[i + 1];
                if (left > 0.0 && levels[currentOrder-1][j]!=0.0)
                {
                    levels[currentOrder][j] += NurbsKnotRatio(u,knots[i],knots[i+currentOrder],knots[i]) * levels[currentOrder - 1][j];
                }
                if (right > 0.0 && levels[currentOrder-1][j+1]!=0.0)
                {
                    levels[currentOrder][j] +=
                        NurbsKnotRatio(knots[i+currentOrder+1],u,knots[i+currentOrder+1],knots[i+1]) * levels[currentOrder - 1][j + 1];
                }
                const double a = left > 0.0 ? NurbsKnotDerivativeScale(d,knots[i+currentOrder],knots[i]) : 0.0;
                const double b = right > 0.0 ? NurbsKnotDerivativeScale(d,knots[i+currentOrder+1],knots[i+1]) : 0.0;
                currentFirst[j] = (levels[currentOrder-1][j]==0?0:a*levels[currentOrder-1][j]) -
                    (levels[currentOrder-1][j+1]==0?0:b*levels[currentOrder-1][j+1]);
                if (d == degree)
                {
                    result.second[j] = (lowerFirst[j]==0?0:a*lowerFirst[j]) - (lowerFirst[j+1]==0?0:b*lowerFirst[j+1]);
                }
            }
            lowerFirst = std::move(currentFirst);
        }
        for (std::size_t j = 0; j <= order; ++j)
        {
            result.value[j] = levels[order][j];
            result.first[j] = lowerFirst[j];
        }
        return result;
    }
}  // namespace Geometry::Detail
