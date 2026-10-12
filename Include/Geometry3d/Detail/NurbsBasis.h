#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
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
        {
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (activeBasis[i] > 0.0)
                {
                    weightScale = std::max(weightScale, weights[i]);
                    coordinateScale = std::max({coordinateScale, std::abs(points[i].x),
                                                std::abs(points[i].y), std::abs(points[i].z)});
                }
                if (i > 0 && points[i] != points[0]) constantGeometry = false;
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
                if (left > 0.0)
                {
                    levels[order][i] += (u - knots[i]) * levels[order - 1][i] / left;
                }
                if (right > 0.0)
                {
                    levels[order][i] += (knots[i + order + 1] - u) * levels[order - 1][i + 1] / right;
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
                const double a = left > 0.0 ? static_cast<double>(d) / left : 0.0;
                const double b = right > 0.0 ? static_cast<double>(d) / right : 0.0;
                currentFirst[i] = a * levels[order - 1][i] - b * levels[order - 1][i + 1];
                if (d == degree)
                {
                    result.second[i] = a * lowerFirst[i] - b * lowerFirst[i + 1];
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
}  // namespace Geometry::Detail
