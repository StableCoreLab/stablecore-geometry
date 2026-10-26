#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <vector>

#include "Core/Intersection.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "../../Detail/NurbsBasis.h"
#include "../../Detail/NurbsRootLimits.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline bool SameSurfaceParameterization(const ISCSurface& first, const ISCSurface& second)
    {
        if (&first == &second)
        {
            return true;
        }
        if (const auto* firstPlane = dynamic_cast<const SCPlaneSurface*>(&first))
        {
            const auto* secondPlane = dynamic_cast<const SCPlaneSurface*>(&second);
            if (secondPlane == nullptr)
            {
                return false;
            }
            const SCPoint3d firstOrigin = firstPlane->PointAt(0.0, 0.0);
            const SCPoint3d secondOrigin = secondPlane->PointAt(0.0, 0.0);
            const SCVector3d firstU = firstPlane->UAxis();
            const SCVector3d secondU = secondPlane->UAxis();
            const SCVector3d firstV = firstPlane->VAxis();
            const SCVector3d secondV = secondPlane->VAxis();
            return firstOrigin.x == secondOrigin.x && firstOrigin.y == secondOrigin.y &&
                   firstOrigin.z == secondOrigin.z &&
                   firstU.x == secondU.x && firstU.y == secondU.y && firstU.z == secondU.z &&
                   firstV.x == secondV.x && firstV.y == secondV.y && firstV.z == secondV.z;
        }
        if (const auto* firstNurbs = dynamic_cast<const SCNurbsSurface*>(&first))
        {
            const auto* secondNurbs = dynamic_cast<const SCNurbsSurface*>(&second);
            if (secondNurbs == nullptr || firstNurbs->DegreeU() != secondNurbs->DegreeU() ||
                firstNurbs->DegreeV() != secondNurbs->DegreeV() ||
                firstNurbs->ControlPointCountU() != secondNurbs->ControlPointCountU() ||
                firstNurbs->ControlPointCountV() != secondNurbs->ControlPointCountV() ||
                firstNurbs->KnotsU() != secondNurbs->KnotsU() ||
                firstNurbs->KnotsV() != secondNurbs->KnotsV() ||
                firstNurbs->Weights() != secondNurbs->Weights())
            {
                return false;
            }
            const auto& firstPoints = firstNurbs->ControlPoints();
            const auto& secondPoints = secondNurbs->ControlPoints();
            return firstPoints.size() == secondPoints.size() &&
                   std::equal(firstPoints.begin(), firstPoints.end(), secondPoints.begin(),
                              [](const SCPoint3d& lhs, const SCPoint3d& rhs) {
                                  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
                              });
        }
        return false;
    }

    [[nodiscard]] inline bool CurveOnSurfaceUvInsideSupportDomain(const SCCurveOnSurface& curve)
    {
        const ISCSurface* surface = curve.SupportSurface();
        if (surface == nullptr)
        {
            return false;
        }
        if (dynamic_cast<const SCPlaneSurface*>(surface) != nullptr)
        {
            return true;
        }
        const SCIntervald uRange = surface->URange();
        const SCIntervald vRange = surface->VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return false;
        }
        for (std::size_t index = 0; index < curve.PointCount(); ++index)
        {
            const SCPoint2d uv = curve.UvPointAt(index);
            if (!uv.IsValid() || !uRange.Contains(uv.x, 0.0) || !vRange.Contains(uv.y, 0.0))
            {
                return false;
            }
        }
        return true;
    }

    struct CurveOnSurfaceSample
    {
        bool valid{false};
        double parameter{0.0};
        SCPoint2d uv{};
        SCPoint3d point{};
    };

    [[nodiscard]] inline CurveOnSurfaceSample EvaluateCurveOnSurfaceSegment(
        const SCCurveOnSurface& curve, std::size_t segmentIndex, double parameter)
    {
        CurveOnSurfaceSample result{};
        const std::size_t pointCount = curve.PointCount();
        const ISCSurface* surface = curve.SupportSurface();
        if (surface == nullptr || pointCount < 2 || segmentIndex >= pointCount ||
            !std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0 ||
            (!curve.UvCurve().IsClosed() && segmentIndex + 1 >= pointCount))
        {
            return result;
        }
        result.parameter = parameter;
        result.uv = curve.UvAt(static_cast<double>(segmentIndex) + parameter);
        if (!result.uv.IsValid())
        {
            return result;
        }
        if (dynamic_cast<const SCPlaneSurface*>(surface) == nullptr &&
            (!surface->URange().Contains(result.uv.x, 0.0) ||
             !surface->VRange().Contains(result.uv.y, 0.0)))
        {
            return result;
        }
        result.point = surface->PointAt(result.uv.x, result.uv.y);
        result.valid = result.point.IsValid();
        return result;
    }

    struct RestrictedNurbsSurfaceCurve
    {
        SCNurbsCurve3d curve{};
        double start{0.0};
        double end{0.0};
        double segmentStart{0.0};
        double segmentEnd{1.0};
    };

    struct HomogeneousSurfaceControl
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double w{0.0};
    };

    [[nodiscard]] inline HomogeneousSurfaceControl LerpSurfaceControl(
        const HomogeneousSurfaceControl& first, const HomogeneousSurfaceControl& second, double parameter)
    {
        return {std::lerp(first.x, second.x, parameter),
                std::lerp(first.y, second.y, parameter),
                std::lerp(first.z, second.z, parameter),
                std::lerp(first.w, second.w, parameter)};
    }

    [[nodiscard]] inline std::vector<HomogeneousSurfaceControl> RestrictBezierControls(
        const std::vector<HomogeneousSurfaceControl>& controls, double first, double second)
    {
        if (first > second)
        {
            auto reversed = RestrictBezierControls(controls, second, first);
            std::reverse(reversed.begin(), reversed.end());
            return reversed;
        }

        const auto splitLeft = [](const std::vector<HomogeneousSurfaceControl>& input,
                                  double parameter, bool retainRight) {
            const std::size_t degree = input.size() - 1;
            auto work = input;
            std::vector<HomogeneousSurfaceControl> output(input.size());
            output[retainRight ? degree : 0] = work[retainRight ? degree : 0];
            for (std::size_t level = 1; level <= degree; ++level)
            {
                for (std::size_t index = 0; index <= degree - level; ++index)
                {
                    work[index] = LerpSurfaceControl(work[index], work[index + 1], parameter);
                }
                output[retainRight ? degree - level : level] =
                    work[retainRight ? degree - level : 0];
            }
            return output;
        };

        const auto right = splitLeft(controls, first, true);
        const double relative = first == 1.0 ? 0.0 : (second - first) / (1.0 - first);
        return splitLeft(right, relative, false);
    }

    [[nodiscard]] inline double BinomialCoefficient(int degree, int index)
    {
        double result = 1.0;
        for (int i = 1; i <= index; ++i)
        {
            result *= static_cast<double>(degree - index + i) / static_cast<double>(i);
        }
        return result;
    }

    [[nodiscard]] inline bool TryReduceBezierControlDegree(std::vector<HomogeneousSurfaceControl>& controls)
    {
        const int degree = static_cast<int>(controls.size()) - 1;
        if (degree <= 1)
        {
            return false;
        }
        std::vector<HomogeneousSurfaceControl> reduced(static_cast<std::size_t>(degree));
        reduced.front() = controls.front();
        for (int index = 1; index < degree; ++index)
        {
            const double fraction = static_cast<double>(index) / degree;
            const double inverse = 1.0 / (1.0 - fraction);
            const auto& current = controls[index];
            const auto& previous = reduced[index - 1];
            reduced[index] = {(current.x - fraction * previous.x) * inverse,
                              (current.y - fraction * previous.y) * inverse,
                              (current.z - fraction * previous.z) * inverse,
                              (current.w - fraction * previous.w) * inverse};
        }

        HomogeneousSurfaceControl scale{};
        for (const auto& control : controls)
        {
            scale.x = std::max(scale.x, std::abs(control.x));
            scale.y = std::max(scale.y, std::abs(control.y));
            scale.z = std::max(scale.z, std::abs(control.z));
            scale.w = std::max(scale.w, std::abs(control.w));
        }
        const auto close = [](double actual, double expected, double magnitude) {
            return std::isfinite(expected) &&
                   std::abs(actual - expected) <=
                       64.0 * std::numeric_limits<double>::epsilon() *
                           std::max(magnitude, std::numeric_limits<double>::min());
        };
        for (int index = 0; index <= degree; ++index)
        {
            const HomogeneousSurfaceControl expected =
                index == 0 ? reduced.front() :
                index == degree ? reduced.back() :
                LerpSurfaceControl(reduced[index], reduced[index - 1], static_cast<double>(index) / degree);
            const auto& actual = controls[index];
            if (!close(actual.x, expected.x, scale.x) || !close(actual.y, expected.y, scale.y) ||
                !close(actual.z, expected.z, scale.z) || !close(actual.w, expected.w, scale.w))
            {
                return false;
            }
        }
        controls = std::move(reduced);
        return true;
    }

    [[nodiscard]] inline std::optional<RestrictedNurbsSurfaceCurve> RestrictBezierNurbsSurfaceSegment(
        const SCCurveOnSurface& curve, std::size_t segmentIndex, const SCGeometryTolerance3d& tolerance)
    {
        const auto* surface = dynamic_cast<const SCNurbsSurface*>(curve.SupportSurface());
        if (surface == nullptr || segmentIndex >= curve.UvCurve().SegmentCount())
        {
            return std::nullopt;
        }
        const int degreeU = surface->DegreeU();
        const int degreeV = surface->DegreeV();
        const int degree = degreeU + degreeV;
        if (degree > kMaxNurbsRootDegree ||
            surface->ControlPointCountU() != static_cast<std::size_t>(degreeU + 1) ||
            surface->ControlPointCountV() != static_cast<std::size_t>(degreeV + 1) ||
            !surface->IsValid(tolerance))
        {
            return std::nullopt;
        }

        const auto& knotsU = surface->KnotsU();
        const auto& knotsV = surface->KnotsV();
        const auto clampedBezier = [](const std::vector<double>& knots, int axisDegree) {
            return knots.size() == static_cast<std::size_t>(2 * (axisDegree + 1)) &&
                   std::all_of(knots.begin(), knots.begin() + axisDegree + 1,
                               [&](double knot) { return knot == knots.front(); }) &&
                   std::all_of(knots.begin() + axisDegree + 1, knots.end(),
                               [&](double knot) { return knot == knots.back(); }) &&
                   knots.front() < knots.back();
        };
        if (!clampedBezier(knotsU, degreeU) || !clampedBezier(knotsV, degreeV))
        {
            return std::nullopt;
        }

        const SCPoint2d first = curve.UvPointAt(segmentIndex);
        const SCPoint2d second = curve.UvPointAt((segmentIndex + 1) % curve.PointCount());
        const double firstU = (first.x - knotsU.front()) / (knotsU.back() - knotsU.front());
        const double secondU = (second.x - knotsU.front()) / (knotsU.back() - knotsU.front());
        const double firstV = (first.y - knotsV.front()) / (knotsV.back() - knotsV.front());
        const double secondV = (second.y - knotsV.front()) / (knotsV.back() - knotsV.front());
        if (!std::isfinite(firstU) || !std::isfinite(secondU) ||
            !std::isfinite(firstV) || !std::isfinite(secondV) ||
            firstU < 0.0 || firstU > 1.0 || secondU < 0.0 || secondU > 1.0 ||
            firstV < 0.0 || firstV > 1.0 || secondV < 0.0 || secondV > 1.0)
        {
            return std::nullopt;
        }

        const auto& points = surface->ControlPoints();
        const auto& surfaceWeights = surface->Weights();
        const double weightScale = *std::max_element(surfaceWeights.begin(), surfaceWeights.end());
        if (!(weightScale > 0.0) || !std::isfinite(weightScale))
        {
            return std::nullopt;
        }

        std::vector<std::vector<HomogeneousSurfaceControl>> uRestricted(degreeV + 1);
        for (int v = 0; v <= degreeV; ++v)
        {
            std::vector<HomogeneousSurfaceControl> row;
            row.reserve(degreeU + 1);
            for (int u = 0; u <= degreeU; ++u)
            {
                const std::size_t index = static_cast<std::size_t>(v * (degreeU + 1) + u);
                const double weight = surfaceWeights[index] / weightScale;
                row.push_back({points[index].x * weight, points[index].y * weight,
                               points[index].z * weight, weight});
            }
            uRestricted[v] = RestrictBezierControls(row, firstU, secondU);
        }

        std::vector<std::vector<HomogeneousSurfaceControl>> restricted(degreeU + 1);
        for (int u = 0; u <= degreeU; ++u)
        {
            std::vector<HomogeneousSurfaceControl> column;
            column.reserve(degreeV + 1);
            for (int v = 0; v <= degreeV; ++v)
            {
                column.push_back(uRestricted[v][u]);
            }
            restricted[u] = RestrictBezierControls(column, firstV, secondV);
        }

        std::vector<HomogeneousSurfaceControl> diagonalControls;
        diagonalControls.reserve(degree + 1);
        for (int index = 0; index <= degree; ++index)
        {
            HomogeneousSurfaceControl control{};
            const double denominator = BinomialCoefficient(degree, index);
            for (int u = std::max(0, index - degreeV); u <= std::min(degreeU, index); ++u)
            {
                const int v = index - u;
                const double factor =
                    BinomialCoefficient(degreeU, u) * BinomialCoefficient(degreeV, v) / denominator;
                const auto& term = restricted[u][v];
                control.x += factor * term.x;
                control.y += factor * term.y;
                control.z += factor * term.z;
                control.w += factor * term.w;
            }
            if (!(control.w > 0.0) || !std::isfinite(control.w) ||
                !std::isfinite(control.x) || !std::isfinite(control.y) || !std::isfinite(control.z))
            {
                return std::nullopt;
            }
            diagonalControls.push_back(control);
        }
        while (TryReduceBezierControlDegree(diagonalControls))
        {
            // 消除张量积升阶产生的冗余次数，保留等价的有理控制多项式。
        }

        const int restrictedDegree = static_cast<int>(diagonalControls.size()) - 1;
        std::vector<SCPoint3d> controls;
        std::vector<double> weights;
        controls.reserve(diagonalControls.size());
        weights.reserve(diagonalControls.size());
        for (const auto& control : diagonalControls)
        {
            const SCPoint3d point{control.x / control.w, control.y / control.w, control.z / control.w};
            if (!point.IsValid())
            {
                return std::nullopt;
            }
            controls.push_back(point);
            weights.push_back(control.w);
        }

        std::vector<double> knots(static_cast<std::size_t>(2 * (restrictedDegree + 1)), 1.0);
        std::fill(knots.begin(), knots.begin() + restrictedDegree + 1, 0.0);
        SCNurbsCurve3d restrictedCurve(restrictedDegree, std::move(controls), std::move(knots), std::move(weights));
        if (!restrictedCurve.IsValid(tolerance))
        {
            return std::nullopt;
        }
        return RestrictedNurbsSurfaceCurve{std::move(restrictedCurve), 0.0, 1.0};
    }

    [[nodiscard]] inline std::optional<RestrictedNurbsSurfaceCurve> RestrictAxisAlignedNurbsSurfaceSegment(
        const SCCurveOnSurface& curve, std::size_t segmentIndex, const SCGeometryTolerance3d& tolerance)
    {
        const auto* surface = dynamic_cast<const SCNurbsSurface*>(curve.SupportSurface());
        if (surface == nullptr || !surface->IsValid(tolerance) ||
            segmentIndex >= curve.UvCurve().SegmentCount())
        {
            return std::nullopt;
        }
        const SCPoint2d first = curve.UvPointAt(segmentIndex);
        const SCPoint2d second = curve.UvPointAt((segmentIndex + 1) % curve.PointCount());
        const bool alongU = first.y == second.y && first.x != second.x;
        const bool alongV = first.x == second.x && first.y != second.y;
        if (!alongU && !alongV)
        {
            return std::nullopt;
        }

        const std::size_t countU = surface->ControlPointCountU();
        const std::size_t countV = surface->ControlPointCountV();
        const std::size_t movingCount = alongU ? countU : countV;
        const std::size_t fixedCount = alongU ? countV : countU;
        const int fixedDegree = alongU ? surface->DegreeV() : surface->DegreeU();
        const double fixedParameter = alongU ? first.y : first.x;
        const auto& fixedKnots = alongU ? surface->KnotsV() : surface->KnotsU();
        const auto basis = EvaluateNurbsBasis(fixedKnots, fixedCount, fixedDegree, fixedParameter).value;
        const auto& surfacePoints = surface->ControlPoints();
        const auto& surfaceWeights = surface->Weights();
        const double weightScale = *std::max_element(surfaceWeights.begin(), surfaceWeights.end());
        if (!(weightScale > 0.0) || !std::isfinite(weightScale))
        {
            return std::nullopt;
        }

        std::vector<SCPoint3d> controls;
        std::vector<double> weights;
        controls.reserve(movingCount);
        weights.reserve(movingCount);
        for (std::size_t moving = 0; moving < movingCount; ++moving)
        {
            double weight = 0.0;
            SCVector3d weightedPoint{};
            for (std::size_t fixed = 0; fixed < fixedCount; ++fixed)
            {
                if (!(basis[fixed] > 0.0))
                {
                    continue;
                }
                const std::size_t index = alongU ? fixed * countU + moving : moving * countU + fixed;
                const double contribution = basis[fixed] * (surfaceWeights[index] / weightScale);
                weight += contribution;
                weightedPoint = weightedPoint + SCVector3d{surfacePoints[index].x, surfacePoints[index].y,
                                                            surfacePoints[index].z} * contribution;
            }
            if (!(weight > 0.0) || !std::isfinite(weight))
            {
                return std::nullopt;
            }
            controls.push_back({weightedPoint.x / weight, weightedPoint.y / weight, weightedPoint.z / weight});
            weights.push_back(weight);
        }

        SCNurbsCurve3d restricted(alongU ? surface->DegreeU() : surface->DegreeV(),
                                  std::move(controls),
                                  alongU ? surface->KnotsU() : surface->KnotsV(),
                                  std::move(weights));
        if (!restricted.IsValid(tolerance))
        {
            return std::nullopt;
        }
        return RestrictedNurbsSurfaceCurve{std::move(restricted),
                                            alongU ? first.x : first.y,
                                            alongU ? second.x : second.y};
    }

    struct RefinedNurbsSurfaceNet
    {
        std::size_t countU{0};
        std::size_t countV{0};
        std::vector<HomogeneousSurfaceControl> controls;
        std::vector<double> knotsU;
        std::vector<double> knotsV;
    };

    [[nodiscard]] inline std::vector<HomogeneousSurfaceControl> InsertNurbsControlKnot(
        const std::vector<HomogeneousSurfaceControl>& controls,
        const std::vector<double>& knots, int degree, double knot)
    {
        const std::size_t span = static_cast<std::size_t>(
            std::upper_bound(knots.begin(), knots.end(), knot) - knots.begin() - 1);
        const std::size_t multiplicity = static_cast<std::size_t>(std::count(knots.begin(), knots.end(), knot));
        std::vector<HomogeneousSurfaceControl> inserted(controls.size() + 1);
        const std::size_t firstBlend = span - static_cast<std::size_t>(degree) + 1;
        const std::size_t lastBlend = span - multiplicity;
        for (std::size_t index = 0; index < firstBlend; ++index)
        {
            inserted[index] = controls[index];
        }
        for (std::size_t index = lastBlend; index < controls.size(); ++index)
        {
            inserted[index + 1] = controls[index];
        }
        for (std::size_t index = firstBlend; index <= lastBlend; ++index)
        {
            const double denominator = knots[index + static_cast<std::size_t>(degree)] - knots[index];
            const double alpha = (knot - knots[index]) / denominator;
            inserted[index] = LerpSurfaceControl(controls[index - 1], controls[index], alpha);
        }
        return inserted;
    }

    inline void RefineNurbsSurfaceAxis(RefinedNurbsSurfaceNet& net, int degree, bool alongU)
    {
        auto& knots = alongU ? net.knotsU : net.knotsV;
        const std::size_t count = alongU ? net.countU : net.countV;
        const std::vector<double> originalKnots = knots;
        for (std::size_t knotIndex = static_cast<std::size_t>(degree);
             knotIndex <= count; ++knotIndex)
        {
            const double knot = originalKnots[knotIndex];
            if (knotIndex > static_cast<std::size_t>(degree) &&
                knot == originalKnots[knotIndex - 1])
            {
                continue;
            }
            while (std::count(knots.begin(), knots.end(), knot) < degree)
            {
                const std::size_t span = static_cast<std::size_t>(
                    std::upper_bound(knots.begin(), knots.end(), knot) - knots.begin() - 1);
                std::vector<HomogeneousSurfaceControl> inserted;
                inserted.reserve(net.controls.size() + (alongU ? net.countV : net.countU));
                if (alongU)
                {
                    for (std::size_t v = 0; v < net.countV; ++v)
                    {
                        const auto begin = net.controls.begin() + v * net.countU;
                        const std::vector<HomogeneousSurfaceControl> row(begin, begin + net.countU);
                        const auto refined = InsertNurbsControlKnot(row, knots, degree, knot);
                        inserted.insert(inserted.end(), refined.begin(), refined.end());
                    }
                    ++net.countU;
                }
                else
                {
                    inserted.resize(net.countU * (net.countV + 1));
                    for (std::size_t u = 0; u < net.countU; ++u)
                    {
                        std::vector<HomogeneousSurfaceControl> column;
                        column.reserve(net.countV);
                        for (std::size_t v = 0; v < net.countV; ++v)
                        {
                            column.push_back(net.controls[v * net.countU + u]);
                        }
                        const auto refined = InsertNurbsControlKnot(column, knots, degree, knot);
                        for (std::size_t v = 0; v <= net.countV; ++v)
                        {
                            inserted[v * net.countU + u] = refined[v];
                        }
                    }
                    ++net.countV;
                }
                net.controls = std::move(inserted);
                knots.insert(knots.begin() + static_cast<std::ptrdiff_t>(span + 1), knot);
            }
        }
    }

    [[nodiscard]] inline std::vector<RestrictedNurbsSurfaceCurve> RestrictNurbsSurfaceSegmentPieces(
        const SCCurveOnSurface& curve, std::size_t segmentIndex, const SCGeometryTolerance3d& tolerance)
    {
        if (auto axis = RestrictAxisAlignedNurbsSurfaceSegment(curve, segmentIndex, tolerance))
        {
            const auto& knots = axis->curve.Knots();
            const auto range = axis->curve.ParameterRange();
            const int degree = axis->curve.Degree();
            if (std::count(knots.begin(), knots.end(), range.min) >= degree + 1 &&
                std::count(knots.begin(), knots.end(), range.max) >= degree + 1)
            {
                std::vector<RestrictedNurbsSurfaceCurve> result;
                result.push_back(std::move(*axis));
                return result;
            }
        }
        if (auto bezier = RestrictBezierNurbsSurfaceSegment(curve, segmentIndex, tolerance))
        {
            std::vector<RestrictedNurbsSurfaceCurve> result;
            result.push_back(std::move(*bezier));
            return result;
        }

        const auto* surface = dynamic_cast<const SCNurbsSurface*>(curve.SupportSurface());
        if (surface == nullptr || segmentIndex >= curve.UvCurve().SegmentCount() ||
            !surface->IsValid(tolerance) ||
            surface->DegreeU() + surface->DegreeV() > kMaxNurbsRootDegree)
        {
            return {};
        }
        const auto& originalU = surface->KnotsU();
        const auto& originalV = surface->KnotsV();

        const SCPoint2d first = curve.UvPointAt(segmentIndex);
        const SCPoint2d second = curve.UvPointAt((segmentIndex + 1) % curve.PointCount());
        const auto inside = [](double value, const std::vector<double>& knots,
                               int degree, std::size_t count) {
            return std::isfinite(value) && value >= knots[static_cast<std::size_t>(degree)] &&
                   value <= knots[count];
        };
        if (!inside(first.x, originalU, surface->DegreeU(), surface->ControlPointCountU()) ||
            !inside(second.x, originalU, surface->DegreeU(), surface->ControlPointCountU()) ||
            !inside(first.y, originalV, surface->DegreeV(), surface->ControlPointCountV()) ||
            !inside(second.y, originalV, surface->DegreeV(), surface->ControlPointCountV()))
        {
            return {};
        }

        RefinedNurbsSurfaceNet net{surface->ControlPointCountU(), surface->ControlPointCountV(),
                                   {}, originalU, originalV};
        const auto& points = surface->ControlPoints();
        const auto& weights = surface->Weights();
        const double weightScale = *std::max_element(weights.begin(), weights.end());
        if (!(weightScale > 0.0) || !std::isfinite(weightScale))
        {
            return {};
        }
        net.controls.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index)
        {
            const double weight = weights[index] / weightScale;
            net.controls.push_back({points[index].x * weight, points[index].y * weight,
                                    points[index].z * weight, weight});
        }
        RefineNurbsSurfaceAxis(net, surface->DegreeU(), true);
        RefineNurbsSurfaceAxis(net, surface->DegreeV(), false);

        std::vector<double> breaks{0.0, 1.0};
        const auto addBreaks = [&](const std::vector<double>& knots, double start, double end) {
            if (start == end) return;
            for (double knot : knots)
            {
                const double parameter = (knot - start) / (end - start);
                if (parameter > 0.0 && parameter < 1.0)
                {
                    breaks.push_back(parameter);
                }
            }
        };
        addBreaks(net.knotsU, first.x, second.x);
        addBreaks(net.knotsV, first.y, second.y);
        std::sort(breaks.begin(), breaks.end());
        breaks.erase(std::unique(breaks.begin(), breaks.end()), breaks.end());

        std::vector<RestrictedNurbsSurfaceCurve> pieces;
        for (std::size_t index = 1; index < breaks.size(); ++index)
        {
            const double start = breaks[index - 1];
            const double end = breaks[index];
            const double middle = std::midpoint(start, end);
            const SCPoint2d middleUv{std::lerp(first.x, second.x, middle),
                                     std::lerp(first.y, second.y, middle)};
            const auto spanAt = [](const std::vector<double>& knots, double value) {
                return static_cast<std::size_t>(
                    std::upper_bound(knots.begin(), knots.end(), value) - knots.begin() - 1);
            };
            const std::size_t spanU = spanAt(net.knotsU, middleUv.x);
            const std::size_t spanV = spanAt(net.knotsV, middleUv.y);
            const int degreeU = surface->DegreeU();
            const int degreeV = surface->DegreeV();
            if (spanU < static_cast<std::size_t>(degreeU) ||
                spanV < static_cast<std::size_t>(degreeV) ||
                spanU + 1 >= net.knotsU.size() || spanV + 1 >= net.knotsV.size())
            {
                return {};
            }
            const double lowU = net.knotsU[spanU];
            const double highU = net.knotsU[spanU + 1];
            const double lowV = net.knotsV[spanV];
            const double highV = net.knotsV[spanV + 1];
            if (!(highU > lowU) || !(highV > lowV)) return {};

            std::vector<SCPoint3d> patchPoints;
            std::vector<double> patchWeights;
            for (int v = 0; v <= degreeV; ++v)
            {
                for (int u = 0; u <= degreeU; ++u)
                {
                    const auto& control = net.controls[
                        (spanV - degreeV + v) * net.countU + spanU - degreeU + u];
                    if (!(control.w > 0.0) || !std::isfinite(control.w)) return {};
                    patchPoints.push_back({control.x / control.w,
                                           control.y / control.w, control.z / control.w});
                    patchWeights.push_back(control.w);
                }
            }
            const auto patchKnots = [](int degree) {
                std::vector<double> knots(static_cast<std::size_t>(2 * (degree + 1)), 1.0);
                std::fill(knots.begin(), knots.begin() + degree + 1, 0.0);
                return knots;
            };
            const auto patch = std::make_shared<SCNurbsSurface>(
                degreeU, degreeV, static_cast<std::size_t>(degreeU + 1),
                static_cast<std::size_t>(degreeV + 1), std::move(patchPoints),
                patchKnots(degreeU), patchKnots(degreeV), std::move(patchWeights));
            const auto localUv = [&](double parameter) {
                return SCPoint2d{
                    std::clamp((std::lerp(first.x, second.x, parameter) - lowU) / (highU - lowU), 0.0, 1.0),
                    std::clamp((std::lerp(first.y, second.y, parameter) - lowV) / (highV - lowV), 0.0, 1.0)};
            };
            const SCCurveOnSurface patchSegment(
                patch, SCPolyline2d({localUv(start), localUv(end)}));
            auto restricted = RestrictBezierNurbsSurfaceSegment(patchSegment, 0, tolerance);
            if (!restricted) return {};
            restricted->segmentStart = start;
            restricted->segmentEnd = end;
            pieces.push_back(std::move(*restricted));
        }
        return pieces;
    }

    template <typename Consider>
    inline void RefineCurveOnSurfaceInterval(double left, double right, Consider&& consider)
    {
        constexpr double golden = 0.3819660112501051;
        double first = std::lerp(left, right, golden);
        double second = std::lerp(left, right, 1.0 - golden);
        double firstDistance = consider(first);
        double secondDistance = consider(second);
        for (int iteration = 0; iteration < 48; ++iteration)
        {
            if (firstDistance <= secondDistance)
            {
                right = second;
                second = first;
                secondDistance = firstDistance;
                first = std::lerp(left, right, golden);
                firstDistance = consider(first);
            }
            else
            {
                left = first;
                first = second;
                firstDistance = secondDistance;
                second = std::lerp(left, right, 1.0 - golden);
                secondDistance = consider(second);
            }
        }
    }

    struct CurveOnSurfaceProjectionCandidate
    {
        bool valid{false};
        CurveOnSurfaceSample sample{};
        double distance{0.0};
        double distanceSquared{0.0};
        bool analyticComplete{false};
    };

    [[nodiscard]] inline CurveOnSurfaceProjectionCandidate ProjectPointToCurveOnSurfaceSegment(
        const SCPoint3d& point, const SCCurveOnSurface& curve, std::size_t segmentIndex)
    {
        CurveOnSurfaceProjectionCandidate best{};
        if (!point.IsValid())
        {
            return best;
        }
        const auto consider = [&](double parameter) {
            const CurveOnSurfaceSample sample = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
            if (!sample.valid)
            {
                return std::numeric_limits<double>::infinity();
            }
            const double distance = std::hypot(point.x - sample.point.x, point.y - sample.point.y,
                                               point.z - sample.point.z);
            const double squared = distance * distance;
            if (std::isfinite(distance) && (!best.valid || distance < best.distance))
            {
                best = {true, sample, distance, squared, false};
            }
            return distance;
        };

        // SCPlaneSurface maps every UV segment to an affine 3D segment.
        if (dynamic_cast<const SCPlaneSurface*>(curve.SupportSurface()) != nullptr)
        {
            const CurveOnSurfaceSample start = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, 0.0);
            const CurveOnSurfaceSample end = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, 1.0);
            if (start.valid && end.valid)
            {
                const SCVector3d chord = end.point - start.point;
                const double length = std::hypot(chord.x, chord.y, chord.z);
                bool hasAnalyticParameter = false;
                const SCVector3d offset = point - start.point;
                if (std::isfinite(length) && length > 0.0 && offset.IsValid())
                {
                    const double projection = Dot(offset, chord / length);
                    if (std::isfinite(projection))
                    {
                        const double parameter = projection <= 0.0 ? 0.0 :
                            projection >= length ? 1.0 : projection / length;
                        hasAnalyticParameter = std::isfinite(consider(parameter));
                    }
                }
                consider(0.0);
                consider(1.0);
                const bool constantSegment = start.uv.x == end.uv.x && start.uv.y == end.uv.y;
                if (best.valid && (hasAnalyticParameter || constantSegment))
                {
                    best.analyticComplete = true;
                    return best;
                }
            }
        }

        constexpr std::size_t sampleCount = 32;
        std::array<double, sampleCount + 1> distances{};
        for (std::size_t i = 0; i <= sampleCount; ++i)
        {
            distances[i] = consider(static_cast<double>(i) / static_cast<double>(sampleCount));
        }
        std::vector<std::size_t> minima;
        for (std::size_t i = 1; i < sampleCount; ++i)
        {
            if (std::isfinite(distances[i]) && distances[i] <= distances[i - 1] &&
                distances[i] <= distances[i + 1] &&
                (distances[i] < distances[i - 1] || distances[i] < distances[i + 1]))
            {
                minima.push_back(i);
            }
        }
        // An interior minimum can lie between an endpoint and its first grid sample.
        RefineCurveOnSurfaceInterval(0.0, 1.0 / sampleCount, consider);
        RefineCurveOnSurfaceInterval((sampleCount - 1.0) / sampleCount, 1.0, consider);
        std::sort(minima.begin(), minima.end(), [&](std::size_t lhs, std::size_t rhs) {
            return distances[lhs] < distances[rhs];
        });
        constexpr std::size_t refinementLimit = 8;
        for (std::size_t index = 0; index < std::min(refinementLimit, minima.size()); ++index)
        {
            const std::size_t seed = minima[index];
            RefineCurveOnSurfaceInterval(static_cast<double>(seed - 1) / sampleCount,
                                         static_cast<double>(seed + 1) / sampleCount, consider);
        }
        return best;
    }

    struct CurveOnSurfaceLineCandidate
    {
        bool valid{false};
        CurveOnSurfaceSample sample{};
        double lineParameter{0.0};
        double distance{0.0};
    };

    [[nodiscard]] inline CurveOnSurfaceLineCandidate ProjectCurveOnSurfaceSegmentToLine(
        const SCLine3d& line, const SCCurveOnSurface& curve, std::size_t segmentIndex,
        std::vector<CurveOnSurfaceLineCandidate>* localMinima = nullptr)
    {
        CurveOnSurfaceLineCandidate best{};
        CurveOnSurfaceLineCandidate intervalBest{};
        const double directionSquared = line.direction.LengthSquared();
        if (!line.origin.IsValid() || !line.direction.IsValid() ||
            !std::isfinite(directionSquared) || !(directionSquared > 0.0))
        {
            return best;
        }
        const auto consider = [&](double parameter) {
            const CurveOnSurfaceSample sample = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
            if (!sample.valid)
            {
                return std::numeric_limits<double>::infinity();
            }
            const double lineParameter = Dot(sample.point - line.origin, line.direction) / directionSquared;
            if (!std::isfinite(lineParameter))
            {
                return std::numeric_limits<double>::infinity();
            }
            const SCPoint3d linePoint = line.PointAt(lineParameter);
            if (!linePoint.IsValid())
            {
                return std::numeric_limits<double>::infinity();
            }
            const double distance = std::hypot(sample.point.x - linePoint.x,
                                               sample.point.y - linePoint.y,
                                               sample.point.z - linePoint.z);
            if (std::isfinite(distance) && (!best.valid || distance < best.distance))
            {
                best = {true, sample, lineParameter, distance};
            }
            if (localMinima != nullptr && std::isfinite(distance) &&
                (!intervalBest.valid || distance < intervalBest.distance))
            {
                intervalBest = {true, sample, lineParameter, distance};
            }
            return distance;
        };
        const auto refineInterval = [&](double left, double right) {
            intervalBest = {};
            consider(left);
            consider(right);
            RefineCurveOnSurfaceInterval(left, right, consider);
            if (localMinima != nullptr && intervalBest.valid)
            {
                localMinima->push_back(intervalBest);
            }
        };

        constexpr std::size_t sampleCount = 32;
        std::array<double, sampleCount + 1> distances{};
        for (std::size_t i = 0; i <= sampleCount; ++i)
        {
            distances[i] = consider(static_cast<double>(i) / sampleCount);
        }
        std::vector<std::size_t> minima;
        for (std::size_t i = 1; i < sampleCount; ++i)
        {
            if (std::isfinite(distances[i]) && distances[i] <= distances[i - 1] &&
                distances[i] <= distances[i + 1])
            {
                minima.push_back(i);
            }
        }
        refineInterval(0.0, 1.0 / sampleCount);
        refineInterval((sampleCount - 1.0) / sampleCount, 1.0);
        std::sort(minima.begin(), minima.end(), [&](std::size_t lhs, std::size_t rhs) {
            return distances[lhs] < distances[rhs];
        });
        for (std::size_t index = 0; index < std::min<std::size_t>(8, minima.size()); ++index)
        {
            const std::size_t seed = minima[index];
            refineInterval(static_cast<double>(seed - 1) / sampleCount,
                           static_cast<double>(seed + 1) / sampleCount);
        }
        return best;
    }

    [[nodiscard]] inline std::vector<CurveOnSurfaceSample> FindPlaneCurveOnSurfaceSegmentHits(
        const SCPlane& plane, const SCCurveOnSurface& curve, std::size_t segmentIndex,
        const SCGeometryTolerance3d& tolerance, bool* nonUnique = nullptr);

    [[nodiscard]] inline std::vector<CurveOnSurfaceLineCandidate> FindLineCurveOnSurfaceSegmentHits(
        const SCLine3d& line, const SCCurveOnSurface& curve, std::size_t segmentIndex,
        const SCGeometryTolerance3d& tolerance, bool* nonUnique = nullptr)
    {
        std::vector<CurveOnSurfaceLineCandidate> candidates;
        const double directionLength = std::hypot(line.direction.x, line.direction.y, line.direction.z);
        if (!std::isfinite(directionLength) || !(directionLength > 0.0))
        {
            return {};
        }
        const SCVector3d unit = line.direction / directionLength;
        const auto distanceToLine = [&](const CurveOnSurfaceSample& sample) {
            const double parameter = Dot(sample.point - line.origin, unit) / directionLength;
            const SCPoint3d linePoint = line.PointAt(parameter);
            if (!std::isfinite(parameter) || !linePoint.IsValid())
            {
                return CurveOnSurfaceLineCandidate{};
            }
            const double distance = std::hypot(sample.point.x - linePoint.x,
                                               sample.point.y - linePoint.y,
                                               sample.point.z - linePoint.z);
            return std::isfinite(distance)
                ? CurveOnSurfaceLineCandidate{true, sample, parameter, distance}
                : CurveOnSurfaceLineCandidate{};
        };
        for (const auto& restricted : RestrictNurbsSurfaceSegmentPieces(curve, segmentIndex, tolerance))
        {
            const auto analytic = IntersectAllLineCurveWithStatus(line, restricted.curve, tolerance);
            if (analytic.state == SCGeometryOperationState::NonUnique)
            {
                if (nonUnique != nullptr) *nonUnique = true;
                return {};
            }
            for (const auto& hit : analytic.hits)
            {
                const double parameter = std::lerp(
                    restricted.segmentStart, restricted.segmentEnd,
                    (hit.curveParameter - restricted.start) / (restricted.end - restricted.start));
                if (!std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0)
                {
                    continue;
                }
                const auto sample = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
                const auto candidate = sample.valid ? distanceToLine(sample) : CurveOnSurfaceLineCandidate{};
                if (candidate.valid)
                {
                    candidates.push_back(candidate);
                }
            }
        }
        (void)ProjectCurveOnSurfaceSegmentToLine(line, curve, segmentIndex, &candidates);
        {
            const SCVector3d axis = std::abs(unit.x) <= std::abs(unit.y) &&
                                            std::abs(unit.x) <= std::abs(unit.z) ? SCVector3d{1, 0, 0} :
                                        std::abs(unit.y) <= std::abs(unit.z) ? SCVector3d{0, 1, 0} :
                                                                               SCVector3d{0, 0, 1};
            const SCVector3d firstNormal = Cross(unit, axis);
            const SCVector3d secondNormal = Cross(unit, firstNormal);
            for (const SCVector3d& normal : {firstNormal, secondNormal})
            {
                const SCPlane plane = SCPlane::FromPointAndNormal(line.origin, normal);
                for (const auto& sample : FindPlaneCurveOnSurfaceSegmentHits(
                         plane, curve, segmentIndex, tolerance))
                {
                    const auto candidate = distanceToLine(sample);
                    if (candidate.valid)
                    {
                        candidates.push_back(candidate);
                    }
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.sample.parameter < rhs.sample.parameter;
        });
        std::vector<CurveOnSurfaceLineCandidate> hits;
        for (const auto& candidate : candidates)
        {
            if (candidate.distance > tolerance.distanceEpsilon)
            {
                continue;
            }
            if (!hits.empty() &&
                std::abs(hits.back().sample.parameter - candidate.sample.parameter) <=
                    std::max(tolerance.parameterEpsilon, 1e-8))
            {
                const auto middle = EvaluateCurveOnSurfaceSegment(
                    curve, segmentIndex,
                    std::lerp(hits.back().sample.parameter, candidate.sample.parameter, 0.5));
                const auto middleCandidate = middle.valid ? distanceToLine(middle) : CurveOnSurfaceLineCandidate{};
                if (middleCandidate.valid && middleCandidate.distance <= tolerance.distanceEpsilon)
                {
                    if (candidate.distance < hits.back().distance)
                    {
                        hits.back() = candidate;
                    }
                    continue;
                }
            }
            hits.push_back(candidate);
        }
        return hits;
    }

    [[nodiscard]] inline std::vector<CurveOnSurfaceSample> FindPlaneCurveOnSurfaceSegmentHits(
        const SCPlane& plane, const SCCurveOnSurface& curve, std::size_t segmentIndex,
        const SCGeometryTolerance3d& tolerance, bool* nonUnique)
    {
        std::vector<CurveOnSurfaceSample> hits;
        for (const auto& restricted : RestrictNurbsSurfaceSegmentPieces(curve, segmentIndex, tolerance))
        {
            const auto analytic = IntersectAllPlaneCurveWithStatus(plane, restricted.curve, tolerance);
            if (analytic.state == SCGeometryOperationState::NonUnique)
            {
                if (nonUnique != nullptr) *nonUnique = true;
                return {};
            }
            for (const auto& hit : analytic.hits)
            {
                const double parameter = std::lerp(
                    restricted.segmentStart, restricted.segmentEnd,
                    (hit.curveParameter - restricted.start) / (restricted.end - restricted.start));
                if (!std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0)
                {
                    continue;
                }
                const auto sample = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
                if (sample.valid &&
                    std::abs(plane.SignedDistanceTo(sample.point, tolerance.distanceEpsilon)) <=
                        tolerance.distanceEpsilon)
                {
                    hits.push_back(sample);
                }
            }
        }
        const auto refineCrossing = [&](double low, double high, double lowDistance) {
            CurveOnSurfaceSample best{};
            double bestDistance = std::numeric_limits<double>::infinity();
            for (int iteration = 0; iteration < 64; ++iteration)
            {
                const double middle = std::lerp(low, high, 0.5);
                if (middle <= low || middle >= high)
                {
                    break;
                }
                const CurveOnSurfaceSample midpoint =
                    EvaluateCurveOnSurfaceSegment(curve, segmentIndex, middle);
                if (!midpoint.valid)
                {
                    break;
                }
                const double middleDistance =
                    plane.SignedDistanceTo(midpoint.point, tolerance.distanceEpsilon);
                if (!std::isfinite(middleDistance))
                {
                    break;
                }
                if (std::abs(middleDistance) < bestDistance)
                {
                    best = midpoint;
                    bestDistance = std::abs(middleDistance);
                }
                if ((lowDistance < 0.0 && middleDistance < 0.0) ||
                    (lowDistance > 0.0 && middleDistance > 0.0))
                {
                    low = middle;
                    lowDistance = middleDistance;
                }
                else
                {
                    high = middle;
                }
            }
            return best.valid && bestDistance <= tolerance.distanceEpsilon
                ? best : CurveOnSurfaceSample{};
        };
        CurveOnSurfaceSample previous{};
        double previousDistance = 0.0;
        constexpr int sampleCount = 32;
        std::array<double, sampleCount + 1> distances{};
        distances.fill(std::numeric_limits<double>::infinity());
        for (int sampleIndex = 0; sampleIndex <= sampleCount; ++sampleIndex)
        {
            const double parameter = static_cast<double>(sampleIndex) / sampleCount;
            const CurveOnSurfaceSample current =
                EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
            if (!current.valid)
            {
                previous = {};
                continue;
            }
            const double distance = plane.SignedDistanceTo(current.point, tolerance.distanceEpsilon);
            if (!std::isfinite(distance))
            {
                previous = {};
                continue;
            }
            distances[sampleIndex] = distance;
            if (std::abs(distance) <= tolerance.distanceEpsilon)
            {
                hits.push_back(current);
            }
            else if (previous.valid && std::abs(previousDistance) > tolerance.distanceEpsilon &&
                ((previousDistance < 0.0 && distance > 0.0) ||
                 (previousDistance > 0.0 && distance < 0.0)))
            {
                const auto root = refineCrossing(previous.parameter, current.parameter, previousDistance);
                if (root.valid)
                {
                    hits.push_back(root);
                }
            }
            previous = current;
            previousDistance = distance;
        }
        for (int i = 0; i <= sampleCount; ++i)
        {
            const int leftIndex = std::max(i - 1, 0);
            const int rightIndex = std::min(i + 1, sampleCount);
            const double leftDistance = distances[leftIndex];
            const double middleDistance = distances[i];
            const double rightDistance = distances[rightIndex];
            if (!std::isfinite(leftDistance) || !std::isfinite(middleDistance) ||
                !std::isfinite(rightDistance) || std::abs(middleDistance) <= tolerance.distanceEpsilon ||
                !((leftDistance > 0.0 && middleDistance > 0.0 && rightDistance > 0.0) ||
                  (leftDistance < 0.0 && middleDistance < 0.0 && rightDistance < 0.0)) ||
                std::abs(middleDistance) > std::abs(leftDistance) ||
                std::abs(middleDistance) > std::abs(rightDistance))
            {
                continue;
            }
            CurveOnSurfaceSample best{};
            double bestValue = std::numeric_limits<double>::infinity();
            const double sign = middleDistance > 0.0 ? 1.0 : -1.0;
            const auto consider = [&](double parameter) {
                const auto sample = EvaluateCurveOnSurfaceSegment(curve, segmentIndex, parameter);
                if (!sample.valid)
                {
                    return std::numeric_limits<double>::infinity();
                }
                const double value =
                    sign * plane.SignedDistanceTo(sample.point, tolerance.distanceEpsilon);
                if (std::isfinite(value) && value < bestValue)
                {
                    best = sample;
                    bestValue = value;
                }
                return value;
            };
            const double left = static_cast<double>(leftIndex) / sampleCount;
            const double right = static_cast<double>(rightIndex) / sampleCount;
            consider(left);
            consider(right);
            RefineCurveOnSurfaceInterval(left, right, consider);
            if (!best.valid)
            {
                continue;
            }
            if (std::abs(bestValue) <= tolerance.distanceEpsilon)
            {
                hits.push_back(best);
            }
            else if (bestValue < -tolerance.distanceEpsilon)
            {
                const auto firstRoot = refineCrossing(left, best.parameter, leftDistance);
                const auto secondRoot = refineCrossing(best.parameter, right, sign * bestValue);
                if (firstRoot.valid) hits.push_back(firstRoot);
                if (secondRoot.valid) hits.push_back(secondRoot);
            }
        }
        std::sort(hits.begin(), hits.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.parameter < rhs.parameter;
        });
        hits.erase(std::unique(hits.begin(), hits.end(), [&](const auto& lhs, const auto& rhs) {
            if (std::abs(lhs.parameter - rhs.parameter) >
                std::max(tolerance.parameterEpsilon, 1e-8))
            {
                return false;
            }
            const auto middle = EvaluateCurveOnSurfaceSegment(
                curve, segmentIndex, std::lerp(lhs.parameter, rhs.parameter, 0.5));
            return middle.valid &&
                   std::abs(plane.SignedDistanceTo(middle.point, tolerance.distanceEpsilon)) <=
                       tolerance.distanceEpsilon;
        }), hits.end());
        return hits;
    }

    [[nodiscard]] inline CurveOnSurfaceSample FindPlaneCurveOnSurfaceSegmentHit(
        const SCPlane& plane, const SCCurveOnSurface& curve, std::size_t segmentIndex,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto hits = FindPlaneCurveOnSurfaceSegmentHits(plane, curve, segmentIndex, tolerance);
        return hits.empty() ? CurveOnSurfaceSample{} : hits.front();
    }
}
