#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Types/Geometry3d/SCLine3d.h"

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
        const SCPoint2d uv0 = curve.UvPointAt(segmentIndex);
        const SCPoint2d uv1 = curve.UvPointAt((segmentIndex + 1) % pointCount);
        if (!uv0.IsValid() || !uv1.IsValid())
        {
            return result;
        }
        result.parameter = parameter;
        result.uv = SCPoint2d{std::lerp(uv0.x, uv1.x, parameter),
                              std::lerp(uv0.y, uv1.y, parameter)};
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
        const SCLine3d& line, const SCCurveOnSurface& curve, std::size_t segmentIndex)
    {
        CurveOnSurfaceLineCandidate best{};
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
            return distance;
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
        RefineCurveOnSurfaceInterval(0.0, 1.0 / sampleCount, consider);
        RefineCurveOnSurfaceInterval((sampleCount - 1.0) / sampleCount, 1.0, consider);
        std::sort(minima.begin(), minima.end(), [&](std::size_t lhs, std::size_t rhs) {
            return distances[lhs] < distances[rhs];
        });
        for (std::size_t index = 0; index < std::min<std::size_t>(8, minima.size()); ++index)
        {
            const std::size_t seed = minima[index];
            RefineCurveOnSurfaceInterval(static_cast<double>(seed - 1) / sampleCount,
                                         static_cast<double>(seed + 1) / sampleCount, consider);
        }
        return best;
    }
}
