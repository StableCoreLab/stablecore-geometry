#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"
#include "Types/Geometry3d/SCIntervald.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline std::vector<double> KnotSpanBoundaries(const SCIntervald& range,
                                                                 const std::vector<double>& knots,
                                                                 int degree,
                                                                 std::size_t controlPointCount)
    {
        std::vector<double> boundaries;
        if (!range.IsValid() || degree < 0 || controlPointCount == 0 ||
            knots.size() <= controlPointCount || static_cast<std::size_t>(degree) >= controlPointCount)
        {
            return boundaries;
        }

        boundaries.push_back(range.min);
        for (std::size_t span = static_cast<std::size_t>(degree); span < controlPointCount; ++span)
        {
            const double boundary = knots[span + 1];
            if (boundary > range.min && boundary < range.max)
            {
                boundaries.push_back(boundary);
            }
        }
        boundaries.push_back(range.max);
        boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
        return boundaries;
    }

    [[nodiscard]] inline std::vector<double> AdaptiveCurveParameterSeeds(const ISCCurve3d& curve,
                                                                         std::vector<double> boundaries,
                                                                         double flatnessTolerance,
                                                                         int maxDepth = 8,
                                                                         std::size_t maxSeedCount = 256)
    {
        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }
        if (maxSeedCount == 0)
        {
            return {};
        }
        if (boundaries.size() < 2)
        {
            boundaries = {range.min, range.max};
        }
        std::sort(boundaries.begin(), boundaries.end());
        boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
        if (boundaries.size() < 2)
        {
            return {0.5 * (range.min + range.max)};
        }
        if (boundaries.front() > range.min)
        {
            boundaries.insert(boundaries.begin(), range.min);
        }
        if (boundaries.back() < range.max)
        {
            boundaries.push_back(range.max);
        }

        const std::size_t cellCount = boundaries.size() - 1;
        std::vector<double> seeds;
        seeds.reserve(maxSeedCount);

        if (cellCount > maxSeedCount / 3)
        {
            const std::size_t samplesPerCell = maxSeedCount >= 3 ? 3 : 1;
            const std::size_t sampleCellCount = maxSeedCount / samplesPerCell;
            for (std::size_t sample = 0; sample < sampleCellCount; ++sample)
            {
                const std::size_t cell = std::min(
                    cellCount - 1, ((2 * sample + 1) * cellCount) / (2 * sampleCellCount));
                const double t0 = boundaries[cell];
                const double t1 = boundaries[cell + 1];
                if (samplesPerCell == 3)
                {
                    seeds.push_back(t0);
                    seeds.push_back(0.5 * (t0 + t1));
                    seeds.push_back(t1);
                }
                else
                {
                    seeds.push_back(0.5 * (t0 + t1));
                }
            }
            std::sort(seeds.begin(), seeds.end());
            seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());
            return seeds;
        }

        const std::size_t perCellSeedBudget = maxSeedCount / cellCount;
        const std::size_t leafBudget = std::max<std::size_t>(1, perCellSeedBudget / 3);
        const auto subdivide = [&](const auto& self,
                                   double t0,
                                   double t1,
                                   int depth,
                                   std::size_t& leafCount) -> void {
            const double tm = 0.5 * (t0 + t1);
            const SCCurveEval3d e0 = curve.Evaluate(t0, 1);
            const SCCurveEval3d em = curve.Evaluate(tm, 1);
            const SCCurveEval3d e1 = curve.Evaluate(t1, 1);
            if (!e0.IsValid() || !em.IsValid() || !e1.IsValid())
            {
                seeds.push_back(tm);
                return;
            }

            const SCPoint3d chordMidpoint = e0.point + (e1.point - e0.point) * 0.5;
            const double flatnessSquared = (em.point - chordMidpoint).LengthSquared();
            const double tangentLength0 = e0.firstDerivative.Length();
            const double tangentLength1 = e1.firstDerivative.Length();
            bool tangentTurn = false;
            if (tangentLength0 > 0.0 && tangentLength1 > 0.0)
            {
                const double tangentCosine = Dot(e0.firstDerivative, e1.firstDerivative) /
                                             (tangentLength0 * tangentLength1);
                tangentTurn = tangentCosine < 0.9659258262890683;
            }

            if (depth < maxDepth && (flatnessSquared > flatnessTolerance * flatnessTolerance || tangentTurn))
            {
                if (leafCount + 1 <= leafBudget)
                {
                    ++leafCount;
                    self(self, t0, tm, depth + 1, leafCount);
                    self(self, tm, t1, depth + 1, leafCount);
                    return;
                }
            }
            seeds.push_back(t0);
            seeds.push_back(tm);
            seeds.push_back(t1);
        };

        for (std::size_t i = 1; i < boundaries.size(); ++i)
        {
            if (boundaries[i] > boundaries[i - 1])
            {
                std::size_t leafCount = 1;
                subdivide(subdivide, boundaries[i - 1], boundaries[i], 0, leafCount);
            }
        }
        std::sort(seeds.begin(), seeds.end());
        seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());
        return seeds;
    }

    struct SurfaceParameterSeed
    {
        double u{0.0};
        double v{0.0};
    };

    [[nodiscard]] inline std::vector<SurfaceParameterSeed> AdaptiveSurfaceParameterSeeds(
        const ISCSurface& surface,
        std::vector<double> uBoundaries,
        std::vector<double> vBoundaries,
        double flatnessTolerance,
        int maxDepth = 4,
        std::size_t maxSeedCount = 256)
    {
        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
        }
        if (uBoundaries.size() < 2)
        {
            uBoundaries = {uRange.min, uRange.max};
        }
        if (vBoundaries.size() < 2)
        {
            vBoundaries = {vRange.min, vRange.max};
        }
        const auto normalizeBoundaries = [](std::vector<double>& boundaries, const SCIntervald& range) {
            std::sort(boundaries.begin(), boundaries.end());
            boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
            if (boundaries.size() < 2)
            {
                boundaries = {range.min, range.max};
            }
            if (boundaries.front() > range.min)
            {
                boundaries.insert(boundaries.begin(), range.min);
            }
            if (boundaries.back() < range.max)
            {
                boundaries.push_back(range.max);
            }
        };
        normalizeBoundaries(uBoundaries, uRange);
        normalizeBoundaries(vBoundaries, vRange);

        if (maxSeedCount == 0)
        {
            return {};
        }

        const std::size_t uCellCount = uBoundaries.size() - 1;
        const std::size_t vCellCount = vBoundaries.size() - 1;
        const bool baseCellsExceedBudget = uCellCount > maxSeedCount / vCellCount;
        std::vector<SurfaceParameterSeed> seeds;
        seeds.reserve(maxSeedCount);

        if (baseCellsExceedBudget)
        {
            const std::size_t uSamples = std::min(
                {uCellCount,
                 maxSeedCount,
                 std::max<std::size_t>(1, static_cast<std::size_t>(std::sqrt(
                     static_cast<double>(maxSeedCount) * static_cast<double>(uCellCount) /
                     static_cast<double>(vCellCount))))});
            const std::size_t vSamples = std::min(vCellCount, std::max<std::size_t>(1, maxSeedCount / uSamples));
            for (std::size_t uSample = 0; uSample < uSamples; ++uSample)
            {
                const std::size_t ui = std::min(uCellCount - 1, ((2 * uSample + 1) * uCellCount) / (2 * uSamples));
                for (std::size_t vSample = 0; vSample < vSamples; ++vSample)
                {
                    const std::size_t vi =
                        std::min(vCellCount - 1, ((2 * vSample + 1) * vCellCount) / (2 * vSamples));
                    seeds.push_back({0.5 * (uBoundaries[ui] + uBoundaries[ui + 1]),
                                     0.5 * (vBoundaries[vi] + vBoundaries[vi + 1])});
                }
            }
            return seeds;
        }

        const std::size_t baseCellCount = uCellCount * vCellCount;
        const std::size_t perCellBudget = std::max<std::size_t>(1, maxSeedCount / baseCellCount);
        const auto subdivide = [&](const auto& self,
                                   double u0,
                                   double v0,
                                   double u1,
                                   double v1,
                                   int depth,
                                   std::size_t leafBudget,
                                   std::size_t& leafCount) -> void {
            const double um = 0.5 * (u0 + u1);
            const double vm = 0.5 * (v0 + v1);
            const SCPoint3d p00 = surface.PointAt(u0, v0);
            const SCPoint3d p10 = surface.PointAt(u1, v0);
            const SCPoint3d p01 = surface.PointAt(u0, v1);
            const SCPoint3d p11 = surface.PointAt(u1, v1);
            const SCPoint3d pm = surface.PointAt(um, vm);
            const SCPoint3d pu0 = surface.PointAt(um, v0);
            const SCPoint3d pu1 = surface.PointAt(um, v1);
            const SCPoint3d pv0 = surface.PointAt(u0, vm);
            const SCPoint3d pv1 = surface.PointAt(u1, vm);
            const SCPoint3d bilinearCenter = p00 + ((p10 - p00) + (p01 - p00) + (p11 - p00)) * 0.25;
            const double flatnessSquared = std::max({(pm - bilinearCenter).LengthSquared(),
                                                     (pu0 - (p00 + (p10 - p00) * 0.5)).LengthSquared(),
                                                     (pu1 - (p01 + (p11 - p01) * 0.5)).LengthSquared(),
                                                     (pv0 - (p00 + (p01 - p00) * 0.5)).LengthSquared(),
                                                     (pv1 - (p10 + (p11 - p10) * 0.5)).LengthSquared()});
            const double twistSquared = ((p11 - p10) - (p01 - p00)).LengthSquared();
            if (depth < maxDepth && leafCount + 3 <= leafBudget &&
                std::max(flatnessSquared, twistSquared) > flatnessTolerance * flatnessTolerance)
            {
                leafCount += 3;
                self(self, u0, v0, um, vm, depth + 1, leafBudget, leafCount);
                self(self, um, v0, u1, vm, depth + 1, leafBudget, leafCount);
                self(self, u0, vm, um, v1, depth + 1, leafBudget, leafCount);
                self(self, um, vm, u1, v1, depth + 1, leafBudget, leafCount);
                return;
            }
            seeds.push_back({um, vm});
        };

        for (std::size_t ui = 1; ui < uBoundaries.size(); ++ui)
        {
            for (std::size_t vi = 1; vi < vBoundaries.size(); ++vi)
            {
                if (uBoundaries[ui] > uBoundaries[ui - 1] && vBoundaries[vi] > vBoundaries[vi - 1])
                {
                    std::size_t leafCount = 1;
                    subdivide(subdivide,
                              uBoundaries[ui - 1],
                              vBoundaries[vi - 1],
                              uBoundaries[ui],
                              vBoundaries[vi],
                              0,
                              perCellBudget,
                              leafCount);
                }
            }
        }
        if (seeds.empty())
        {
            seeds.push_back({0.5 * (uRange.min + uRange.max), 0.5 * (vRange.min + vRange.max)});
        }
        std::sort(seeds.begin(), seeds.end(), [](const SurfaceParameterSeed& lhs, const SurfaceParameterSeed& rhs) {
            return std::tie(lhs.u, lhs.v) < std::tie(rhs.u, rhs.v);
        });
        seeds.erase(std::unique(seeds.begin(), seeds.end(), [](const SurfaceParameterSeed& lhs, const SurfaceParameterSeed& rhs) {
                        return lhs.u == rhs.u && lhs.v == rhs.v;
                    }),
                    seeds.end());
        return seeds;
    }
}  // namespace Geometry::Detail
