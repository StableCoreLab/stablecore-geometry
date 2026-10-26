#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"
#include "SurfaceProjectionSolver.h"
#include "Types/Geometry3d/SCIntervald.h"

namespace Geometry::Detail
{
    [[nodiscard]] inline SCPoint3d ParameterSeedMidpoint(const SCPoint3d& a,const SCPoint3d& b)
    { return {std::lerp(a.x,b.x,0.5),std::lerp(a.y,b.y,0.5),std::lerp(a.z,b.z,0.5)}; }

    [[nodiscard]] inline double ParameterSeedNorm(const SCVector3d& v)
    { return std::hypot(v.x,v.y,v.z); }

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
            return {std::lerp(range.min, range.max, 0.5)};
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
            seeds.push_back(range.min);
            if (maxSeedCount >= 2)
            {
                seeds.push_back(range.max);
            }
            const std::size_t sampleCellCount = (maxSeedCount - seeds.size()) / 3;
            for (std::size_t sample = 0; sample < sampleCellCount; ++sample)
            {
                const std::size_t cell = std::min(
                    cellCount - 1, ((2 * sample + 1) * cellCount) / (2 * sampleCellCount));
                const double t0 = boundaries[cell];
                const double t1 = boundaries[cell + 1];
                seeds.push_back(t0);
                seeds.push_back(std::lerp(t0, t1, 0.5));
                seeds.push_back(t1);
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
            const double tm = std::lerp(t0, t1, 0.5);
            const SCPoint3d p0 = curve.PointAt(t0);
            const SCPoint3d pm = curve.PointAt(tm);
            const SCPoint3d p1 = curve.PointAt(t1);
            const SCCurveEval3d e0 = curve.Evaluate(t0, 1);
            const SCCurveEval3d em = curve.Evaluate(tm, 1);
            const SCCurveEval3d e1 = curve.Evaluate(t1, 1);
            if (!e0.IsValid() || !em.IsValid() || !e1.IsValid() ||
                e0.derivativeOrder < 1 || em.derivativeOrder < 1 || e1.derivativeOrder < 1 ||
                e0.point != p0 || em.point != pm || e1.point != p1)
            {
                if (depth == 0 && depth < maxDepth && leafBudget >= 2 && tm > t0 && tm < t1)
                {
                    ++leafCount;
                    self(self, t0, tm, depth + 1, leafCount);
                    self(self, tm, t1, depth + 1, leafCount);
                    return;
                }
                if (depth < maxDepth && tm > t0 && tm < t1 && leafCount + 1 <= leafBudget &&
                    p0.IsValid() && pm.IsValid() && p1.IsValid() &&
                    ParameterSeedNorm(pm-ParameterSeedMidpoint(p0,p1)) > flatnessTolerance)
                {
                    ++leafCount;
                    self(self, t0, tm, depth + 1, leafCount);
                    self(self, tm, t1, depth + 1, leafCount);
                    return;
                }
                seeds.push_back(t0);
                seeds.push_back(tm);
                seeds.push_back(t1);
                if (depth == 0 && perCellSeedBudget >= 5)
                {
                    seeds.push_back(std::lerp(t0, tm, 0.5));
                    seeds.push_back(std::lerp(tm, t1, 0.5));
                }
                return;
            }

            const SCPoint3d chordMidpoint = ParameterSeedMidpoint(p0,p1);
            const double flatness = ParameterSeedNorm(pm-chordMidpoint);
            SCVector3d tangent0,tangent1;
            bool tangentTurn = false;
            if (TryUnitVector(e0.firstDerivative,tangent0) && TryUnitVector(e1.firstDerivative,tangent1))
            {
                const double tangentCosine = Dot(tangent0,tangent1);
                tangentTurn = tangentCosine < 0.9659258262890683;
            }

            if (depth < maxDepth && tm > t0 && tm < t1 &&
                (flatness > flatnessTolerance || tangentTurn))
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
        std::size_t maxSeedCount = 256,
        SurfaceSearchBudget* budget = nullptr)
    {
        const auto positionAt = [&](double u, double v) {
            if (budget != nullptr && !budget->Consume(0))
                return SCPoint3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
            return surface.PointAt(u, v);
        };
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
                    seeds.push_back({std::lerp(uBoundaries[ui], uBoundaries[ui + 1], 0.5),
                                     std::lerp(vBoundaries[vi], vBoundaries[vi + 1], 0.5)});
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
            const double um = std::lerp(u0, u1, 0.5);
            const double vm = std::lerp(v0, v1, 0.5);
            const SCPoint3d p00 = positionAt(u0, v0);
            const SCPoint3d p10 = positionAt(u1, v0);
            const SCPoint3d p01 = positionAt(u0, v1);
            const SCPoint3d p11 = positionAt(u1, v1);
            const SCPoint3d pm = positionAt(um, vm);
            const SCPoint3d pu0 = positionAt(um, v0);
            const SCPoint3d pu1 = positionAt(um, v1);
            const SCPoint3d pv0 = positionAt(u0, vm);
            const SCPoint3d pv1 = positionAt(u1, vm);
            const SCPoint3d bilinearCenter = ParameterSeedMidpoint(ParameterSeedMidpoint(p00,p10),ParameterSeedMidpoint(p01,p11));
            const double flatness = std::max({ParameterSeedNorm(pm-bilinearCenter),
                ParameterSeedNorm(pu0-ParameterSeedMidpoint(p00,p10)),
                ParameterSeedNorm(pu1-ParameterSeedMidpoint(p01,p11)),
                ParameterSeedNorm(pv0-ParameterSeedMidpoint(p00,p01)),
                ParameterSeedNorm(pv1-ParameterSeedMidpoint(p10,p11))});
            const double twist = ParameterSeedNorm((p11-p10)-(p01-p00));
            if (depth < maxDepth && leafCount + 3 <= leafBudget &&
                std::max(flatness, twist) > flatnessTolerance)
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
            seeds.push_back({std::lerp(uRange.min, uRange.max, 0.5),
                             std::lerp(vRange.min, vRange.max, 0.5)});
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
