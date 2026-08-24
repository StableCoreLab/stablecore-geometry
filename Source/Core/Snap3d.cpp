#include "Core/Snap3d.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include "Geometry3d/SCSegmentSearch3d.h"

namespace Geometry
{
    namespace
    {
        SCSnapResult3d SnapWithSearch(const SCPoint3d& point,
                                      const std::vector<std::shared_ptr<const ISCCurve3d>>& ownedCurves,
                                      const std::vector<std::size_t>& ids,
                                      double maxDistance,
                                      const SCGeometryTolerance3d& tolerance)
        {
            SCSnapResult3d result{};
            if (!(maxDistance >= 0.0) || ownedCurves.empty())
            {
                return result;
            }

            SCSegmentSearch3d search;
            std::unordered_map<std::size_t, std::size_t> idToIndex;
            for (std::size_t index = 0; index < ownedCurves.size(); ++index)
            {
                const std::size_t id = search.Add(ownedCurves[index]);
                idToIndex[id] = ids[index];
            }

            const auto nearest = search.Nearest(point, tolerance);
            if (!nearest.has_value() || nearest->distanceSquared > maxDistance * maxDistance)
            {
                return result;
            }

            result.snapped = true;
            result.point = nearest->point;
            result.distanceSquared = nearest->distanceSquared;
            result.parameter = nearest->parameter;
            const auto it = idToIndex.find(nearest->id);
            if (it == idToIndex.end())
            {
                return {};
            }
            result.segmentIndex = it->second;
            return result;
        }
    }  // namespace

    SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                         std::span<const ISCCurve3d* const> curves,
                                         double maxDistance,
                                         const SCGeometryTolerance3d& tolerance)
    {
        std::vector<std::shared_ptr<const ISCCurve3d>> ownedCurves;
        ownedCurves.reserve(curves.size());
        std::vector<std::size_t> ids;
        ids.reserve(curves.size());
        for (std::size_t index = 0; index < curves.size(); ++index)
        {
            const ISCCurve3d* curve = curves[index];
            if (curve == nullptr || !curve->IsValid(tolerance))
            {
                continue;
            }

            ownedCurves.push_back(curve->Clone());
            ids.push_back(index);
        }

        return SnapWithSearch(point, ownedCurves, ids, maxDistance, tolerance);
    }

    SCSnapResult3d SnapPointToSegments3d(const SCPoint3d& point,
                                         std::span<const SCLineSegment3d* const> segments,
                                         double maxDistance,
                                         const SCGeometryTolerance3d& tolerance)
    {
        std::vector<std::shared_ptr<const ISCCurve3d>> ownedCurves;
        ownedCurves.reserve(segments.size());
        std::vector<std::size_t> ids;
        ids.reserve(segments.size());
        for (std::size_t index = 0; index < segments.size(); ++index)
        {
            const SCLineSegment3d* segment = segments[index];
            if (segment == nullptr || !segment->IsValid(tolerance.distanceEpsilon))
            {
                continue;
            }

            const SCLine3d line = SCLine3d::FromOriginAndDirection(segment->startPoint, segment->endPoint - segment->startPoint);
            ownedCurves.push_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(line, SCIntervald{0.0, 1.0})));
            ids.push_back(index);
        }

        return SnapWithSearch(point, ownedCurves, ids, maxDistance, tolerance);
    }
}  // namespace Geometry
