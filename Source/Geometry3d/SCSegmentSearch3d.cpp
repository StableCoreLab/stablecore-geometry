#include "Geometry3d/SCSegmentSearch3d.h"

#include <algorithm>
#include <sstream>
#include <utility>

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::shared_ptr<const ISCCurve3d> WrapLineSegment(const SCLineSegment3d& segment)
        {
            const SCLine3d line = SCLine3d::FromOriginAndDirection(segment.startPoint, segment.endPoint - segment.startPoint);
            return std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(line, SCIntervald{0.0, 1.0}));
        }
    }  // namespace

    SCSegmentSearch3d::SCSegmentSearch3d(std::vector<SCSegmentSearchEntry3d> entries) : entries_(std::move(entries))
    {
        for (const auto& entry : entries_)
        {
            nextId_ = std::max(nextId_, entry.id + 1);
        }
    }

    void SCSegmentSearch3d::Clear()
    {
        entries_.clear();
        nextId_ = 0;
    }

    std::size_t SCSegmentSearch3d::Add(std::shared_ptr<const ISCCurve3d> curve)
    {
        const std::size_t id = nextId_++;
        const SCBox3d box = curve != nullptr ? curve->Bounds() : SCBox3d{};
        entries_.push_back(SCSegmentSearchEntry3d{id, std::move(curve), box});
        return id;
    }

    std::size_t SCSegmentSearch3d::Add(const ISCCurve3d& curve)
    {
        return Add(curve.Clone());
    }

    std::size_t SCSegmentSearch3d::Add(const SCLineSegment3d& segment)
    {
        return Add(WrapLineSegment(segment));
    }

    std::size_t SCSegmentSearch3d::Add(const SCLineCurve3d& curve)
    {
        return Add(curve.Clone());
    }

    bool SCSegmentSearch3d::Remove(std::size_t id)
    {
        const auto it = std::remove_if(
            entries_.begin(), entries_.end(), [id](const SCSegmentSearchEntry3d& entry) { return entry.id == id; });
        if (it == entries_.end())
        {
            return false;
        }
        entries_.erase(it, entries_.end());
        return true;
    }

    std::size_t SCSegmentSearch3d::Size() const
    {
        return entries_.size();
    }

    bool SCSegmentSearch3d::IsEmpty() const
    {
        return entries_.empty();
    }

    bool SCSegmentSearch3d::IsValid() const
    {
        for (const auto& entry : entries_)
        {
            if (!entry.IsValid())
            {
                return false;
            }
        }
        return true;
    }

    bool SCSegmentSearch3d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCSegmentSearchEntry3d* SCSegmentSearch3d::Find(std::size_t id) const
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCSegmentSearchEntry3d& entry) { return entry.id == id; });
        return it == entries_.end() ? nullptr : &*it;
    }

    std::vector<std::size_t> SCSegmentSearch3d::QueryIntersecting(const SCBox3d& box, double eps) const
    {
        std::vector<std::size_t> result;
        for (const auto& entry : entries_)
        {
            if (Intersects(entry.box, box, eps))
            {
                result.push_back(entry.id);
            }
        }
        return result;
    }

    std::vector<std::size_t> SCSegmentSearch3d::QueryIntersecting(const ISCCurve3d& curve, double eps) const
    {
        std::vector<std::size_t> result;
        const SCBox3d queryBox = curve.Bounds();
        for (const auto& entry : entries_)
        {
            if (Intersects(entry.box, queryBox, eps))
            {
                result.push_back(entry.id);
            }
        }
        return result;
    }

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryWithinDistance(const SCPoint3d& point, double maxDistance) const
    {
        return QueryWithinDistance(point, maxDistance, {});
    }

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryWithinDistance(const SCPoint3d& point,
                                                                              double maxDistance,
                                                                              const SCGeometryTolerance3d& tolerance) const
    {
        std::vector<SCSegmentSearchHit3d> result;
        const double maxDistanceSquared = maxDistance * maxDistance;
        for (const auto& entry : entries_)
        {
            const auto projection = ProjectPointToCurve(point, *entry.curve, tolerance);
            if (!projection.success || projection.distanceSquared > maxDistanceSquared)
            {
                continue;
            }

            const bool onSegment = entry.curve->ParameterRange().Contains(projection.parameter);
            result.push_back(SCSegmentSearchHit3d{entry.id, projection.point, projection.distanceSquared, projection.parameter, onSegment});
        }
        return result;
    }

    std::optional<SCSegmentSearchHit3d> SCSegmentSearch3d::Nearest(const SCPoint3d& point) const
    {
        return Nearest(point, {});
    }

    std::optional<SCSegmentSearchHit3d> SCSegmentSearch3d::Nearest(const SCPoint3d& point,
                                                                   const SCGeometryTolerance3d& tolerance) const
    {
        std::optional<SCSegmentSearchHit3d> best;
        for (const auto& entry : entries_)
        {
            const auto projection = ProjectPointToCurve(point, *entry.curve, tolerance);
            if (!projection.success)
            {
                continue;
            }

            if (!best || projection.distanceSquared < best->distanceSquared)
            {
                best = SCSegmentSearchHit3d{entry.id,
                                            projection.point,
                                            projection.distanceSquared,
                                            projection.parameter,
                                            entry.curve->ParameterRange().Contains(projection.parameter)};
            }
        }
        return best;
    }

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryKNearest(const SCPoint3d& point,
                                                                       std::size_t k,
                                                                       double maxDistance) const
    {
        return QueryKNearest(point, k, maxDistance, {});
    }

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryKNearest(const SCPoint3d& point,
                                                                       std::size_t k,
                                                                       double maxDistance,
                                                                       const SCGeometryTolerance3d& tolerance) const
    {
        std::vector<SCSegmentSearchHit3d> hits;
        if (!(maxDistance >= 0.0))
        {
            return hits;
        }

        const double maxDistanceSquared = maxDistance * maxDistance;
        hits.reserve(entries_.size());
        for (const auto& entry : entries_)
        {
            const auto projection = ProjectPointToCurve(point, *entry.curve, tolerance);
            if (!projection.success || projection.distanceSquared > maxDistanceSquared)
            {
                continue;
            }

            hits.push_back(SCSegmentSearchHit3d{entry.id,
                                                projection.point,
                                                projection.distanceSquared,
                                                projection.parameter,
                                                entry.curve->ParameterRange().Contains(projection.parameter)});
        }

        if (k < hits.size())
        {
            std::nth_element(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(k), hits.end(), [](const auto& lhs, const auto& rhs) {
                if (lhs.distanceSquared != rhs.distanceSquared)
                {
                    return lhs.distanceSquared < rhs.distanceSquared;
                }
                return lhs.id < rhs.id;
            });
            hits.resize(k);
        }

        std::sort(hits.begin(), hits.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.distanceSquared != rhs.distanceSquared)
            {
                return lhs.distanceSquared < rhs.distanceSquared;
            }
            return lhs.id < rhs.id;
        });
        return hits;
    }

    std::string SCSegmentSearch3d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCSegmentSearch3d{size=" << Size() << ", nextId=" << nextId_
               << ", valid=" << (IsValid() ? "true" : "false") << "}";
        return stream.str();
    }

    const std::vector<SCSegmentSearchEntry3d>& SCSegmentSearch3d::Entries() const
    {
        return entries_;
    }

    std::vector<SCSegmentSearchEntry3d>& SCSegmentSearch3d::Entries()
    {
        return entries_;
    }

    const std::vector<SCSegmentSearchEntry3d>& SCSegmentSearch3d::Data() const
    {
        return Entries();
    }

    std::vector<SCSegmentSearchEntry3d>& SCSegmentSearch3d::Data()
    {
        return Entries();
    }
}  // namespace Geometry
