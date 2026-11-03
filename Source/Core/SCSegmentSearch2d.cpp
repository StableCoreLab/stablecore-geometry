#include "Geometry2d/SCSegmentSearch2d.h"

#include "Core/Intersection.h"
#include "Core/Metrics.h"
#include "Core/Projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include "../Detail/GeometryContainerOps.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCBox2d BoundsAtTolerance(const ISCSegment2d& segment, double eps)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                return line->Bounds(eps);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                return arc->Bounds(eps);
            }
            return segment.IsValid() ? segment.Bounds() : SCBox2d{};
        }
    }  // namespace

    SCSegmentSearch2d::SCSegmentSearch2d(std::vector<SCSegmentSearchEntry2d> entries) : entries_(std::move(entries))
    {
        for (const auto& entry : entries_)
        {
            nextId_ = std::max(nextId_, entry.id + 1);
        }
    }

    void SCSegmentSearch2d::Clear()
    {
        entries_.clear();
        nextId_ = 0;
    }

    std::size_t SCSegmentSearch2d::Add(std::shared_ptr<const ISCSegment2d> segment)
    {
        const SCBox2d box = segment != nullptr ? segment->Bounds() : SCBox2d{};
        const std::size_t id = nextId_++;
        entries_.push_back(SCSegmentSearchEntry2d{id, std::move(segment), box});
        return id;
    }

    std::size_t SCSegmentSearch2d::Add(const ISCSegment2d& segment)
    {
        return Add(segment.Clone());
    }
    bool SCSegmentSearch2d::Remove(std::size_t id)
    {
        const auto it = std::remove_if(
            entries_.begin(), entries_.end(), [id](const SCSegmentSearchEntry2d& entry) { return entry.id == id; });
        if (it == entries_.end())
        {
            return false;
        }
        entries_.erase(it, entries_.end());
        return true;
    }

    std::size_t SCSegmentSearch2d::Size() const
    {
        return Detail::ContainerCount(entries_);
    }
    bool SCSegmentSearch2d::IsEmpty() const
    {
        return Detail::ContainerIsEmpty(entries_);
    }
    bool SCSegmentSearch2d::IsValid() const
    {
        return Detail::ContainerIsValid(entries_);
    }

    bool SCSegmentSearch2d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCSegmentSearchEntry2d* SCSegmentSearch2d::Find(std::size_t id) const
    {
        return Detail::FindEntryById(entries_, id);
    }

    std::vector<std::size_t> SCSegmentSearch2d::QueryIntersecting(const SCBox2d& box, double eps) const
    {
        std::vector<std::size_t> result;
        if (!std::isfinite(eps) || eps < 0.0) return result;
        for (const auto& entry : entries_)
        {
            if (entry.segment == nullptr) continue;
            const SCBox2d currentBounds = BoundsAtTolerance(*entry.segment, eps);
            if (currentBounds.IsValid() && Intersects(currentBounds, box, eps))
            {
                result.push_back(entry.id);
            }
        }
        return result;
    }

    std::vector<std::size_t> SCSegmentSearch2d::QueryIntersecting(const ISCSegment2d& segment, double eps) const
    {
        std::vector<std::size_t> result;
        if (!std::isfinite(eps) || eps < 0.0 || !BoundsAtTolerance(segment, eps).IsValid())
        {
            return result;
        }
        for (const auto& entry : entries_)
        {
            if (entry.segment != nullptr && BoundsAtTolerance(*entry.segment, eps).IsValid() &&
                HasIntersection(*entry.segment, segment, eps))
            {
                result.push_back(entry.id);
            }
        }
        return result;
    }

    std::vector<SCSegmentSearchHit2d> SCSegmentSearch2d::QueryWithinDistance(const SCPoint2d& point, double maxDistance) const
    {
        std::vector<SCSegmentSearchHit2d> result;
        if (!(maxDistance >= 0.0))
        {
            return result;
        }
        for (const auto& entry : entries_)
        {
            if (!entry.IsValid()) continue;
            const auto projection = ProjectPointToSegment(point, *entry.segment, true);
            if (!projection.IsValid()) continue;
            const double distance = Distance(point, projection.point);
            if (std::isfinite(distance) && distance <= maxDistance)
            {
                result.push_back(SCSegmentSearchHit2d{entry.id,
                                                    projection.point,
                                                    projection.distanceSquared,
                                                    projection.parameter,
                                                    projection.isOnSegment});
            }
        }
        return result;
    }

    std::optional<SCSegmentSearchHit2d> SCSegmentSearch2d::Nearest(const SCPoint2d& point) const
    {
        std::optional<SCSegmentSearchHit2d> best;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (const auto& entry : entries_)
        {
            if (!entry.IsValid()) continue;
            const auto projection = ProjectPointToSegment(point, *entry.segment, true);
            if (!projection.IsValid()) continue;
            const double distance = Distance(point, projection.point);
            if (std::isfinite(distance) && (!best || distance < bestDistance))
            {
                bestDistance = distance;
                best = SCSegmentSearchHit2d{entry.id,
                                          projection.point,
                                          projection.distanceSquared,
                                          projection.parameter,
                                          projection.isOnSegment};
            }
        }
        return best;
    }

    std::vector<SCSegmentSearchHit2d> SCSegmentSearch2d::QueryKNearest(const SCPoint2d& point,
                                                                       std::size_t k,
                                                                       double maxDistance) const
    {
        std::vector<SCSegmentSearchHit2d> hits;
        if (!(maxDistance >= 0.0))
        {
            return hits;
        }

        hits.reserve(entries_.size());
        for (const auto& entry : entries_)
        {
            if (!entry.IsValid()) continue;
            const auto projection = ProjectPointToSegment(point, *entry.segment, true);
            if (!projection.IsValid()) continue;
            const double distance = Distance(point, projection.point);
            if (!std::isfinite(distance) || distance > maxDistance) continue;

            hits.push_back(SCSegmentSearchHit2d{entry.id,
                                                projection.point,
                                                projection.distanceSquared,
                                                projection.parameter,
                                                projection.isOnSegment});
        }

        const auto nearer = [&point](const auto& lhs, const auto& rhs) {
            const double lhsDistance = Distance(point, lhs.point);
            const double rhsDistance = Distance(point, rhs.point);
            if (lhsDistance != rhsDistance)
            {
                return lhsDistance < rhsDistance;
            }
            return lhs.id < rhs.id;
        };
        if (k < hits.size())
        {
            std::nth_element(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(k), hits.end(), nearer);
            hits.resize(k);
        }
        std::sort(hits.begin(), hits.end(), nearer);
        return hits;
    }

    std::string SCSegmentSearch2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCSegmentSearch2d{size=" << Size() << ", nextId=" << nextId_
               << ", valid=" << (IsValid() ? "true" : "false") << "}";
        return stream.str();
    }

    const std::vector<SCSegmentSearchEntry2d>& SCSegmentSearch2d::Entries() const
    {
        return entries_;
    }
    std::vector<SCSegmentSearchEntry2d>& SCSegmentSearch2d::Entries()
    {
        return entries_;
    }
    const std::vector<SCSegmentSearchEntry2d>& SCSegmentSearch2d::Data() const
    {
        return Entries();
    }
    std::vector<SCSegmentSearchEntry2d>& SCSegmentSearch2d::Data()
    {
        return Entries();
    }
}  // namespace Geometry
