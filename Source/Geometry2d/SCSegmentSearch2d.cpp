#include "Geometry2d/SCSegmentSearch2d.h"

#include <algorithm>
#include <sstream>
#include <utility>

namespace Geometry
{
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
        const std::size_t id = nextId_++;
        entries_.push_back(
            SCSegmentSearchEntry2d{id, std::move(segment), entries_.empty() ? SCBox2d{} : entries_.back().box});
        entries_.back().box = entries_.back().segment->Bounds();
        return id;
    }

    std::size_t SCSegmentSearch2d::Add(const ISCSegment2d& segment)
    {
        return Add(segment.Clone());
    }
    std::size_t SCSegmentSearch2d::Add(const SCLineSegment2d& segment)
    {
        return Add(segment.Clone());
    }
    std::size_t SCSegmentSearch2d::Add(const SCArcSegment2d& segment)
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
        return entries_.size();
    }
    bool SCSegmentSearch2d::IsEmpty() const
    {
        return entries_.empty();
    }
    bool SCSegmentSearch2d::IsValid() const
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

    bool SCSegmentSearch2d::Contains(std::size_t id) const
    {
        return Find(id) != nullptr;
    }

    const SCSegmentSearchEntry2d* SCSegmentSearch2d::Find(std::size_t id) const
    {
        const auto it = std::find_if(
            entries_.begin(), entries_.end(), [id](const SCSegmentSearchEntry2d& entry) { return entry.id == id; });
        return it == entries_.end() ? nullptr : &*it;
    }

    std::vector<std::size_t> SCSegmentSearch2d::QueryIntersecting(const SCBox2d& box, double eps) const
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

    std::vector<std::size_t> SCSegmentSearch2d::QueryIntersecting(const ISCSegment2d& segment, double eps) const
    {
        std::vector<std::size_t> result;
        for (const auto& entry : entries_)
        {
            if (HasIntersection(*entry.segment, segment, eps))
            {
                result.push_back(entry.id);
            }
        }
        return result;
    }

    std::vector<SCSegmentSearchHit2d> SCSegmentSearch2d::QueryWithinDistance(const SCPoint2d& point, double maxDistance) const
    {
        std::vector<SCSegmentSearchHit2d> result;
        const double maxDistanceSquared = maxDistance * maxDistance;
        for (const auto& entry : entries_)
        {
            const auto projection = ProjectPointToSegment(point, *entry.segment, true);
            if (projection.distanceSquared <= maxDistanceSquared)
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
        for (const auto& entry : entries_)
        {
            const auto projection = ProjectPointToSegment(point, *entry.segment, true);
            if (!best || projection.distanceSquared < best->distanceSquared)
            {
                best = SCSegmentSearchHit2d{entry.id,
                                          projection.point,
                                          projection.distanceSquared,
                                          projection.parameter,
                                          projection.isOnSegment};
            }
        }
        return best;
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
