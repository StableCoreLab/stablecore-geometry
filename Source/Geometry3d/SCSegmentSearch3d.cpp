#include "Geometry3d/SCSegmentSearch3d.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <utility>

#include "Core/Metrics.h"
#include "Core/Projection.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] std::shared_ptr<const ISCCurve3d> WrapLineSegment(const SCLineSegment3d& segment)
        {
            const SCLine3d line = SCLine3d::FromOriginAndDirection(segment.startPoint, segment.endPoint - segment.startPoint);
            return std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(line, SCIntervald{0.0, 1.0}));
        }

        [[nodiscard]] bool Encloses(const SCBox3d& outer, const SCBox3d& inner)
        {
            if (!outer.IsValid() || !inner.IsValid())
            {
                return false;
            }
            const SCPoint3d& outerMin = outer.MinPoint();
            const SCPoint3d& outerMax = outer.MaxPoint();
            const SCPoint3d& innerMin = inner.MinPoint();
            const SCPoint3d& innerMax = inner.MaxPoint();
            return outerMin.x <= innerMin.x && outerMin.y <= innerMin.y && outerMin.z <= innerMin.z &&
                   innerMax.x <= outerMax.x && innerMax.y <= outerMax.y && innerMax.z <= outerMax.z;
        }
    }  // namespace

    SCSegmentSearch3d::SCSegmentSearch3d(std::vector<SCSegmentSearchEntry3d> entries) : entries_(std::move(entries))
    {
        for (const auto& entry : entries_)
        {
            nextId_ = std::max(nextId_, entry.id + 1);
        }
        RebuildIndices();
    }

    void SCSegmentSearch3d::RebuildIndices()
    {
        boxTree_.Clear();
        entryIndices_.clear();
        hasUntrustedBounds_ = false;
        for (std::size_t index = 0; index < entries_.size(); ++index)
        {
            const auto& entry = entries_[index];
            if (entry.curve != nullptr && !Encloses(entry.box, entry.curve->Bounds()))
            {
                hasUntrustedBounds_ = true;
            }
            if (!entryIndices_.try_emplace(entry.id, index).second)
            {
                hasUntrustedBounds_ = true;
            }
            boxTree_.Add(entry.id, entry.box);
        }
    }

    void SCSegmentSearch3d::Clear()
    {
        entries_.clear();
        boxTree_.Clear();
        entryIndices_.clear();
        hasUntrustedBounds_ = false;
        nextId_ = 0;
    }

    std::size_t SCSegmentSearch3d::Add(std::shared_ptr<const ISCCurve3d> curve)
    {
        const std::size_t id = nextId_++;
        const SCBox3d box = curve != nullptr ? curve->Bounds() : SCBox3d{};
        if (curve != nullptr && !box.IsValid())
        {
            hasUntrustedBounds_ = true;
        }
        entries_.push_back(SCSegmentSearchEntry3d{id, std::move(curve), box});
        if (!entryIndices_.emplace(id, entries_.size() - 1).second)
        {
            hasUntrustedBounds_ = true;
        }
        boxTree_.Add(id, box);
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
        RebuildIndices();
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
        const auto it = entryIndices_.find(id);
        return it == entryIndices_.end() ? nullptr : &entries_[it->second];
    }

    std::vector<std::size_t> SCSegmentSearch3d::QueryIntersecting(const SCBox3d& box, double eps) const
    {
        if (entryIndices_.size() != entries_.size())
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
        std::vector<std::size_t> result = boxTree_.Query(box, eps);
        std::sort(result.begin(), result.end(), [this](std::size_t lhs, std::size_t rhs) {
            return entryIndices_.at(lhs) < entryIndices_.at(rhs);
        });
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    std::vector<std::size_t> SCSegmentSearch3d::QueryIntersecting(const ISCCurve3d& curve, double eps) const
    {
        return QueryIntersecting(curve.Bounds(), eps);
    }

    namespace
    {
        [[nodiscard]] SCBox3d MakeSearchBox(const SCPoint3d& point, double radius)
        {
            return SCBox3d::FromMinMax(SCPoint3d{point.x - radius, point.y - radius, point.z - radius},
                                       SCPoint3d{point.x + radius, point.y + radius, point.z + radius});
        }

        [[nodiscard]] std::vector<std::size_t> OrderIndicesByEntry(
            const std::vector<std::size_t>& ids,
            const std::unordered_map<std::size_t, std::size_t>& indices)
        {
            std::vector<std::size_t> ordered;
            ordered.reserve(ids.size());
            for (const std::size_t id : ids)
            {
                ordered.push_back(indices.at(id));
            }
            std::sort(ordered.begin(), ordered.end());
            ordered.erase(std::unique(ordered.begin(), ordered.end()), ordered.end());
            return ordered;
        }

        [[nodiscard]] double StableDistance(const SCPoint3d& first, const SCPoint3d& second)
        {
            if (!first.IsValid() || !second.IsValid())
            {
                return std::numeric_limits<double>::infinity();
            }
            return std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
        }

        [[nodiscard]] bool WithinDistance(const SCPoint3d& first, const SCPoint3d& second, double maximum)
        {
            const double distance = StableDistance(first, second);
            return std::isfinite(distance) && distance <= maximum;
        }
    }  // namespace

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryWithinDistance(const SCPoint3d& point, double maxDistance) const
    {
        return QueryWithinDistance(point, maxDistance, {});
    }

    std::vector<SCSegmentSearchHit3d> SCSegmentSearch3d::QueryWithinDistance(const SCPoint3d& point,
                                                                              double maxDistance,
                                                                              const SCGeometryTolerance3d& tolerance) const
    {
        std::vector<SCSegmentSearchHit3d> result;
        if (std::isnan(maxDistance) || maxDistance < 0.0)
        {
            return result;
        }
        std::vector<std::size_t> candidateIndices;
        if (hasUntrustedBounds_)
        {
            candidateIndices.reserve(entries_.size());
            for (std::size_t index = 0; index < entries_.size(); ++index)
            {
                candidateIndices.push_back(index);
            }
        }
        else if (std::isfinite(maxDistance))
        {
            const SCBox3d queryBox = MakeSearchBox(point, maxDistance);
            if (queryBox.IsValid())
            {
                candidateIndices = OrderIndicesByEntry(boxTree_.Query(queryBox, tolerance.boxPadding), entryIndices_);
            }
            else
            {
                candidateIndices.reserve(entries_.size());
                for (std::size_t index = 0; index < entries_.size(); ++index)
                {
                    candidateIndices.push_back(index);
                }
            }
        }
        else
        {
            candidateIndices.reserve(entries_.size());
            for (std::size_t index = 0; index < entries_.size(); ++index)
            {
                candidateIndices.push_back(index);
            }
        }
        for (const std::size_t index : candidateIndices)
        {
            const auto& entry = entries_[index];
            if (entry.curve == nullptr)
            {
                continue;
            }
            const auto projection = ProjectPointToCurve(point, *entry.curve, tolerance);
            if (!projection.success || !WithinDistance(point, projection.point, maxDistance))
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
        const auto hits = QueryKNearest(point, 1, std::numeric_limits<double>::max(), tolerance);
        return hits.empty() ? std::nullopt : std::optional<SCSegmentSearchHit3d>{hits.front()};
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
        if (k == 0 || std::isnan(maxDistance) || maxDistance < 0.0)
        {
            return hits;
        }

        std::unordered_set<std::size_t> projectedIds;
        std::size_t candidateCount = std::min(k, entries_.size());
        if (candidateCount == 0)
        {
            return hits;
        }

        const auto compareHits = [&point](const auto& lhs, const auto& rhs) {
            const double lhsDistance = StableDistance(point, lhs.point);
            const double rhsDistance = StableDistance(point, rhs.point);
            if (lhsDistance != rhsDistance)
            {
                return lhsDistance < rhsDistance;
            }
            return lhs.id < rhs.id;
        };

        if (hasUntrustedBounds_)
        {
            hits.reserve(entries_.size());
            for (const auto& entry : entries_)
            {
                if (entry.curve == nullptr)
                {
                    continue;
                }
                const auto projection = ProjectPointToCurve(point, *entry.curve, tolerance);
                if (!projection.success || !WithinDistance(point, projection.point, maxDistance))
                {
                    continue;
                }
                hits.push_back(SCSegmentSearchHit3d{entry.id,
                                                    projection.point,
                                                    projection.distanceSquared,
                                                    projection.parameter,
                                                    entry.curve->ParameterRange().Contains(projection.parameter)});
            }
            std::sort(hits.begin(), hits.end(), compareHits);
            if (hits.size() > k)
            {
                hits.resize(k);
            }
            return hits;
        }

        for (;;)
        {
            const auto boxHits = boxTree_.QueryKNearest(point, candidateCount, 0.0);
            for (const SCBoxTreeKnnHit3d& boxHit : boxHits)
            {
                if (!projectedIds.insert(boxHit.id).second)
                {
                    continue;
                }
                const auto* entry = Find(boxHit.id);
                if (entry == nullptr || entry->curve == nullptr)
                {
                    continue;
                }
                const auto projection = ProjectPointToCurve(point, *entry->curve, tolerance);
                if (!projection.success || !WithinDistance(point, projection.point, maxDistance))
                {
                    continue;
                }
                hits.push_back(SCSegmentSearchHit3d{entry->id,
                                                    projection.point,
                                                    projection.distanceSquared,
                                                    projection.parameter,
                                                    entry->curve->ParameterRange().Contains(projection.parameter)});
            }

            if (hits.size() > k)
            {
                std::nth_element(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(k - 1), hits.end(), compareHits);
                hits.resize(k);
            }

            const auto worstHit = std::max_element(hits.begin(), hits.end(), [&point](const auto& lhs, const auto& rhs) {
                return StableDistance(point, lhs.point) < StableDistance(point, rhs.point);
            });
            if (boxHits.size() < candidateCount || candidateCount == entries_.size() ||
                (hits.size() >= k && std::sqrt(std::max(0.0, boxHits.back().distanceSquared)) >
                    StableDistance(point, worstHit->point)))
            {
                break;
            }
            const std::size_t doubled = candidateCount > entries_.size() / 2 ? entries_.size() : candidateCount * 2;
            candidateCount = std::max(candidateCount + 1, doubled);
        }

        std::sort(hits.begin(), hits.end(), compareHits);
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

    const std::vector<SCSegmentSearchEntry3d>& SCSegmentSearch3d::Data() const
    {
        return Entries();
    }
}  // namespace Geometry
