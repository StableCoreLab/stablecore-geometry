#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Core/Metrics.h"
#include "Core/Projection.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Support/Epsilon.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    struct GEOMETRY_API SCSegmentSearchEntry3d
    {
        std::size_t id{0};
        std::shared_ptr<const ISCCurve3d> curve{};
        SCBox3d box{};

        [[nodiscard]] bool IsValid() const
        {
            return curve != nullptr && box.IsValid();
        }
    };

    struct GEOMETRY_API SCSegmentSearchHit3d
    {
        std::size_t id{0};
        SCPoint3d point{};
        double distanceSquared{0.0};
        double parameter{0.0};
        bool isOnSegment{false};

        [[nodiscard]] bool IsValid() const
        {
            return point.IsValid() && distanceSquared >= 0.0;
        }
    };

    class GEOMETRY_API SCSegmentSearch3d
    {
    public:
        SCSegmentSearch3d() = default;
        explicit SCSegmentSearch3d(std::vector<SCSegmentSearchEntry3d> entries);

        void Clear();
        std::size_t Add(std::shared_ptr<const ISCCurve3d> curve);
        std::size_t Add(const ISCCurve3d& curve);
        std::size_t Add(const SCLineSegment3d& segment);
        std::size_t Add(const SCLineCurve3d& curve);
        bool Remove(std::size_t id);

        [[nodiscard]] std::size_t Size() const;
        [[nodiscard]] bool IsEmpty() const;
        [[nodiscard]] bool IsValid() const;
        [[nodiscard]] bool Contains(std::size_t id) const;
        [[nodiscard]] const SCSegmentSearchEntry3d* Find(std::size_t id) const;
        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const SCBox3d& box,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<std::size_t> QueryIntersecting(const ISCCurve3d& curve,
                                                                 double eps = Geometry::kDefaultEpsilon) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d> QueryWithinDistance(const SCPoint3d& point,
                                                                            double maxDistance) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d> QueryWithinDistance(const SCPoint3d& point,
                                                                            double maxDistance,
                                                                            const SCGeometryTolerance3d& tolerance) const;
        [[nodiscard]] std::optional<SCSegmentSearchHit3d> Nearest(const SCPoint3d& point) const;
        [[nodiscard]] std::optional<SCSegmentSearchHit3d> Nearest(const SCPoint3d& point,
                                                                  const SCGeometryTolerance3d& tolerance) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d> QueryKNearest(const SCPoint3d& point,
                                                                      std::size_t k,
                                                                      double maxDistance =
                                                                          std::numeric_limits<double>::max()) const;
        [[nodiscard]] std::vector<SCSegmentSearchHit3d> QueryKNearest(
            const SCPoint3d& point,
            std::size_t k,
            double maxDistance,
            const SCGeometryTolerance3d& tolerance) const;
        [[nodiscard]] std::string DebugString() const;

        [[nodiscard]] const std::vector<SCSegmentSearchEntry3d>& Entries() const;
        [[nodiscard]] std::vector<SCSegmentSearchEntry3d>& Entries();
        [[nodiscard]] const std::vector<SCSegmentSearchEntry3d>& Data() const;
        [[nodiscard]] std::vector<SCSegmentSearchEntry3d>& Data();

    private:
        std::size_t nextId_{0};
        std::vector<SCSegmentSearchEntry3d> entries_{};
    };
}  // namespace Geometry
