#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry3d/SCBox3.h"
#include "Types/Geometry3d/SCPoint3.h"
#include "Types/Geometry3d/SCLineSegment3d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& lhs, const SCPoint2d& rhs);
    // 一阶距离直接计算，避免可表示的距离因平方溢出或下溢而失真。
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& lhs, const SCPoint2d& rhs);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const SCLineSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const SCArcSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const ISCSegment2d& segment);

    // 投影点可用时直接计算一阶距离；投影失败时保留失败结果。
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const SCLineSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const SCArcSegment2d& segment);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint2d& point, const ISCSegment2d& segment);

    [[deprecated("Use SCPolyline2d::Bounds() instead")]]
    [[nodiscard]] GEOMETRY_API SCBox2d Bounds(const SCPolyline2d& polyline);

    // 盒与点、盒与盒关系允许 eps == 0；负值或非有限 eps 返回 false。
    [[nodiscard]] GEOMETRY_API bool Contains(const SCBox2d& box,
                                             const SCPoint2d& point,
                                             double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool Intersects(const SCBox2d& lhs,
                                               const SCBox2d& rhs,
                                               double eps = Geometry::kDefaultEpsilon);
    // 盒-多段线包含：当且仅当路径的全部曲线点位于扩张闭盒 B_eps 时返回 true。
    // B_eps 由 box 以 eps 扩张得到。空或零段多段线、无效输入均返回 false。
    [[nodiscard]] GEOMETRY_API bool Contains(const SCBox2d& box,
                                             const SCPolyline2d& polyline,
                                             double eps = Geometry::kDefaultEpsilon);
    // 盒-多段线相交：当且仅当路径与扩张闭盒 B_eps 有公共点时返回 true。
    // 闭合多段线围住选择框但路径本身不接触时仍返回 false。
    [[nodiscard]] GEOMETRY_API bool Intersects(const SCBox2d& box,
                                               const SCPolyline2d& polyline,
                                               double eps = Geometry::kDefaultEpsilon);
    // 三维盒关系与二维盒关系采用相同的容差有效性规则。
    [[nodiscard]] GEOMETRY_API bool Contains(const SCBox3d& box,
                                             const SCPoint3d& point,
                                             double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API bool Intersects(const SCBox3d& lhs,
                                               const SCBox3d& rhs,
                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint2d& point, const SCBox2d& box);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCBox3d& box);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCLineSegment3d& segment);
}  // namespace Geometry
