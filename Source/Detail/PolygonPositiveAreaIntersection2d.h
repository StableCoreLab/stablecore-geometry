#pragma once

#include "Core/Relation.h"
#include "Core/Results.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"
#include "CheckedSegmentIntersection2d.h"
#include "CurveArrangement2d.h"

namespace Geometry::Detail
{
    // checked 几何状态：NoIntersection 是已可靠判定的正常结果，不是失败；
    // InvalidInput 与 NumericalIndeterminate 必须向上分别映射为公开失败原因。
    // checked 面面积：必须显式返回成功或非有限结果，不能用零面积代替失败。
    struct SCCheckedArea2d
    {
        SCCheckedGeometryStatus2d status{SCCheckedGeometryStatus2d::InvalidInput};
        double signedArea{0.0};
    };

    // checked 严格定位：内部结果必须包含 success、containment 与状态，
    // 不得把无效或无法判定情形回退为 Outside。
    struct SCCheckedContainment2d
    {
        SCCheckedGeometryStatus2d status{SCCheckedGeometryStatus2d::InvalidInput};
        SCPointContainment2d containment{SCPointContainment2d::Outside};
    };

    // checked 面代表点：构造并验证代表点严格位于面环内部，不得以默认点表达失败。
    struct SCCheckedRepresentative2d
    {
        SCCheckedGeometryStatus2d status{SCCheckedGeometryStatus2d::InvalidInput};
        SCPoint2d point{};
    };

    [[nodiscard]] SCCheckedArea2d CheckedComputeSignedArea(const SCPolyline2d& ring, double tolerance);

    // 构造并验证面代表点严格位于面环内部。signedArea 用于确定朝环内部偏移的方向。
    [[nodiscard]] SCCheckedRepresentative2d CheckedFaceRepresentative(const SCPolyline2d& ring,
                                                                      double signedArea,
                                                                      double tolerance);

    // checked 严格定位：点严格位于闭合环内部时返回 Inside；位于边界或无法可靠判定时
    // 返回 NumericalIndeterminate，不得回退为 Outside。
    [[nodiscard]] SCCheckedContainment2d CheckedStrictlyInsideRing(const SCPoint2d& point,
                                                                    const SCPolyline2d& ring,
                                                                    double tolerance);

    // checked 填充集定位：点严格位于规范化多边形（外环内、所有洞外）内部时返回 Inside；
    // 边界或无法可靠判定时返回 NumericalIndeterminate。
    [[nodiscard]] SCCheckedContainment2d CheckedStrictlyInsidePolygon(const SCPoint2d& point,
                                                                       const SCPolygon2d& polygon,
                                                                       double tolerance);

    enum class PositiveAreaStatus2d
    {
        Success,
        ArrangementFailure,
        FaceClassificationFailure,
        NumericalIndeterminate,
        NonFiniteResult
    };

    struct PositiveAreaResult2d
    {
        PositiveAreaStatus2d status{PositiveAreaStatus2d::ArrangementFailure};
        bool hasPositiveAreaIntersection{false};
    };

    // 专用 checked 正面积交集流程：对两个规范化多边形构造曲线 arrangement，对每个可可靠
    // 判定为正面积的面构造并验证代表点，再对其做填充集 checked 定位。任一面分类失败优先返回失败；
    // 仅在所有面均验证成功后，依据是否存在至少一个同时严格位于两个填充集合内部的代表点返回结果。
    [[nodiscard]] PositiveAreaResult2d BuildPositiveAreaArrangement2d(const SCPolygon2d& first,
                                                                       const SCPolygon2d& second,
                                                                       double tolerance);
}  // namespace Geometry::Detail
