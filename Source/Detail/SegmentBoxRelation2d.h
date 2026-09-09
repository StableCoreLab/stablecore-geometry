#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Types/Geometry2d/SCBox2.h"

namespace Geometry::Detail
{
    // 段-盒关系分类。仅支持 SCLineSegment2d 与 SCArcSegment2d；不得降级为折线采样。
    enum class SCBoxSegmentRelation2d
    {
        Unknown,
        Disjoint,
        Touching,
        Contained,
        ContainedTouching,
        Crossing
    };

    // 段-盒分类失败原因。失败不得编码为 Disjoint，必须沿 SCBoxSegmentClassification2d 原样传播。
    enum class SCBoxSegmentFailure2d
    {
        None,
        InvalidSegment,
        NonFiniteInput,
        LocalTransformFailure,
        IntersectionSolveFailure,
        EventOrderingFailure,
        RepresentativePointFailure
    };

    struct SCBoxSegmentClassification2d
    {
        bool success{false};
        SCBoxSegmentRelation2d relation{SCBoxSegmentRelation2d::Unknown};
        SCBoxSegmentFailure2d failure{SCBoxSegmentFailure2d::InvalidSegment};
    };

    // 对单条段与扩张闭盒 B_eps（已含调用方 eps）做精确分类。expandedBox 即 B_eps，
    // 内核不得再次把调用方 eps 用作几何扩张。所有 Line/Arc 求交、圆弧参数事件、PointAt()
    // 代表点和边界分类都在同一局部几何中完成，段参数 t 保持与原段一致。
    [[nodiscard]] SCBoxSegmentClassification2d ClassifySegmentBox(const ISCSegment2d& segment,
                                                                  const SCBox2d& expandedBox);
}  // namespace Geometry::Detail
