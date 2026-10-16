#pragma once

#include "Core/Intersection.h"

namespace Geometry::Detail
{
    // 一般曲面只收集可核验根，不宣称有限种子覆盖整个参数域。
    [[nodiscard]] std::vector<SCLineBrepFaceIntersection3d> CollectLineFaceHits(
        const SCLine3d&, const SCBrepFace&, const SCGeometryTolerance3d&);
}
