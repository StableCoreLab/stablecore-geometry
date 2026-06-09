#include "Support/Epsilon.h"

namespace Geometry
{
    const double kPi = 3.141592653589793238462643383279502884;
    const double kTwoPi = 2.0 * kPi;
    const std::array<double, 4> kArcBoundsCriticalAngles = {0.0, kPi * 0.5, kPi, kPi * 1.5};
    const double kSharedTopologyMatchEpsilon = 1e-6;
    const double kLoopCleanupEpsilon = kDefaultEpsilon;
    const double kRepresentativeMatchEpsilon = 1e-6;
    const double kRepresentativeGroupingEpsilon = kRepresentativeMatchEpsilon;
    const double kBrepVertexDedupEpsilon = kRepresentativeMatchEpsilon;
    const double kPlaneProjectionEpsilon = kDefaultEpsilon;
    const double kBodyBooleanDefaultEpsilon = kDefaultEpsilon;
    const double kIntersectionDefaultEpsilon = kDefaultEpsilon;
    const double kProjectionDefaultEpsilon = kDefaultEpsilon;
    const double kPathOpsDefaultEpsilon = kDefaultEpsilon;
    const double kOffsetDefaultEpsilon = kDefaultEpsilon;
    const double kAxisOpsDefaultEpsilon = kDefaultEpsilon;
    const double kAlgorithmsDefaultEpsilon = kDefaultEpsilon;
    const double kArcSegmentDefaultEpsilon = kDefaultEpsilon;
    const double kHealingDefaultEpsilon = kDefaultEpsilon;
    const double kMeshConversionDefaultEpsilon = kDefaultEpsilon;
    const double kShapeOpsDefaultEpsilon = kDefaultEpsilon;
    const double kTransformDefaultEpsilon = kDefaultEpsilon;
    const double kSectionDefaultEpsilon = kDefaultEpsilon;
    const double kSearchPolyDefaultEpsilon = kDefaultEpsilon;
    const double kBooleanComparisonEpsilon = 1e-12;
    const double kBooleanAreaEpsilon = 1e-10;
    const double kBooleanRebuildFallbackEpsilon = 1e-8;
    const double kPathOpsComparisonEpsilon = 1e-12;
    const double kPathOpsAreaEpsilon = 1e-10;
    const double kPathOpsRebuildFallbackEpsilon = 1e-8;
    const double kSearchPolyComparisonEpsilon = 1e-12;
    const double kOffsetRebuildFallbackEpsilon = 1e-8;
    const double kSupportPlaneHoleDistanceWeight = 4.0;
    const double kSupportPlaneOnPlaneEpsilon = 1e-10;
}  // namespace Geometry
