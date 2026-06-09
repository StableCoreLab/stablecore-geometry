#pragma once

#include <array>

#include "Export/GeometryExport.h"

namespace Geometry
{
    constexpr double kDefaultEpsilon = 1e-9;
    extern GEOMETRY_API const double kPi;
    extern GEOMETRY_API const double kTwoPi;
    extern GEOMETRY_API const std::array<double, 4> kArcBoundsCriticalAngles;

    // Shared topology matching for cross-face representative grouping.
    extern GEOMETRY_API const double kSharedTopologyMatchEpsilon;

    // Loop cleanup removes consecutive duplicate vertices and trims closed
    // loops.
    extern GEOMETRY_API const double kLoopCleanupEpsilon;

    // Representative-id aggregation uses a looser match than exact topology
    // comparison so near-equal inputs collapse to the same shared target.
    extern GEOMETRY_API const double kRepresentativeMatchEpsilon;

    // Explicit call-site name for the representative grouping pass.
    extern GEOMETRY_API const double kRepresentativeGroupingEpsilon;

    // BRep vertex deduplication uses representative-style matching when merging
    // converted vertices onto shared output topology.
    extern GEOMETRY_API const double kBrepVertexDedupEpsilon;

    // Plane projection uses the default distance epsilon when building 2D
    // trims.
    extern GEOMETRY_API const double kPlaneProjectionEpsilon;

    // Body-boolean internals use the default geometric epsilon for option
    // fallbacks and deterministic result ordering.
    extern GEOMETRY_API const double kBodyBooleanDefaultEpsilon;

    // 2D/3D intersection internals use the default geometric epsilon for their
    // local degeneracy, projection, and update checks.
    extern GEOMETRY_API const double kIntersectionDefaultEpsilon;

    // Projection helpers use the default geometric epsilon for endpoint and
    // parameter checks.
    extern GEOMETRY_API const double kProjectionDefaultEpsilon;

    // Path operations use the default geometric epsilon for clipping and
    // normalization helpers.
    extern GEOMETRY_API const double kPathOpsDefaultEpsilon;

    // Offset helpers use the default geometric epsilon for degeneracy checks
    // and polygon rebuild fallbacks.
    extern GEOMETRY_API const double kOffsetDefaultEpsilon;

    // Axis operations use the default geometric epsilon for segment and arc
    // parameterization checks.
    extern GEOMETRY_API const double kAxisOpsDefaultEpsilon;

    // Shared geometry algorithms use the default geometric epsilon for
    // parameter clamping checks.
    extern GEOMETRY_API const double kAlgorithmsDefaultEpsilon;

    // Arc-segment helpers use the default geometric epsilon for validity and
    // arc endpoint checks.
    extern GEOMETRY_API const double kArcSegmentDefaultEpsilon;

    // Healing and repair helpers use the default geometric epsilon for fallback
    // validity checks.
    extern GEOMETRY_API const double kHealingDefaultEpsilon;

    // Mesh conversion helpers use the default geometric epsilon for coordinate
    // comparisons during ordering and deduplication.
    extern GEOMETRY_API const double kMeshConversionDefaultEpsilon;

    // Shape helpers use the default geometric epsilon for signed-area checks.
    extern GEOMETRY_API const double kShapeOpsDefaultEpsilon;

    // Transform helpers use the default geometric epsilon for invertibility and
    // zero-length checks.
    extern GEOMETRY_API const double kTransformDefaultEpsilon;

    // Section helpers use the default geometric epsilon for plane and axis
    // normalization checks.
    extern GEOMETRY_API const double kSectionDefaultEpsilon;

    // Search helpers use the default geometric epsilon for clustering and
    // candidate ranking.
    extern GEOMETRY_API const double kSearchPolyDefaultEpsilon;

    // Boolean helpers use these floors for their internal tolerance fallback,
    // area filtering, and rebuild retry logic.
    extern GEOMETRY_API const double kBooleanComparisonEpsilon;
    extern GEOMETRY_API const double kBooleanAreaEpsilon;
    extern GEOMETRY_API const double kBooleanRebuildFallbackEpsilon;

    // Path helpers use these floors for parameter compacting, score comparison,
    // and polygon rebuild retry logic.
    extern GEOMETRY_API const double kPathOpsComparisonEpsilon;
    extern GEOMETRY_API const double kPathOpsAreaEpsilon;
    extern GEOMETRY_API const double kPathOpsRebuildFallbackEpsilon;

    // Search helpers use a tight comparison epsilon when ranking candidate
    // branches and rings.
    extern GEOMETRY_API const double kSearchPolyComparisonEpsilon;

    // Offset helpers retry line-based rebuilding with a slightly looser
    // epsilon.
    extern GEOMETRY_API const double kOffsetRebuildFallbackEpsilon;

    // Hole distances are weighted slightly higher during support-plane scoring
    // so hole-dominated faces prefer the plane implied by their interior loop.
    extern GEOMETRY_API const double kSupportPlaneHoleDistanceWeight;

    // Support-plane scoring treats distances below this threshold as
    // effectively on-plane when comparing candidate refit planes.
    extern GEOMETRY_API const double kSupportPlaneOnPlaneEpsilon;
}  // namespace Geometry

