#include <gtest/gtest.h>

#include "sdk/Geometry.h"

using geometry::sdk::LineSegment3d;
using geometry::sdk::Plane;
using geometry::sdk::Point3d;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::Section;
using geometry::sdk::Vector3d;

TEST(Section3dGapTest, NonPlanarDominantSectionGraphRemainsOpen)
{
    const PolyhedronBody body;
    const Plane cuttingPlane = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.1},
        Vector3d{0.0, 0.0, 1.0});

    const auto section = Section(body, cuttingPlane);
    (void)section;
    GTEST_SKIP() << "Known 3D gap: ambiguous non-manifold contour stitching semantics in non-planar-dominant sections are still open.";
}

TEST(Section3dGapTest, FaceMergeSemanticsAfterSectionRemainsOpen)
{
    const PolyhedronBody body;
    const Plane cuttingPlane = Plane::FromPointAndNormal(
        Point3d{0.0, 0.0, 0.0},
        Vector3d{0.0, 0.0, 1.0});

    const auto section = Section(body, cuttingPlane);
    (void)section;
    GTEST_SKIP() << "Known 3D gap: section face-merge policy for ambiguous coplanar fragments beyond adjacent coplanar face union subset is not closed yet.";
}
