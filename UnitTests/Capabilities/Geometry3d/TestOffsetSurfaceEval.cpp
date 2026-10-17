#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCOffsetSurface.h"

TEST(OffsetSurfaceEvalTest, CurvedBaseChangesOffsetTangentsAndNormal)
{
    const Geometry::SCNurbsSurface base(2, 1, 3, 2,
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {1.0, 0.0, 1.0},
         {0.0, 1.0, 0.0}, {0.5, 1.0, 0.0}, {1.0, 1.0, 1.0}},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
    ASSERT_TRUE(base.IsValid());
    const auto surface = Geometry::SCOffsetSurface::FromSurface(base, 0.2);
    const double u = 0.37;
    const double v = 0.43;
    const auto eval = surface.Evaluate(u, v, 1);
    ASSERT_TRUE(eval.IsValid());

    const double factor = std::pow(1.0 + 4.0 * u * u, 1.5);
    const Geometry::SCVector3d expectedU{1.0 - 0.4 / factor, 0.0,
                                          2.0 * u - 0.8 * u / factor};
    EXPECT_TRUE(eval.derivativeU.AlmostEquals(expectedU, 1e-5));
    EXPECT_TRUE(eval.derivativeV.AlmostEquals(Geometry::SCVector3d{0.0, 1.0, 0.0}, 1e-5));
    EXPECT_TRUE(eval.normal.AlmostEquals(
        Geometry::Cross(eval.derivativeU, eval.derivativeV), 1e-5));
    EXPECT_TRUE(eval.point.AlmostEquals(surface.PointAt(u, v), 1e-12));

    for (double boundaryU : {0.0, 1.0})
    {
        const auto boundary = surface.Evaluate(boundaryU, v, 1);
        ASSERT_TRUE(boundary.IsValid());
        const double boundaryFactor = std::pow(1.0 + 4.0 * boundaryU * boundaryU, 1.5);
        const Geometry::SCVector3d boundaryExpected{
            1.0 - 0.4 / boundaryFactor, 0.0,
            2.0 * boundaryU - 0.8 * boundaryU / boundaryFactor};
        EXPECT_TRUE(boundary.derivativeU.AlmostEquals(boundaryExpected, 1e-4));
    }

    const auto zeroOffset = Geometry::SCOffsetSurface::FromSurface(base, 0.0);
    EXPECT_TRUE(zeroOffset.Evaluate(u, v, 1).derivativeU.AlmostEquals(
        base.Evaluate(u, v, 1).derivativeU, 1e-12));

    const auto invalidOffset = Geometry::SCOffsetSurface::FromSurface(
        base, std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(invalidOffset.IsValid());
    EXPECT_FALSE(invalidOffset.Evaluate(u, v, 1).IsValid());
}
