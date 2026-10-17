#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <vector>

#include "Core/Projection.h"
#include "Core/Intersection.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"

static_assert(std::is_base_of_v<Geometry::ISCCurve3d, Geometry::SCCurveOnSurface>);

namespace
{
    TEST(CurveOnSurfaceInterfaceTest, GenericCurveUsesUvSegmentParametersAndSurfaceDerivatives)
    {
        auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 4}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface composed(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {1, 1}}));
        const Geometry::ISCCurve3d& curve = composed;

        ASSERT_TRUE(curve.IsValid());
        EXPECT_DOUBLE_EQ(curve.ParameterRange().min, 0.0);
        EXPECT_DOUBLE_EQ(curve.ParameterRange().max, 2.0);
        EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals(surface->PointAt(0.5, 0.0), 1e-12));
        EXPECT_TRUE(curve.PointAt(1.5).AlmostEquals(surface->PointAt(1.0, 0.5), 1e-12));
        EXPECT_TRUE(composed.UvAt(1.5).AlmostEquals({1.0, 0.5}, 1e-12));
        EXPECT_TRUE(composed.PointAt(1).AlmostEquals(surface->PointAt(1.0, 0.0), 1e-12));
        EXPECT_TRUE(composed.PointAt(1u).AlmostEquals(surface->PointAt(1.0, 0.0), 1e-12));

        const auto first = curve.Evaluate(0.5, 1);
        const auto second = curve.Evaluate(1.5, 1);
        ASSERT_TRUE(first.IsValid());
        ASSERT_TRUE(second.IsValid());
        EXPECT_TRUE(first.firstDerivative.AlmostEquals({1, 0, 0}, 1e-12));
        EXPECT_TRUE(second.firstDerivative.AlmostEquals({0, 1, 4}, 1e-12));

        const auto projected = Geometry::ProjectPointToCurve(curve.PointAt(1.5), curve);
        ASSERT_TRUE(projected.success);
        EXPECT_TRUE(projected.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        const auto line = Geometry::SCLine3d::FromOriginAndDirection(
            {1.0, 0.5, 3.0}, {0.0, 0.0, -1.0});
        const auto lineHit = Geometry::Intersect(line, curve);
        ASSERT_TRUE(lineHit.intersects);
        EXPECT_TRUE(lineHit.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        const auto plane = Geometry::SCPlane::FromPointAndNormal(
            {0.0, 0.0, 2.0}, {0.0, 0.0, 1.0});
        const auto planeHit = Geometry::Intersect(plane, curve);
        ASSERT_TRUE(planeHit.intersects);
        EXPECT_TRUE(planeHit.point.AlmostEquals(curve.PointAt(1.5), 1e-7));
        ASSERT_TRUE(curve.Bounds().IsValid());
        EXPECT_GE(curve.Bounds().MaxPoint().z, 4.0);

        const auto clone = curve.Clone();
        ASSERT_NE(clone, nullptr);
        EXPECT_TRUE(clone->PointAt(1.5).AlmostEquals(curve.PointAt(1.5), 1e-12));
        const auto* clonedComposition = dynamic_cast<const Geometry::SCCurveOnSurface*>(clone.get());
        ASSERT_NE(clonedComposition, nullptr);
        EXPECT_NE(clonedComposition->SupportSurface(), composed.SupportSurface());
    }

    TEST(CurveOnSurfaceInterfaceTest, ClosedCurveAndInvalidUvDomainHaveExplicitContracts)
    {
        const auto plane = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlaneSurface::FromPlane(
                Geometry::SCPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}), {-1, 1}, {-1, 1}));
        const Geometry::SCCurveOnSurface closed(
            plane, Geometry::SCPolyline2d({{0, 0}, {1, 0}, {0, 1}}, Geometry::SCPolylineClosure::Closed));
        ASSERT_TRUE(closed.IsValid());
        EXPECT_TRUE(closed.IsClosed());
        EXPECT_FALSE(closed.IsPeriodic());
        EXPECT_TRUE(closed.PointAt(3.0).AlmostEquals(closed.PointAt(0.0), 1e-12));
        EXPECT_FALSE(closed.UvAt(-0.1).IsValid());

        auto curvedUv = std::make_shared<Geometry::SCArcSegment2d>(
            Geometry::SCPoint2d{0, 0}, 1.0, 0.0, Geometry::kPi * 0.5);
        const Geometry::SCCurveOnSurface curved(
            plane, Geometry::SCPolyline2d(
                std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{curvedUv},
                Geometry::SCPolylineClosure::Open));
        EXPECT_FALSE(curved.IsValid());

        const auto surface = std::make_shared<Geometry::SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<Geometry::SCPoint3d>{{0, 0, 0}, {1, 0, 4}, {0, 1, 4}, {1, 1, 0}},
            std::vector<double>{0, 0, 1, 1}, std::vector<double>{0, 0, 1, 1});
        const Geometry::SCCurveOnSurface outside(
            surface, Geometry::SCPolyline2d({{0, 0}, {2, 0}}));
        EXPECT_FALSE(outside.IsValid());

        const Geometry::SCCurveOnSurface diagonal(
            surface, Geometry::SCPolyline2d({{0, 0}, {1, 1}}));
        ASSERT_TRUE(diagonal.IsValid());
        ASSERT_TRUE(diagonal.Bounds().IsValid());
        EXPECT_GT(diagonal.PointAt(0.5).z, 0.0);
        EXPECT_GE(diagonal.Bounds().MaxPoint().z, diagonal.PointAt(0.5).z);
    }
}
