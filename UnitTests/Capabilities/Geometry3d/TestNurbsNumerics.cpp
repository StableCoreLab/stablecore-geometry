#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include "Geometry.h"
#include "../../../Source/Core/Detail/SurfaceProjectionSolver.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToSurface;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;

namespace
{
    TEST(NurbsNumericsTest, ValidGeometryRejectsInvalidStructuralTolerance)
    {
        const Geometry::SCLineCurve3d line = Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection({0, 0, 0}, {1, 0, 0}), {0, 1});
        const SCNurbsCurve3d curve(1, {{0, 0, 0}, {1, 0, 0}}, {0, 0, 1, 1});
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
            {0, 0, 1, 1}, {0, 0, 1, 1});
        ASSERT_TRUE(line.IsValid());
        ASSERT_TRUE(curve.IsValid());
        ASSERT_TRUE(surface.IsValid());

        Geometry::SCGeometryTolerance3d invalid{};
        invalid.distanceEpsilon = -1.0;
        EXPECT_FALSE(line.IsValid(invalid));
        EXPECT_FALSE(curve.IsValid(invalid));
        EXPECT_FALSE(surface.IsValid(invalid));
        invalid = {};
        invalid.angleEpsilon = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(line.IsValid(invalid));
        EXPECT_FALSE(curve.IsValid(invalid));
        EXPECT_FALSE(surface.IsValid(invalid));
        invalid = {};
        invalid.parameterEpsilon = -1.0;
        EXPECT_FALSE(line.IsValid(invalid));
        EXPECT_FALSE(curve.IsValid(invalid));
        EXPECT_FALSE(surface.IsValid(invalid));
        invalid = {};
        invalid.boxPadding = std::numeric_limits<double>::infinity();
        EXPECT_FALSE(line.IsValid(invalid));
        EXPECT_FALSE(curve.IsValid(invalid));
        EXPECT_FALSE(surface.IsValid(invalid));
    }

    TEST(NurbsNumericsTest, ExtremeKnotSpanKeepsActualPositionsAndCacheKeysConsistent)
    {
        const double maximum=std::numeric_limits<double>::max();
        const std::vector<double> knots{-maximum,-maximum,maximum,maximum};
        const SCNurbsSurface surface(1,1,2,2,{{0,0,1},{2,0,1},{0,2,1},{2,2,1}},knots,{0,0,1,1});
        ASSERT_TRUE(surface.IsValid());
        Geometry::Detail::SurfaceSearchBudget cache;
        cache.PreparePositionCache(surface);
        for (const auto [u,x]:{std::pair{-maximum,0.0},std::pair{0.0,1.0},std::pair{maximum,2.0}}) {
            const auto actual=surface.PointAt(u,0.5);
            ASSERT_TRUE(actual.IsValid());
            EXPECT_DOUBLE_EQ(actual.x,x);
            EXPECT_DOUBLE_EQ(actual.y,1);
            EXPECT_DOUBLE_EQ(actual.z,1);
            EXPECT_EQ(cache.Position(surface,u,0.5),actual);
        }
        EXPECT_EQ(cache.statistics.positionEvaluations,3u);
        EXPECT_EQ(cache.Position(surface,0,0.5),surface.PointAt(0,0.5));
        EXPECT_EQ(cache.statistics.positionEvaluations,3u);
        const SCNurbsCurve3d curve(1,{{0,0,0},{2,0,0}},knots);
        ASSERT_TRUE(curve.IsValid());
        const auto eval=curve.Evaluate(0,1);
        ASSERT_TRUE(eval.point.IsValid());
        EXPECT_DOUBLE_EQ(eval.point.x,1);
        EXPECT_GT(eval.firstDerivative.x,0);
        EXPECT_TRUE(std::isfinite(eval.firstDerivative.x));
        EXPECT_FALSE(surface.PointAt(std::numeric_limits<double>::infinity(),0.5).IsValid());
        EXPECT_FALSE(curve.PointAt(std::numeric_limits<double>::infinity()).IsValid());
        EXPECT_FALSE(SCNurbsSurface{}.PointAt(0,0).IsValid());
        EXPECT_FALSE(SCNurbsCurve3d{}.PointAt(0).IsValid());
    }

    class PointOnlyPlaneTouchCurve final : public Geometry::ISCCurve3d
    {
    public:
        explicit PointOnlyPlaneTouchCurve(bool invalidFirstOrder, double oscillations = 1.0,
                                          bool mismatchedZeroOrder = false)
            : invalidFirstOrder_(invalidFirstOrder), oscillations_(oscillations),
              mismatchedZeroOrder_(mismatchedZeroOrder)
        {
        }

        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald ParameterRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] bool IsClosed(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] bool IsPeriodic(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override
        {
            const double height = std::cos(2.0 * std::numbers::pi * oscillations_ * parameter);
            return {parameter, 0.0, height * height};
        }
        [[nodiscard]] Geometry::SCCurveEval3d Evaluate(double parameter, int derivativeOrder = 1) const override
        {
            Geometry::SCCurveEval3d eval{};
            eval.point = PointAt(parameter);
            if (mismatchedZeroOrder_ && derivativeOrder == 0)
            {
                eval.point.z += 1.0;
            }
            if (invalidFirstOrder_ && derivativeOrder > 0)
            {
                eval.derivativeOrder = -1;
            }
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 0.0, 1.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCCurve3d> Clone() const override
        {
            return std::make_unique<PointOnlyPlaneTouchCurve>(*this);
        }

    private:
        bool invalidFirstOrder_;
        double oscillations_;
        bool mismatchedZeroOrder_;
    };

    class PointOnlyPatchSurface final : public Geometry::ISCSurface
    {
    public:
        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald URange() const override { return {0.0, 1.0}; }
        [[nodiscard]] Geometry::SCIntervald VRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] SCPoint3d PointAt(double u, double v) const override { return {u, v, 0.0}; }
        [[nodiscard]] Geometry::SCSurfaceEval3d Evaluate(double u, double v, int = 1) const override
        {
            Geometry::SCSurfaceEval3d eval{};
            eval.point = {u, v, 1.0};
            eval.derivativeOrder = -1;
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 1.0, 0.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCSurface> Clone() const override
        {
            return std::make_unique<PointOnlyPatchSurface>(*this);
        }
    };

    class MisleadingCurveEval final : public Geometry::ISCCurve3d
    {
    public:
        [[nodiscard]] bool IsValid(const Geometry::SCGeometryTolerance3d& = {}) const override { return true; }
        [[nodiscard]] Geometry::SCIntervald ParameterRange() const override { return {0.0, 1.0}; }
        [[nodiscard]] bool IsClosed(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] bool IsPeriodic(const Geometry::SCGeometryTolerance3d& = {}) const override { return false; }
        [[nodiscard]] SCPoint3d PointAt(double parameter) const override { return {parameter, 0.0, 1.0}; }
        [[nodiscard]] Geometry::SCCurveEval3d Evaluate(double parameter, int = 1) const override
        {
            Geometry::SCCurveEval3d eval{};
            eval.point = {parameter, 0.0, 0.0};
            eval.firstDerivative = {1.0, 0.0, 0.0};
            eval.derivativeOrder = 1;
            return eval;
        }
        [[nodiscard]] Geometry::SCBox3d Bounds() const override
        {
            return Geometry::SCBox3d::FromMinMax({0.0, 0.0, 1.0}, {1.0, 0.0, 1.0});
        }
        [[nodiscard]] std::unique_ptr<Geometry::ISCCurve3d> Clone() const override
        {
            return std::make_unique<MisleadingCurveEval>(*this);
        }
    };

    TEST(NurbsNumericsTest, CompositeSurfacesRejectInvalidToleranceWithPermissiveComponents)
    {
        const auto curve = std::make_shared<PointOnlyPlaneTouchCurve>(false);
        const Geometry::SCRuledSurface ruled(curve, curve);
        const Geometry::SCOffsetSurface offset(std::make_shared<PointOnlyPatchSurface>(), 1.0);
        ASSERT_TRUE(ruled.IsValid());
        ASSERT_TRUE(offset.IsValid());

        Geometry::SCGeometryTolerance3d invalid{};
        invalid.angleEpsilon = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(ruled.IsValid(invalid));
        EXPECT_FALSE(offset.IsValid(invalid));
    }

    TEST(NurbsNumericsTest, BrepContainersRejectInvalidToleranceWithPermissiveGeometry)
    {
        const Geometry::SCBrepEdge edge(std::make_shared<PointOnlyPlaneTouchCurve>(false), 0, 1);
        const Geometry::SCBrepFace face(
            std::make_shared<PointOnlyPatchSurface>(),
            Geometry::SCBrepLoop({Geometry::SCBrepCoedge(0, false)}));
        const Geometry::SCBrepShell shell({face});
        const Geometry::SCBrepBody body(
            {Geometry::SCBrepVertex({0, 0, 0}), Geometry::SCBrepVertex({1, 0, 0})},
            {edge}, {shell});
        ASSERT_TRUE(edge.IsValid());
        ASSERT_TRUE(edge.IsValid(2));
        ASSERT_TRUE(face.IsValid());
        ASSERT_TRUE(face.IsValidWithEdgeCount(1));
        ASSERT_TRUE(shell.IsValid());
        ASSERT_TRUE(shell.IsValid(1));
        ASSERT_TRUE(body.IsValid());

        Geometry::SCGeometryTolerance3d invalid{};
        invalid.angleEpsilon = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(edge.IsValid(invalid));
        EXPECT_FALSE(edge.IsValid(2, invalid));
        EXPECT_FALSE(face.IsValid(invalid));
        EXPECT_FALSE(face.IsValidWithEdgeCount(1, invalid));
        EXPECT_FALSE(shell.IsValid(invalid));
        EXPECT_FALSE(shell.IsValid(1, invalid));
        EXPECT_FALSE(body.IsValid(invalid));
    }

    TEST(NurbsNumericsTest, PointOnlyCurvePlaneIntersectionFindsQuarterTouch)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
        for (const bool invalidFirstOrder : {false, true})
        {
            SCOPED_TRACE(invalidFirstOrder);
            const PointOnlyPlaneTouchCurve curve(invalidFirstOrder);
            const auto intersection = Intersect(plane, curve);
            ASSERT_TRUE(intersection.intersects);
            EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-12);
            EXPECT_TRUE(intersection.point.AlmostEquals(curve.PointAt(0.25), 1e-12));
        }
    }

    TEST(NurbsNumericsTest, PointOnlyCurvePlaneIntersectionKeepsFullDomainSamples)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        for (const bool invalidFirstOrder : {false, true})
        {
            const PointOnlyPlaneTouchCurve curve(invalidFirstOrder, 4.0, true);
            const auto hit = Intersect(plane, curve);
            ASSERT_TRUE(hit.intersects);
            EXPECT_NEAR(hit.curveParameter, 1.0 / 16.0, 1e-12);
            EXPECT_TRUE(hit.point.AlmostEquals(curve.PointAt(hit.curveParameter), 1e-12));
        }
    }

    TEST(NurbsNumericsTest, PointOnlyCurveProjectionUsesPointAt)
    {
        const PointOnlyPlaneTouchCurve curve(true, 1.0, true);
        const SCPoint3d point = curve.PointAt(0.25);
        const auto projection = ProjectPointToCurve(point, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_TRUE(projection.point.AlmostEquals(curve.PointAt(projection.parameter), 1e-12));
        EXPECT_NEAR(projection.distanceSquared, 0.0, 1e-12);
    }

    TEST(NurbsNumericsTest, CurvePlaneIntersectionRejectsMismatchedEvaluationPoint)
    {
        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{}, SCVector3d{0.0, 0.0, 1.0});
        EXPECT_FALSE(Intersect(plane, MisleadingCurveEval{}).intersects);
    }

    TEST(NurbsNumericsTest, PointOnlySurfaceProjectionUsesPointAt)
    {
        const PointOnlyPatchSurface surface;
        const auto projection = ProjectPointToSurface(SCPoint3d{0.25, 0.75, 0.1}, surface);
        ASSERT_TRUE(projection.success);
        EXPECT_TRUE(projection.point.AlmostEquals(surface.PointAt(projection.u, projection.v), 1e-12));
        EXPECT_NEAR(projection.u, 0.25, 1e-8);
        EXPECT_NEAR(projection.v, 0.75, 1e-8);
        EXPECT_NEAR(projection.distanceSquared, 0.01, 1e-10);
    }

}  // namespace
