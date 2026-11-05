#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Core/Measure.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/SCNurbsSurface.h"

namespace
{
    class ParabolaUv : public Geometry::ISCCurve2d
    {
    public:
        bool acrossKnot{false};
        bool IsValid(double eps = Geometry::kDefaultEpsilon) const override
        { return std::isfinite(eps) && eps > 0.0; }
        Geometry::SCIntervald ParameterRange() const override { return {2.0, 6.0}; }
        Geometry::SCCurveEval2d Evaluate(double t, int order = 1) const override
        {
            Geometry::SCCurveEval2d r{};
            r.point.x = std::numeric_limits<double>::quiet_NaN();
            if (!ParameterRange().Contains(t, 0.0)) return r;
            const double s = (t - 2.0) / 4.0;
            r.point = acrossKnot ? Geometry::SCPoint2d{s*s, s} : Geometry::SCPoint2d{s, 4*s*(1-s)};
            r.derivativeOrder = std::clamp(order, 0, 1);
            if (r.derivativeOrder)
                r.firstDerivative = acrossKnot ? Geometry::SCVector2d{s/2, 0.25} : Geometry::SCVector2d{0.25, 1-2*s};
            return r;
        }
        Geometry::SCBox2d Bounds() const override { return {{0,0},{1,1}}; }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override
        { return std::make_unique<ParabolaUv>(*this); }
    };
    class SplitParabolaUv final : public ParabolaUv
    {
    public:
        bool gap{false};
        std::size_t PieceCount() const override { return 2; }
        Geometry::SCIntervald PieceRange(std::size_t i) const override
        {
            if (i >= 2) throw std::out_of_range("SplitParabolaUv");
            return i == 0 ? Geometry::SCIntervald{2,3} : Geometry::SCIntervald{gap ? 3.5 : 3.0,6};
        }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override
        { return std::make_unique<SplitParabolaUv>(*this); }
    };
    class LinearNativeUv final : public ParabolaUv
    {
    public:
        double width{1.0};
        Geometry::SCIntervald ParameterRange() const override { return {0.0, width}; }
        Geometry::SCCurveEval2d Evaluate(double t, int order = 1) const override
        {
            if (!ParameterRange().Contains(t, 0.0))
                return {{std::numeric_limits<double>::quiet_NaN(), 0}, {}, 0};
            return {{t/width, 0}, {1/width, 0}, std::clamp(order, 0, 1)};
        }
        Geometry::SCBox2d Bounds() const override { return {{0,0},{1,0}}; }
        bool IsLinearPiece(std::size_t index) const override { return index == 0; }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override
        { return std::make_unique<LinearNativeUv>(*this); }
    };
    auto Plane()
    {
        return std::make_shared<Geometry::SCPlaneSurface>(Geometry::SCPlaneSurface::FromPlane(
            Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1})));
    }
}

TEST(GenericUvCurve3dTest, LinearPlaneIntersectionUsesNativeParameterTolerance)
{
    const auto support = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1}),
        Geometry::SCVector3d{1,0,0}, Geometry::SCVector3d{0,1,0},
        Geometry::SCIntervald{0,1}, Geometry::SCIntervald{0,1});
    Geometry::SCGeometryTolerance3d tolerance;
    tolerance.parameterEpsilon = 1e-6;
    for (const double width : {1000.0, 1.0, 1e-3})
    {
        LinearNativeUv uv; uv.width = width;
        const Geometry::SCCurveOnSurface curve(support, uv);
        ASSERT_TRUE(curve.IsValid(tolerance));
        for (const double end : {0.0, width})
            for (const double factor : {5.0, 0.5})
            {
                SCOPED_TRACE(::testing::Message() << width << ',' << end << ',' << factor);
                const double parameter = end + (end == 0.0 ? -1 : 1)*factor*tolerance.parameterEpsilon;
                const auto plane = Geometry::SCPlane::FromPointAndNormal({parameter/width,0,0}, {1,0,0});
                const auto result = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
                EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Verified);
                if (factor < 1.0)
                {
                    ASSERT_EQ(result.hits.size(), 1u);
                    EXPECT_NEAR(result.hits.front().curveParameter, parameter, 1e-12);
                }
                else
                {
                    EXPECT_TRUE(result.hits.empty());
                    EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::NoResult);
                }
            }
        tolerance.parameterEpsilon = 0.0;
        const auto plane = Geometry::SCPlane::FromPointAndNormal({0.5,0,0}, {1,0,0});
        const auto exact = Geometry::IntersectAllPlaneCurveWithStatus(plane, curve, tolerance);
        ASSERT_EQ(exact.hits.size(), 1u);
        EXPECT_NEAR(exact.hits.front().curveParameter, width/2, 1e-12);
        tolerance.parameterEpsilon = 1e-6;
    }
}

TEST(GenericUvCurve3dTest, LinearLineIntersectionUsesNativeParameterTolerance)
{
    const auto support = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1}),
        Geometry::SCVector3d{1,0,0}, Geometry::SCVector3d{0,1,0},
        Geometry::SCIntervald{0,1}, Geometry::SCIntervald{0,1});
    Geometry::SCGeometryTolerance3d tolerance;
    tolerance.parameterEpsilon = 1e-6;
    // 允许端点夹取后的空间残差，独立检验原参数域的准入门槛。
    tolerance.distanceEpsilon = 1e-2;
    for (const double width : {1000.0, 1.0, 1e-3})
    {
        LinearNativeUv uv; uv.width = width;
        const Geometry::SCCurveOnSurface curve(support, uv);
        ASSERT_TRUE(curve.IsValid(tolerance));
        for (const double end : {0.0, width})
            for (const double factor : {5.0, 0.5})
            {
                SCOPED_TRACE(::testing::Message() << width << ',' << end << ',' << factor);
                const double parameter = end + (end == 0.0 ? -1 : 1)*factor*tolerance.parameterEpsilon;
                const auto line = Geometry::SCLine3d::FromOriginAndDirection({parameter/width,0,-1}, {0,0,1});
                const auto result = Geometry::IntersectAllLineCurveWithStatus(line, curve, tolerance);
                // 直线内核既有完整性仍为 Unknown，本回归只检查参数域准入。
                EXPECT_EQ(result.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
                if (factor < 1.0)
                {
                    ASSERT_EQ(result.hits.size(), 1u);
                    EXPECT_DOUBLE_EQ(result.hits.front().curveParameter, end);
                }
                else
                {
                    EXPECT_TRUE(result.hits.empty());
                    EXPECT_EQ(result.state, Geometry::SCGeometryOperationState::Unknown);
                }
            }
        tolerance.parameterEpsilon = 0.0;
        const auto line = Geometry::SCLine3d::FromOriginAndDirection({0.5,0,-1}, {0,0,1});
        const auto exact = Geometry::IntersectAllLineCurveWithStatus(line, curve, tolerance);
        ASSERT_EQ(exact.hits.size(), 1u);
        EXPECT_NEAR(exact.hits.front().curveParameter, width/2, 1e-12);
        tolerance.parameterEpsilon = 1e-6;
    }
}

TEST(GenericUvCurve3dTest, PreservesNativeDomainDerivativeBoundsAndClone)
{
    ParabolaUv uv;
    const Geometry::SCCurveOnSurface curve(Plane(), uv);
    ASSERT_TRUE(curve.IsValid());
    EXPECT_DOUBLE_EQ(curve.ParameterRange().min, 2.0);
    EXPECT_DOUBLE_EQ(curve.ParameterRange().max, 6.0);
    EXPECT_DOUBLE_EQ(curve.UvParameterAt(0, 0.5), 4.0);
    EXPECT_EQ(curve.TryUvCurve(), nullptr);
    EXPECT_THROW((void)curve.UvCurve(), std::logic_error);
    EXPECT_THROW((void)curve.UvPointAt(0), std::logic_error);
    EXPECT_THROW((void)curve.PointCount(), std::logic_error);
    EXPECT_THROW((void)curve.PointAt(std::size_t{0}), std::logic_error);
    const auto middle = curve.Evaluate(4.0);
    EXPECT_TRUE(middle.point.AlmostEquals({0.5,1,0}, 1e-12));
    EXPECT_TRUE(middle.firstDerivative.AlmostEquals({0.25,0,0}, 1e-12));
    EXPECT_EQ(middle.derivativeOrder, 1);
    EXPECT_NEAR(curve.Bounds().MaxPoint().y, 1.0, 1e-12);
    const auto copy = curve.Clone();
    uv.acrossKnot = true;
    EXPECT_TRUE(curve.PointAt(4.0).AlmostEquals({0.5,1,0}, 1e-12));
    EXPECT_TRUE(copy->PointAt(4.0).AlmostEquals({0.5,1,0}, 1e-12));
    EXPECT_FALSE(curve.PointAt(1.0).IsValid());
    EXPECT_FALSE(curve.IsClosed());
    EXPECT_NEAR(Geometry::Length(curve), std::sqrt(17.0)/2 + std::asinh(4.0)/8, 1e-6);
}

TEST(GenericUvCurve3dTest, UsesActualUvForProjectionAndIntersectionsWithoutChordProof)
{
    const Geometry::SCCurveOnSurface curve(Plane(), SplitParabolaUv{});
    const Geometry::ISCCurve3d& base = curve;
    const auto specialized = Geometry::ProjectPointToCurveOnSurface({0.5,1,2}, curve);
    ASSERT_TRUE(specialized.success);
    EXPECT_EQ(specialized.segmentIndex, 1u);
    EXPECT_NEAR(specialized.segmentParameter, 1.0/3, 1e-5);
    const auto projected = Geometry::ProjectPointToCurve({0.5,1,2}, base);
    ASSERT_TRUE(projected.success);
    EXPECT_NEAR(projected.parameter, 4.0, 1e-5);
    EXPECT_TRUE(projected.point.AlmostEquals({0.5,1,0}, 1e-6));
    const auto section = Geometry::SCPlane::FromPointAndNormal({0.5,0,0}, {1,0,0});
    const auto hits = Geometry::IntersectAllPlaneCurveWithStatus(section, base);
    ASSERT_FALSE(hits.hits.empty());
    EXPECT_NEAR(hits.hits.front().curveParameter, 4.0, 1e-6);
    EXPECT_TRUE(hits.hits.front().point.AlmostEquals({0.5,1,0}, 1e-6));
    EXPECT_EQ(hits.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    const auto chordPlane = Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,1,0});
    const auto endpoints = Geometry::IntersectAllPlaneCurveWithStatus(chordPlane, base);
    EXPECT_NE(endpoints.state, Geometry::SCGeometryOperationState::NonUnique);
    EXPECT_EQ(endpoints.evidence.negativeCompleteness, Geometry::SCGeometryEvidenceState::Unknown);
    const auto line = Geometry::SCLine3d::FromOriginAndDirection({0.5,1,-2}, {0,0,1});
    const auto lineHit = Geometry::Intersect(line, base);
    ASSERT_TRUE(lineHit.intersects);
    EXPECT_NEAR(lineHit.curveParameter, 4.0, 1e-5);
}

TEST(GenericUvCurve3dTest, EvaluatesNonlinearUvAcrossSurfaceKnot)
{
    auto surface = std::make_shared<Geometry::SCNurbsSurface>(1, 1, 3, 2,
        std::vector<Geometry::SCPoint3d>{{0,0,0},{0.5,0,1},{1,0,0},{0,1,0},{0.5,1,1},{1,1,0}},
        std::vector<double>{0,0,0.5,1,1}, std::vector<double>{0,0,1,1});
    ParabolaUv uv; uv.acrossKnot = true;
    const Geometry::SCCurveOnSurface curve(surface, uv);
    ASSERT_TRUE(curve.IsValid());
    const double s = std::sqrt(0.5);
    EXPECT_TRUE(curve.PointAt(2.0 + 4*s).AlmostEquals({0.5,s,1}, 1e-10));
    const double expected = std::sqrt(21.0)/2 + std::asinh(std::sqrt(20.0))/(2*std::sqrt(20.0));
    EXPECT_NEAR(Geometry::Length(curve), expected, 1e-5);
}

TEST(GenericUvCurve3dTest, RejectsParameterPartitionGap)
{
    SplitParabolaUv uv;
    uv.gap = true;
    const Geometry::SCCurveOnSurface curve(Plane(), uv);
    EXPECT_FALSE(curve.IsValid());
    EXPECT_FALSE(curve.PointAt(4.0).IsValid());
}

TEST(GenericUvCurve3dTest, ArcUvUsesInteriorExtremumAndActualLength)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(Geometry::SCPoint2d{}, 2.0, 0.0, Geometry::kPi);
    const Geometry::SCCurveOnSurface curve(Plane(), Geometry::SCPolyline2d(
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc}, Geometry::SCPolylineClosure::Open));
    ASSERT_TRUE(curve.IsValid());
    EXPECT_NEAR(curve.Bounds().MaxPoint().y, 2.0, 1e-12);
    EXPECT_TRUE(curve.PointAt(0.5).AlmostEquals({0,2,0}, 1e-12));
    EXPECT_NEAR(Geometry::Length(curve), 2*Geometry::kPi, 1e-10);
    const auto projected = Geometry::ProjectPointToCurve({0,3,0}, curve);
    ASSERT_TRUE(projected.success);
    EXPECT_TRUE(projected.point.AlmostEquals({0,2,0}, 1e-6));
}

TEST(GenericUvCurve3dTest, LengthAvoidsIntermediateQuadratureOverflow)
{
    const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1}),
        Geometry::SCVector3d{5e307,0,0}, Geometry::SCVector3d{0,1,0},
        Geometry::SCIntervald{0,1}, Geometry::SCIntervald{0,1});
    ParabolaUv uv; uv.acrossKnot = true;
    const Geometry::SCCurveOnSurface curve(surface, uv);
    ASSERT_TRUE(curve.IsValid());
    const double length = Geometry::Length(curve);
    EXPECT_TRUE(std::isfinite(length));
    EXPECT_NEAR(length/5e307, 1.0, 1e-12);
}

TEST(GenericUvCurve3dTest, BoundsRejectsUnrepresentableMappedCorners)
{
    const auto surface = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlane::FromPointAndNormal({0,1.4e308,0}, {0,0,1}),
        Geometry::SCVector3d{-5e307,-5e307,0}, Geometry::SCVector3d{-5e307,5e307,0},
        Geometry::SCIntervald{0,1}, Geometry::SCIntervald{0,1});
    const Geometry::SCCurveOnSurface curve(surface, ParabolaUv{});
    ASSERT_TRUE(curve.IsValid());
    ASSERT_TRUE(curve.PointAt(4.0).IsValid());
    EXPECT_FALSE(curve.Bounds().IsValid());
}

TEST(GenericUvCurve3dTest, LengthUsesIncomingDerivativeAtNonlinearPieceEnd)
{
    const auto arc = std::make_shared<Geometry::SCArcSegment2d>(
        Geometry::SCPoint2d{}, 2.0, 0.0, Geometry::kPi/2);
    const auto line = std::make_shared<Geometry::SCLineSegment2d>(arc->EndPoint(), Geometry::SCPoint2d{100,2});
    const Geometry::SCCurveOnSurface curve(Plane(), Geometry::SCPolyline2d(
        std::vector<std::shared_ptr<Geometry::ISCSegment2d>>{arc,line}, Geometry::SCPolylineClosure::Open));
    ASSERT_TRUE(curve.IsValid());
    EXPECT_NEAR(Geometry::Length(curve), Geometry::kPi+100, 1e-8);
}

TEST(GenericUvCurve3dTest, LengthHandlesFiniteDomainWithOverflowingWidth)
{
    class WideUv final : public ParabolaUv
    {
    public:
        Geometry::SCIntervald ParameterRange() const override { return {-1e308,1e308}; }
        Geometry::SCCurveEval2d Evaluate(double t, int order = 1) const override
        {
            return {{t/1e308,0}, {1e-308,0}, std::clamp(order,0,1)};
        }
        Geometry::SCBox2d Bounds() const override { return {{-1,0},{1,0}}; }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override { return std::make_unique<WideUv>(); }
    };
    const Geometry::SCCurveOnSurface curve(Plane(), WideUv{});
    ASSERT_TRUE(curve.IsValid());
    EXPECT_NEAR(Geometry::Length(curve), 2.0, 1e-12);
}

TEST(GenericUvCurve3dTest, LengthScalesDerivativeBeforeOverflowingNorm)
{
    class NarrowUv final : public ParabolaUv
    {
    public:
        double component{1.4e308};
        Geometry::SCIntervald ParameterRange() const override { return {0,1e-308}; }
        Geometry::SCCurveEval2d Evaluate(double t, int order = 1) const override
        {
            const double coordinate = (t/1e-308)*(component*1e-308);
            return {{coordinate,coordinate}, {component,component}, std::clamp(order,0,1)};
        }
        Geometry::SCBox2d Bounds() const override
        { return {{0,0},{component*1e-308,component*1e-308}}; }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override { return std::make_unique<NarrowUv>(*this); }
    };
    for (double component : {7e307,1.4e308})
    {
        NarrowUv uv; uv.component = component;
        const Geometry::SCCurveOnSurface curve(Plane(), uv);
        ASSERT_TRUE(curve.IsValid());
        const double expected = std::hypot(component*1e-308,component*1e-308);
        EXPECT_NEAR(Geometry::Length(curve), expected, 1e-12) << component;
    }
}

TEST(GenericUvCurve3dTest, LengthPreservesSubnormalDerivativeComponentsBeforeScaling)
{
    class TinyDerivativeUv final : public ParabolaUv
    {
    public:
        bool overflowingWidth{false};
        Geometry::SCIntervald ParameterRange() const override
        { return overflowingWidth ? Geometry::SCIntervald{-1e308,1e308} : Geometry::SCIntervald{0,1e308}; }
        Geometry::SCCurveEval2d Evaluate(double t, int order = 1) const override
        {
            const double d = std::numeric_limits<double>::denorm_min();
            const double coordinate = (t/1e308)*(d*1e308);
            return {{coordinate,coordinate},{d,d},std::clamp(order,0,1)};
        }
        Geometry::SCBox2d Bounds() const override
        {
            const double extent = std::numeric_limits<double>::denorm_min()*1e308;
            return overflowingWidth ? Geometry::SCBox2d{{-extent,-extent},{extent,extent}} :
                                      Geometry::SCBox2d{{0,0},{extent,extent}};
        }
        std::unique_ptr<Geometry::ISCCurve2d> Clone() const override
        { return std::make_unique<TinyDerivativeUv>(*this); }
    };
    for (bool overflowingWidth : {false,true})
    {
        TinyDerivativeUv uv; uv.overflowingWidth = overflowingWidth;
        const Geometry::SCCurveOnSurface curve(Plane(), uv);
        ASSERT_TRUE(curve.IsValid());
        const double extent = std::numeric_limits<double>::denorm_min()*1e308;
        const double expected = std::hypot(extent,extent)*(overflowingWidth ? 2 : 1);
        EXPECT_NEAR(Geometry::Length(curve)/expected, 1.0, 1e-12) << overflowingWidth;
    }
}

TEST(GenericUvCurve3dTest, NonlinearLengthRespectsExplicitSupportTolerance)
{
    const auto support = std::make_shared<Geometry::SCPlaneSurface>(
        Geometry::SCPlane::FromPointAndNormal({0,0,0}, {0,0,1e-12}),
        Geometry::SCVector3d{1,0,0}, Geometry::SCVector3d{0,1,0},
        Geometry::SCIntervald{-2,2}, Geometry::SCIntervald{-2,2});
    const Geometry::SCCurveOnSurface curve(support, ParabolaUv{});
    Geometry::SCGeometryTolerance3d tolerance;
    tolerance.distanceEpsilon = 1e-15;
    ASSERT_TRUE(curve.IsValid(tolerance));
    ASSERT_FALSE(curve.IsValid());
    const double exact = std::sqrt(17.0)/2 + std::asinh(4.0)/8;
    EXPECT_NEAR(Geometry::Length(curve,tolerance), exact, 1e-10);
    const auto sample = curve.Evaluate(4.0,1,tolerance);
    ASSERT_EQ(sample.derivativeOrder,1);
    EXPECT_TRUE(sample.point.AlmostEquals({0.5,1,0},1e-12));
    EXPECT_TRUE(sample.firstDerivative.AlmostEquals({0.25,0,0},1e-12));
    auto invalid = tolerance; invalid.distanceEpsilon = -1;
    EXPECT_FALSE(curve.Evaluate(4.0,1,invalid).point.IsValid());
    EXPECT_FALSE(curve.Evaluate(7.0,1,tolerance).point.IsValid());
    EXPECT_DOUBLE_EQ(Geometry::Length(curve), 0.0);
}
