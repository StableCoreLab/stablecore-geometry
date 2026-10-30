#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "Geometry.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Fixtures3d.h"

using Geometry::Intersect;
using Geometry::ProjectPointToCurve;
using Geometry::ProjectPointToCurveOnSurface;
using Geometry::ProjectPointToSurface;
using Geometry::SCCurveOnSurface;
using Geometry::SCLine3d;
using Geometry::SCNurbsCurve3d;
using Geometry::SCNurbsSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCPolyline2d;
using Geometry::SCVector3d;

namespace
{
    TEST(NurbsNumericsTest, PlanarSurfaceProjectionAndIntersectionMeetNumericalBaseline)
    {
        const SCNurbsSurface surface = Geometry::Test::BuildUnitNurbsSurface();
        const SCPoint3d source{0.6, 1.4, 1.0};

        const auto projection = ProjectPointToSurface(source, surface);
        ASSERT_TRUE(projection.success);
        ASSERT_TRUE(projection.IsValid());
        EXPECT_NEAR(projection.u, 0.3, 1e-4);
        EXPECT_NEAR(projection.v, 0.7, 1e-4);
        EXPECT_TRUE(projection.point.AlmostEquals(SCPoint3d{0.6, 1.4, 0.0}, 1e-4));
        EXPECT_NEAR(projection.distanceSquared, 1.0, 1e-7);

        const auto intersection =
            Intersect(SCLine3d::FromOriginAndDirection(source, SCVector3d{0.0, 0.0, -1.0}), surface);
        ASSERT_TRUE(intersection.intersects);
        ASSERT_TRUE(intersection.IsValid());
        EXPECT_NEAR(intersection.lineParameter, 1.0, 1e-8);
        EXPECT_NEAR(intersection.u, projection.u, 1e-4);
        EXPECT_NEAR(intersection.v, projection.v, 1e-4);
        EXPECT_TRUE(intersection.point.AlmostEquals(projection.point, 1e-4));
    }

    TEST(NurbsNumericsTest, NearlyPlanarNurbsDoesNotReportParallelLineAsCoincident)
    {
        constexpr double bend = 2e-10;
        const SCNurbsSurface surface(1, 1, 2, 2,
            {{0.0, 0.0, bend}, {1.0, 0.0, -bend},
             {0.0, 1.0, -bend}, {1.0, 1.0, bend}},
            {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0});
        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.75, bend * 0.25}, SCVector3d{1.0, 0.0, 0.0});
        ASSERT_TRUE(surface.IsValid());
        ASSERT_TRUE(line.IsValid());

        const auto intersection = Intersect(line, surface);
        EXPECT_FALSE(intersection.liesOnSurface);
        if (intersection.intersects)
        {
            EXPECT_TRUE(intersection.point.IsValid());
            EXPECT_TRUE(surface.PointAt(intersection.u, intersection.v).AlmostEquals(
                intersection.point, 1e-12));
        }
    }

    TEST(NurbsNumericsTest, CurveOnSurfaceProjectionUsesActualUvSegmentImage)
    {
        auto surface = std::make_shared<SCNurbsSurface>(
            1, 1, 2, 2,
            std::vector<SCPoint3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                   {0.0, 1.0, 0.0}, {1.0, 1.0, 4.0}},
            std::vector<double>{0.0, 0.0, 1.0, 1.0},
            std::vector<double>{0.0, 0.0, 1.0, 1.0});
        const SCCurveOnSurface curve(
            surface, SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 1.0}}));
        ASSERT_TRUE(curve.IsValid());
        const SCPoint3d expected = surface->PointAt(0.5, 0.5);
        const double actualLength=Geometry::Length(curve);
        const double expectedLength=0.5*std::sqrt(66.0)+0.125*std::asinh(4*std::sqrt(2.0));
        EXPECT_NEAR(actualLength,expectedLength,1e-6);
        EXPECT_GT(actualLength,Geometry::Distance(curve.PointAt(0),curve.PointAt(1)));

        const auto projection = ProjectPointToCurveOnSurface(expected, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(projection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(projection.point.AlmostEquals(
            surface->PointAt(projection.uv.x, projection.uv.y), 1e-12));
        EXPECT_DOUBLE_EQ(projection.distanceSquared, 0.0);

        const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
            SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0});
        const auto intersection = Intersect(plane, curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(intersection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(intersection.point.AlmostEquals(
            surface->PointAt(intersection.uv.x, intersection.uv.y), 1e-12));

        const SCLine3d line = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.5, 0.5, 3.0}, SCVector3d{0.0, 0.0, -1.0});
        const auto lineIntersection = Intersect(line, curve);
        ASSERT_TRUE(lineIntersection.intersects);
        EXPECT_NEAR(lineIntersection.segmentParameter, 0.5, 1e-12);
        EXPECT_TRUE(lineIntersection.point.AlmostEquals(expected, 1e-12));
        EXPECT_TRUE(lineIntersection.point.AlmostEquals(
            surface->PointAt(lineIntersection.uv.x, lineIntersection.uv.y), 1e-12));

        constexpr double nearStartParameter = 1.0 / 128.0;
        const SCPoint3d nearStart = surface->PointAt(nearStartParameter, nearStartParameter);
        const auto nearStartProjection = ProjectPointToCurveOnSurface(nearStart, curve);
        ASSERT_TRUE(nearStartProjection.success);
        EXPECT_NEAR(nearStartProjection.segmentParameter, nearStartParameter, 1e-8);
        EXPECT_TRUE(nearStartProjection.point.AlmostEquals(nearStart, 1e-9));

        const SCLine3d nearStartLine = SCLine3d::FromOriginAndDirection(
            SCPoint3d{nearStart.x, nearStart.y, 1.0}, SCVector3d{0.0, 0.0, -1.0});
        const auto nearStartIntersection = Intersect(nearStartLine, curve);
        ASSERT_TRUE(nearStartIntersection.intersects);
        EXPECT_NEAR(nearStartIntersection.segmentParameter, nearStartParameter, 1e-8);
        EXPECT_TRUE(nearStartIntersection.point.AlmostEquals(nearStart, 1e-9));
    }

    TEST(NurbsNumericsTest, FaceProjectionRequiresCompatibleTrimRegion)
    {
        const auto support = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlane::FromPointAndNormal({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            SCVector3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0},
            Geometry::SCIntervald{-1.0, 1.0}, Geometry::SCIntervald{-1.0, 1.0});
        const auto displaced = std::make_shared<Geometry::SCPlaneSurface>(
            Geometry::SCPlane::FromPointAndNormal({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}),
            SCVector3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0},
            Geometry::SCIntervald{-1.0, 1.0}, Geometry::SCIntervald{-1.0, 1.0});
        const Geometry::SCBrepLoop loop({Geometry::SCBrepCoedge{0}});
        const SCPolyline2d ring(
            std::vector<SCPoint2d>{{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}},
            Geometry::SCPolylineClosure::Closed);
        const SCCurveOnSurface compatibleTrim(std::make_shared<Geometry::SCPlaneSurface>(*support), ring);
        const SCCurveOnSurface incompatibleTrim(displaced, ring);
        const Geometry::SCBrepFace missingTrim(support, loop);
        const Geometry::SCBrepFace compatibleFace(support, loop, {}, compatibleTrim);
        const Geometry::SCBrepFace incompatibleFace(support, loop, {}, incompatibleTrim);
        ASSERT_TRUE(missingTrim.IsValid());
        ASSERT_TRUE(compatibleFace.IsValid());
        ASSERT_TRUE(incompatibleFace.IsValid());

        const SCPoint3d query = support->PointAt(0.0, 0.0);
        EXPECT_FALSE(Geometry::ProjectPointToBrepFace(query, missingTrim).success);
        const SCLine3d faceLine = SCLine3d::FromOriginAndDirection(
            SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, -1.0});
        EXPECT_FALSE(Intersect(faceLine, missingTrim).intersects);
        const auto validProjection = Geometry::ProjectPointToBrepFace(query, compatibleFace);
        ASSERT_TRUE(validProjection.success);
        EXPECT_TRUE(validProjection.point.AlmostEquals(query, 1e-12));
        EXPECT_FALSE(Geometry::ProjectPointToBrepFace(query, incompatibleFace).success);
        const auto faceIntersection = Intersect(faceLine, compatibleFace);
        ASSERT_TRUE(faceIntersection.intersects);
        EXPECT_TRUE(faceIntersection.point.AlmostEquals(query, 1e-12));
        EXPECT_FALSE(Intersect(faceLine, incompatibleFace).intersects);

        const auto narrowSupport = std::make_shared<Geometry::SCPlaneSurface>(
            support->SupportPlane(), support->UAxis(), support->VAxis(),
            Geometry::SCIntervald{-0.1, 0.1}, Geometry::SCIntervald{-0.1, 0.1});
        const Geometry::SCBrepFace extendedTrimFace(
            narrowSupport, loop, {},
            SCCurveOnSurface(std::make_shared<Geometry::SCPlaneSurface>(*narrowSupport), ring));
        const auto extendedProjection = Geometry::ProjectPointToBrepFace({0.4, 0.0, 1.0}, extendedTrimFace);
        ASSERT_TRUE(extendedProjection.success);
        EXPECT_NEAR(extendedProjection.u, 0.4, 1e-12);
        EXPECT_NEAR(extendedProjection.v, 0.0, 1e-12);
        EXPECT_TRUE(extendedProjection.point.AlmostEquals(SCPoint3d{0.4, 0.0, 0.0}, 1e-12));

        const std::vector<SCPoint3d> corners{{-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0},
                                             {0.5, 0.5, 0.0}, {-0.5, 0.5, 0.0}};
        std::vector<Geometry::SCBrepVertex> vertices;
        std::vector<Geometry::SCBrepEdge> edges;
        std::vector<Geometry::SCBrepCoedge> coedges;
        for (std::size_t index = 0; index < corners.size(); ++index)
        {
            const std::size_t next = (index + 1) % corners.size();
            vertices.emplace_back(corners[index]);
            edges.emplace_back(std::make_shared<Geometry::SCLineCurve3d>(
                                   SCLine3d::FromOriginAndDirection(
                                       corners[index], corners[next] - corners[index]),
                                   Geometry::SCIntervald{0.0, 1.0}), index, next);
            coedges.emplace_back(index);
        }
        const Geometry::SCBrepFace topologyFace(support, Geometry::SCBrepLoop(std::move(coedges)));
        const Geometry::SCBrepBody body(std::move(vertices), std::move(edges),
                                        {Geometry::SCBrepShell({topologyFace})});
        ASSERT_TRUE(body.IsValid());
        const auto bodyProjection = Geometry::ProjectPointToBrepBody(query, body);
        ASSERT_TRUE(bodyProjection.success);
        EXPECT_TRUE(bodyProjection.projection.point.AlmostEquals(query, 1e-12));
        const auto bodyIntersection = Intersect(faceLine, body);
        ASSERT_TRUE(bodyIntersection.intersects);
        ASSERT_EQ(bodyIntersection.hits.size(), 1u);
        EXPECT_TRUE(bodyIntersection.hits.front().point.AlmostEquals(query, 1e-12));

        std::vector<Geometry::SCBrepEdge> reversedEdges = body.Edges();
        reversedEdges[0] = Geometry::SCBrepEdge(
            std::make_shared<Geometry::SCLineCurve3d>(
                SCLine3d::FromOriginAndDirection(corners[1], corners[0] - corners[1]),
                Geometry::SCIntervald{0.0, 1.0}), 0, 1);
        const Geometry::SCBrepBody mismatchedBody(body.Vertices(), std::move(reversedEdges), body.Shells());
        ASSERT_TRUE(mismatchedBody.IsValid());
        EXPECT_FALSE(Geometry::ProjectPointToBrepBody(query, mismatchedBody).success);
        EXPECT_FALSE(Intersect(faceLine, mismatchedBody).intersects);
    }

    TEST(NurbsNumericsTest, SmallScaleCurveProjectionAndIntersectionRefineBetweenSeeds)
    {
        constexpr double length = 1e-5;
        const SCNurbsCurve3d curve(1, {{0.0, 0.0, 0.0}, {length, 0.0, 0.0}},
                                   {0.0, 0.0, 1.0, 1.0});
        const SCPoint3d source{length * 0.25, length, 0.0};
        const SCPoint3d expected{length * 0.25, 0.0, 0.0};
        ASSERT_TRUE(curve.IsValid());

        const auto projection = ProjectPointToCurve(source, curve);
        ASSERT_TRUE(projection.success);
        EXPECT_NEAR(projection.parameter, 0.25, 1e-10);
        EXPECT_TRUE(projection.point.AlmostEquals(expected, 1e-12));

        const auto intersection = Intersect(
            SCLine3d::FromOriginAndDirection(source, SCVector3d{0.0, -1.0, 0.0}), curve);
        ASSERT_TRUE(intersection.intersects);
        EXPECT_NEAR(intersection.curveParameter, 0.25, 1e-10);
        EXPECT_TRUE(intersection.point.AlmostEquals(expected, 1e-12));
    }

}  // namespace
