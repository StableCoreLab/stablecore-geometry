#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(BrepTopologyValidityTest, ReferenceChecksUseOwningContainerSizes)
{
    const SCLine3d line = SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto curve = std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(line, {0.0, 1.0}));
    const SCBrepEdge edge(curve, 0, 2);
    EXPECT_TRUE(edge.IsValid());
    EXPECT_FALSE(edge.IsValid(2U));
    EXPECT_TRUE(edge.IsValid(3U));

    const SCBrepEdge sentinelEdge(curve, static_cast<std::size_t>(-1), 0);
    EXPECT_FALSE(sentinelEdge.IsValid());

    const SCBrepCoedge coedge(2);
    EXPECT_TRUE(coedge.IsValid());
    EXPECT_FALSE(coedge.IsValid(2U));
    EXPECT_TRUE(coedge.IsValid(3U));

    const SCBrepLoop loop({coedge});
    EXPECT_TRUE(loop.IsValid());
    EXPECT_FALSE(loop.IsValid(2U));
    EXPECT_TRUE(loop.IsValid(3U));
}

TEST(BrepTopologyValidityTest, ValidationCountsTopologicalEdgeIdentities)
{
    auto edges = Geometry::Test::BuildPlanarBrepEdgesAtZ5();
    edges.push_back(edges.front());
    const SCBrepFace face(
        std::make_shared<SCPlaneSurface>(Geometry::Test::BuildFinitePlaneSurfaceAtZ5()),
        Geometry::Test::BuildPlanarBrepOuterLoop());
    const SCBrepBody body(
        Geometry::Test::BuildPlanarBrepVerticesAtZ5(), std::move(edges),
        {SCBrepShell({face}, false)});
    ASSERT_TRUE(body.IsValid());

    const auto result = Validate(body);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.issue, SCBrepValidationIssue3d::InvalidFaceAdjacency);
    EXPECT_EQ(result.elementIndex, 4U);

    const SCBrepFace secondFace(
        std::make_shared<SCPlaneSurface>(Geometry::Test::BuildFinitePlaneSurfaceAtZ5()),
        SCBrepLoop({SCBrepCoedge(4, false), SCBrepCoedge(1, false),
                    SCBrepCoedge(2, false), SCBrepCoedge(3, false)}));
    auto pairedEdges = Geometry::Test::BuildPlanarBrepEdgesAtZ5();
    pairedEdges.push_back(pairedEdges.front());
    const SCBrepBody pairedBody(
        Geometry::Test::BuildPlanarBrepVerticesAtZ5(),
        std::move(pairedEdges),
        {SCBrepShell({face, secondFace}, false)});
    ASSERT_TRUE(pairedBody.IsValid());
    EXPECT_TRUE(Validate(pairedBody).valid);
}
