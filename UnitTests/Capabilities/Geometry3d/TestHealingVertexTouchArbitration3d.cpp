
#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using Geometry::BrepHealing3d;
using Geometry::Heal;
using Geometry::HealingIssue3d;
using Geometry::HealingPolicy3d;
using Geometry::ISCSurface;
using Geometry::SCBrepBody;
using Geometry::SCBrepCoedge;
using Geometry::SCBrepEdge;
using Geometry::SCBrepFace;
using Geometry::SCBrepLoop;
using Geometry::SCBrepShell;
using Geometry::SCBrepVertex;
using Geometry::SCGeometryTolerance3d;
using Geometry::SCIntervald;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCPlane;
using Geometry::SCPlaneSurface;
using Geometry::SCPoint3d;
using Geometry::SCVector3d;

// Demonstrates multi-shell arbitration is no longer blocked by a mere
// vertex-touch between eligible shared-edge shells: without a shared boundary
// edge, both shells can still be boundary-capped independently.
TEST(Healing3dCapabilityTest, AggressiveHealingClosesVertexTouchingEligibleSharedEdgeShells)
{
    std::vector<SCBrepVertex> vertices{// Eligible shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Eligible shell B shares only vertex 5 with shell A.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Shell A.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Shell B shares only boundary vertex 5 with shell A.
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 8);
    addEdge(8, 5);
    addEdge(6, 9);
    addEdge(9, 10);
    addEdge(10, 7);

    const SCBrepLoop shellA0(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop shellA1(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCBrepLoop shellB0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop shellB1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace shellAFace0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace shellAFace1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace shellBFace0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace shellBFace1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({shellAFace0, shellAFace1}, false),
                              SCBrepShell({shellBFace0, shellBFace1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 2);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 3);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 6);
    ASSERT_TRUE(healed.body.ShellAt(0).FaceAt(2).OuterTrim().IsValid());
    ASSERT_TRUE(healed.body.ShellAt(1).FaceAt(2).OuterTrim().IsValid());
}

// Demonstrates the aggressive arbitration rules compose deterministically:
// shells that share a boundary edge still remain open, while another eligible
// shell that only vertex-touches that competing set can still close.
TEST(Healing3dCapabilityTest, AggressiveHealingKeepsCompetingPairOpenButClosesVertexTouchShell)
{
    std::vector<SCBrepVertex> vertices{// Competing shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Competing shell B shares boundary edge (4,5) with shell A.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       // Vertex-touch shell C shares only vertex 9 with shell B.
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Shell A.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Shell B shares boundary edge (4,5) with shell A.
    addEdge(4, 6);
    addEdge(6, 7);  // shared interior edge
    addEdge(7, 5);
    addEdge(5, 4);
    addEdge(6, 8);
    addEdge(8, 9);
    addEdge(9, 7);

    // Shell C shares only vertex 9 with shell B.
    addEdge(9, 10);
    addEdge(10, 11);  // shared interior edge
    addEdge(11, 12);
    addEdge(12, 9);
    addEdge(10, 13);
    addEdge(13, 14);
    addEdge(14, 11);

    const SCBrepLoop shellA0(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop shellA1(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCBrepLoop shellB0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop shellB1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});
    const SCBrepLoop shellC0(
        {SCBrepCoedge(14, false), SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false)});
    const SCBrepLoop shellC1(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(15, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);
    const SCBrepFace faceC0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC0);
    const SCBrepFace faceC1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({faceA0, faceA1}, false),
                              SCBrepShell({faceB0, faceB1}, false),
                              SCBrepShell({faceC0, faceC1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 3);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_FALSE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 7);
    ASSERT_TRUE(healed.body.ShellAt(2).FaceAt(2).OuterTrim().IsValid());
}
