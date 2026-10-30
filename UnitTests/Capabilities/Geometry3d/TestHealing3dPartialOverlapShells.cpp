
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

TEST(Healing3dCapabilityTest, AggressiveHealingClosesIndependentShellWhileSkippingPartialOverlapPair)
{
    std::vector<SCBrepVertex> vertices{// Independent eligible shell.
                                       SCBrepVertex(SCPoint3d{-4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 1.0, 0.0}),
                                       // Partial-overlap shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Partial-overlap shell B.
                                       SCBrepVertex(SCPoint3d{0.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Independent shell.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Partial-overlap shell A.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    // Partial-overlap shell B.
    addEdge(12, 13);
    addEdge(13, 14);  // shared interior edge
    addEdge(14, 15);
    addEdge(15, 12);
    addEdge(13, 16);
    addEdge(16, 17);
    addEdge(17, 14);

    const SCBrepLoop independentA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop independentB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCBrepLoop shellA0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop shellA1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});
    const SCBrepLoop shellB0(
        {SCBrepCoedge(14, false), SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false)});
    const SCBrepLoop shellB1(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(15, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace independentFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentA);
    const SCBrepFace independentFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentB);
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({independentFaceA, independentFaceB}, false),
                              SCBrepShell({faceA0, faceA1}, false),
                              SCBrepShell({faceB0, faceB1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 3);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 3);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.FaceCount(), 7);
    ASSERT_TRUE(healed.body.ShellAt(0).FaceAt(2).OuterTrim().IsValid());
}

TEST(Healing3dCapabilityTest, AggressiveHealingKeepsClosedShellWhileSkippingPartialOverlapPair)
{
    std::vector<SCBrepVertex> vertices{// Closed shell.
                                       SCBrepVertex(SCPoint3d{-4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-4.0, 1.0, 0.0}),
                                       // Partial-overlap shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Partial-overlap shell B.
                                       SCBrepVertex(SCPoint3d{0.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Closed shell.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);

    // Partial-overlap shell A.
    addEdge(4, 5);
    addEdge(5, 6);  // shared interior edge
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(5, 8);
    addEdge(8, 9);
    addEdge(9, 6);

    // Partial-overlap shell B.
    addEdge(10, 11);
    addEdge(11, 12);  // shared interior edge
    addEdge(12, 13);
    addEdge(13, 10);
    addEdge(11, 14);
    addEdge(14, 15);
    addEdge(15, 12);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop shellA0(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop shellA1(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(5, true)});
    const SCBrepLoop shellB0(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false)});
    const SCBrepLoop shellB1(
        {SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false), SCBrepCoedge(12, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({faceA0, faceA1}, false),
                              SCBrepShell({faceB0, faceB1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 3);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.FaceCount(), 6);
}

TEST(Healing3dCapabilityTest, AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingPartialOverlapPair)
{
    std::vector<SCBrepVertex> vertices{// Independent shell.
                                       SCBrepVertex(SCPoint3d{-4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 1.0, 0.0}),
                                       // Partial-overlap shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Partial-overlap shell B.
                                       SCBrepVertex(SCPoint3d{0.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 1.0, 0.0}),
                                       // Vertex-touch shell shares only vertex 17 with partial-overlap shell
                                       // B.
                                       SCBrepVertex(SCPoint3d{3.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.5, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.5, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Independent shell.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Partial-overlap shell A.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    // Partial-overlap shell B.
    addEdge(12, 13);
    addEdge(13, 14);  // shared interior edge
    addEdge(14, 15);
    addEdge(15, 12);
    addEdge(13, 16);
    addEdge(16, 17);
    addEdge(17, 14);

    // Vertex-touch shell shares only vertex 17 with shell B.
    addEdge(17, 18);
    addEdge(18, 19);  // shared interior edge
    addEdge(19, 20);
    addEdge(20, 17);
    addEdge(18, 21);
    addEdge(21, 22);
    addEdge(22, 19);

    const SCBrepLoop independentA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop independentB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCBrepLoop shellA0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop shellA1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});
    const SCBrepLoop shellB0(
        {SCBrepCoedge(14, false), SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false)});
    const SCBrepLoop shellB1(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(15, true)});
    const SCBrepLoop shellC0(
        {SCBrepCoedge(21, false), SCBrepCoedge(22, false), SCBrepCoedge(23, false), SCBrepCoedge(24, false)});
    const SCBrepLoop shellC1(
        {SCBrepCoedge(25, false), SCBrepCoedge(26, false), SCBrepCoedge(27, false), SCBrepCoedge(22, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace independentFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentA);
    const SCBrepFace independentFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentB);
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);
    const SCBrepFace faceC0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC0);
    const SCBrepFace faceC1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({independentFaceA, independentFaceB}, false),
                              SCBrepShell({faceA0, faceA1}, false),
                              SCBrepShell({faceB0, faceB1}, false),
                              SCBrepShell({faceC0, faceC1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 4);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 4);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(3).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 3);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(3).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 10);
    ASSERT_TRUE(healed.body.ShellAt(0).FaceAt(2).OuterTrim().IsValid());
    ASSERT_TRUE(healed.body.ShellAt(3).FaceAt(2).OuterTrim().IsValid());
}
