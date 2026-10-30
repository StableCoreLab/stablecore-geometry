
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

// Demonstrates competing-shell arbitration now also recognizes duplicated
// topology that occupies the same geometric boundary loop: the independent
// shell still closes, while the two geometrically coincident shells remain
// open even though they do not share topology edge ids.
TEST(Healing3dCapabilityTest, AggressiveHealingSkipsGeometricallyCoincidentBoundaryLoopShells)
{
    std::vector<SCBrepVertex> vertices{// Independent eligible shell.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Geometrically coincident shell A.
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       // Geometrically coincident shell B uses duplicated topology at the same
                                       // coordinates.
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0})};

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

    // Geometrically coincident shell A.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    // Geometrically coincident shell B with duplicated topology.
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
    const SCBrepLoop coincidentA0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop coincidentA1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});
    const SCBrepLoop coincidentB0(
        {SCBrepCoedge(14, false), SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false)});
    const SCBrepLoop coincidentB1(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(15, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace independentFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentA);
    const SCBrepFace independentFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentB);
    const SCBrepFace coincidentFaceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), coincidentA0);
    const SCBrepFace coincidentFaceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), coincidentA1);
    const SCBrepFace coincidentFaceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), coincidentB0);
    const SCBrepFace coincidentFaceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), coincidentB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({independentFaceA, independentFaceB}, false),
                              SCBrepShell({coincidentFaceA0, coincidentFaceA1}, false),
                              SCBrepShell({coincidentFaceB0, coincidentFaceB1}, false),
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

TEST(Healing3dCapabilityTest, AggressiveHealingSkipsPartiallyOverlappedBoundaryLoopShells)
{
    // Keep the partially overlapping horizontal edges shorter than sqrt(default epsilon).
    constexpr double xScale = 1e-6;
    std::vector<SCBrepVertex> vertices{// Partial-overlap shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0 * xScale, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0 * xScale, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0 * xScale, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0 * xScale, 1.0, 0.0}),
                                       // Partial-overlap shell B. Its boundary only overlaps shell A on a
                                       // sub-interval, never on a whole boundary edge.
                                       SCBrepVertex(SCPoint3d{0.5 * xScale, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5 * xScale, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5 * xScale, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5 * xScale, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5 * xScale, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.5 * xScale, 1.0, 0.0})};

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

    // Shell B.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

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
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({faceA0, faceA1}, false),
                              SCBrepShell({faceB0, faceB1}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 2);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_FALSE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.FaceCount(), 4);
    ASSERT_TRUE(healed.body.ShellAt(0).FaceAt(0).OuterTrim().IsValid());
    ASSERT_TRUE(healed.body.ShellAt(1).FaceAt(0).OuterTrim().IsValid());
}
