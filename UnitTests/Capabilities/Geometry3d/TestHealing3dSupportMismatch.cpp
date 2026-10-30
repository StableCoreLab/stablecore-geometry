#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using Geometry::BrepHealing3d;
using Geometry::Heal;
using Geometry::HealingIssue3d;
using Geometry::HealingPolicy3d;
using Geometry::ISCSurface;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronHealing3d;
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

// Demonstrates composition under support-plane mismatch: eligible multi-face
// holed shell with missing trims is backfilled and deterministically closed.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellEligibleMultiFaceHoledSupportPlaneMismatch)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible face A (holed)
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 3.0, 0.0}),
                                       // Eligible face B (plain)
                                       SCBrepVertex(SCPoint3d{8.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 1.0, 0.0}),
                                       // Ineligible shell
                                       SCBrepVertex(SCPoint3d{11.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{12.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{12.0, 1.0, 0.10}),
                                       SCBrepVertex(SCPoint3d{11.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Closed shell
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible holed face
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);
    // Eligible plain face
    addEdge(12, 13);
    addEdge(13, 14);
    addEdge(14, 15);
    addEdge(15, 12);
    // Ineligible face
    addEdge(16, 17);
    addEdge(17, 18);
    addEdge(18, 19);
    addEdge(19, 16);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleHoledOuter(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop eligibleHoledHole(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop eligiblePlain(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(15, false)});
    const SCBrepLoop ineligibleLoop(
        {SCBrepCoedge(16, false), SCBrepCoedge(17, false), SCBrepCoedge(18, false), SCBrepCoedge(19, false)});

    const SCPlaneSurface closedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    // Deliberately offset eligible support plane to force support-plane
    // mismatch.
    const SCPlaneSurface eligibleMismatchedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{11.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace eligibleFaceA(std::shared_ptr<ISCSurface>(eligibleMismatchedSurface.Clone().release()),
                                   eligibleHoledOuter,
                                   {eligibleHoledHole});
    const SCBrepFace eligibleFaceB(std::shared_ptr<ISCSurface>(eligibleMismatchedSurface.Clone().release()),
                                   eligiblePlain);
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFaceA, eligibleFaceB}, false),
                              SCBrepShell({ineligibleFace}, false),
                          });
    ASSERT_TRUE(body.IsValid());

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 7);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
    ASSERT_EQ(eligibleHealedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates support-plane-mismatch eligible shell can be closed while an
// ineligible multi-face shared-edge shell remains open in the same body.
TEST(Healing3dCapabilityTest, AggressiveHealingMixedSupportMismatchWithIneligibleMultiFaceShell)
{
    std::vector<SCBrepVertex> vertices{// Closed shell
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible open shell
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       // Ineligible multi-face shell
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Closed shell edges.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible open shell edges.
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Ineligible shell edges: face A plus face B sharing edge (9->10).
    addEdge(8, 9);    // 8
    addEdge(9, 10);   // 9 shared
    addEdge(10, 11);  // 10
    addEdge(11, 8);   // 11
    addEdge(9, 12);   // 12
    addEdge(12, 13);  // 13
    addEdge(13, 10);  // 14

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleOuter(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop ineligibleA(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop ineligibleB(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(9, true)});

    const SCPlaneSurface closedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface eligibleMismatch =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{6.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace eligibleFace(std::shared_ptr<ISCSurface>(eligibleMismatch.Clone().release()), eligibleOuter);
    const SCBrepFace ineligibleFaceA(std::shared_ptr<ISCSurface>(ineligibleSurface.Clone().release()), ineligibleA);
    const SCBrepFace ineligibleFaceB(std::shared_ptr<ISCSurface>(ineligibleSurface.Clone().release()), ineligibleB);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFace}, false),
                              SCBrepShell({ineligibleFaceA, ineligibleFaceB}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 5);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    // closed:2, eligible:1->2, ineligible:2 unchanged.
    ASSERT_EQ(healed.body.FaceCount(), 6);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
}
