
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

// Demonstrates conservative trim backfill for a SCBrepFace whose support plane
// has a non-horizontal (vertically-oriented) normal 闂?the y=0 plane with
// normal (0,1,0). This narrows the
// NonPlanarTrimmedFaceTopologyRepairRemainsOpen gap to the subset: single
// axis-non-horizontal planar face with missing trim.
TEST(Healing3dCapabilityTest, NonHorizontalPlaneBrepFaceWithoutTrimIsHealedWithBackfilledTrim)
{
    // Vertices on the y=0 plane (vertical face, normal = +y).
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 1.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 1.0})};

    std::vector<SCBrepEdge> edges;
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       0,
                       1);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                           SCIntervald{0.0, 1.0})),
                       1,
                       2);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 0.0, 1.0}, SCVector3d{-1.0, 0.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       2,
                       3);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, -1.0}),
                           SCIntervald{0.0, 1.0})),
                       3,
                       0);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});

    // Support plane: y=0, normal pointing in +y direction.
    const SCPlaneSurface verticalSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}));
    const SCBrepFace face(std::shared_ptr<ISCSurface>(verticalSurface.Clone().release()), outerLoop);
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});

    ASSERT_TRUE(body.IsValid());
    ASSERT_FALSE(face.OuterTrim().IsValid());

    const BrepHealing3d healed = Heal(body);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.FaceCount(), 1);
    const auto healedFace = healed.body.ShellAt(0).FaceAt(0);
    ASSERT_TRUE(healedFace.OuterTrim().IsValid());
    ASSERT_EQ(healedFace.OuterTrim().PointCount(), 4);
}

// Demonstrates conservative trim backfill also holds for a face whose support
// plane has an x-axis normal (x=0 plane, normal +x). Extends the non-planar
// trim-backfill coverage to a third distinct orientation (z=0/+z horizontal
// already covered; y=0/+y vertical already covered; now x=0/+x vertical).
TEST(Healing3dCapabilityTest, XNormalPlaneBrepFaceWithoutTrimIsHealedWithBackfilledTrim)
{
    // Vertices on the x=0 plane (vertical face, normal = +x).
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 1.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 1.0})};

    std::vector<SCBrepEdge> edges;
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       0,
                       1);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                           SCIntervald{0.0, 1.0})),
                       1,
                       2);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 1.0, 1.0}, SCVector3d{0.0, -1.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       2,
                       3);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, -1.0}),
                           SCIntervald{0.0, 1.0})),
                       3,
                       0);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});

    // Support plane: x=0, normal pointing in +x direction.
    const SCPlaneSurface xNormalSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}));
    const SCBrepFace face(std::shared_ptr<ISCSurface>(xNormalSurface.Clone().release()), outerLoop);
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});

    ASSERT_TRUE(body.IsValid());
    ASSERT_FALSE(face.OuterTrim().IsValid());

    const BrepHealing3d healed = Heal(body);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.FaceCount(), 1);
    const auto healedFaceX = healed.body.ShellAt(0).FaceAt(0);
    ASSERT_TRUE(healedFaceX.OuterTrim().IsValid());
    ASSERT_EQ(healedFaceX.OuterTrim().PointCount(), 4);
}

// Demonstrates conservative trim backfill also works on an oblique planar
// face (x+y+z=0, normal (1,1,1)) with line-edge topology and missing trim.
TEST(Healing3dCapabilityTest, ObliquePlaneBrepFaceWithoutTrimIsHealedWithBackfilledTrim)
{
    // All vertices lie on x+y+z=0.
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, -1.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, -2.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, -1.0})};

    std::vector<SCBrepEdge> edges;
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, -1.0}),
                           SCIntervald{0.0, 1.0})),
                       0,
                       1);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 0.0, -1.0}, SCVector3d{0.0, 1.0, -1.0}),
                           SCIntervald{0.0, 1.0})),
                       1,
                       2);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, -2.0}, SCVector3d{-1.0, 0.0, 1.0}),
                           SCIntervald{0.0, 1.0})),
                       2,
                       3);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 1.0, -1.0}, SCVector3d{0.0, -1.0, 1.0}),
                           SCIntervald{0.0, 1.0})),
                       3,
                       0);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});

    const SCPlaneSurface obliqueSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 1.0, 1.0}));
    const SCBrepFace face(std::shared_ptr<ISCSurface>(obliqueSurface.Clone().release()), outerLoop);
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});

    ASSERT_TRUE(body.IsValid());
    ASSERT_FALSE(face.OuterTrim().IsValid());

    const BrepHealing3d healed = Heal(body);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.FaceCount(), 1);
    const auto healedFace = healed.body.ShellAt(0).FaceAt(0);
    ASSERT_TRUE(healedFace.OuterTrim().IsValid());
    ASSERT_EQ(healedFace.OuterTrim().PointCount(), 4);
}

// Demonstrates aggressive healing handles a four-shell body 闂?one closed plus
// two independent eligible single-face open shells plus one ineligible 闂?and
// closes exactly the two eligible shells, leaving the closed and ineligible
// shells unchanged. Narrows the AggressiveShellRepairPolicyRemainsOpen gap to
// the four-shell two-eligible-plus-one-ineligible subset.
TEST(Healing3dCapabilityTest, AggressiveFourShellTwoEligibleOneIneligibleDeterministicBehavior)
{
    std::vector<SCBrepVertex> vertices{// Shell 0: closed (2-face, z=0)
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Shell 1: eligible single-face open (z=0)
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       // Shell 2: eligible single-face open (z=0)
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       // Shell 3: ineligible 闂?vertex 14 has z-offset (not on z=0 support
                                       // plane)
                                       SCBrepVertex(SCPoint3d{9.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{10.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{10.0, 1.0, 0.15}),
                                       SCBrepVertex(SCPoint3d{9.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);  // shell 0 (edges 0-3)
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);  // shell 1 (edges 4-7)
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);  // shell 2 (edges 8-11)
    addEdge(12, 13);
    addEdge(13, 14);
    addEdge(14, 15);
    addEdge(15, 12);  // shell 3 (edges 12-15)

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligible1(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop eligible2(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop ineligible(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(15, false)});

    const SCPlaneSurface flatSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{9.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(flatSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(flatSurface.Clone().release()), closedReversed);
    const SCBrepFace eligibleFace1(std::shared_ptr<ISCSurface>(flatSurface.Clone().release()), eligible1);
    const SCBrepFace eligibleFace2(std::shared_ptr<ISCSurface>(flatSurface.Clone().release()), eligible2);
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligible);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFace1}, false),
                              SCBrepShell({eligibleFace2}, false),
                              SCBrepShell({ineligibleFace}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 4);
    ASSERT_TRUE(body.ShellAt(0).IsClosed());
    ASSERT_FALSE(body.ShellAt(1).IsClosed());
    ASSERT_FALSE(body.ShellAt(2).IsClosed());
    ASSERT_FALSE(body.ShellAt(3).IsClosed());
    ASSERT_EQ(body.FaceCount(), 5);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 4);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(2).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(3).IsClosed());
    // closed:2, eligible1:1->2, eligible2:1->2, ineligible:1 unchanged.
    ASSERT_EQ(healed.body.FaceCount(), 7);
}
