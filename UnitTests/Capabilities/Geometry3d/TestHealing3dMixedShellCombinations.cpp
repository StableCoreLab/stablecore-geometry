
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

TEST(Healing3dCapabilityTest, ConservativeHealingKeepsSameDirectionPairedEdgesOpen)
{
    const auto surface = std::make_shared<SCPlaneSurface>(Geometry::Test::BuildFinitePlaneSurfaceAtZ5());
    const SCBrepFace face(surface, Geometry::Test::BuildPlanarBrepOuterLoop());
    const SCBrepBody body(
        Geometry::Test::BuildPlanarBrepVerticesAtZ5(), Geometry::Test::BuildPlanarBrepEdgesAtZ5(),
        {SCBrepShell({face, face}, false)});
    ASSERT_TRUE(body.IsValid());

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Conservative);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1U);
    EXPECT_FALSE(healed.body.ShellAtRef(0).IsClosed());

    const SCBrepLoop reversedLoop({SCBrepCoedge(3, true), SCBrepCoedge(2, true),
                                   SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepFace opposingFace(surface, reversedLoop);
    const SCBrepBody opposingBody(
        Geometry::Test::BuildPlanarBrepVerticesAtZ5(), Geometry::Test::BuildPlanarBrepEdgesAtZ5(),
        {SCBrepShell({face, opposingFace}, false)});
    ASSERT_TRUE(opposingBody.IsValid());
    const BrepHealing3d opposingHealed = Heal(
        opposingBody, SCGeometryTolerance3d{}, HealingPolicy3d::Conservative);
    ASSERT_TRUE(opposingHealed.success);
    ASSERT_TRUE(opposingHealed.body.IsValid());
    EXPECT_TRUE(opposingHealed.body.ShellAtRef(0).IsClosed());
}

// Demonstrates aggressive policy can partially repair a mixed open-shell set:
// eligible planar shell is closed, unsupported non-planar shell remains open.
TEST(Healing3dCapabilityTest, AggressiveHealingPartiallyRepairsMixedOpenShells)
{
    std::vector<SCBrepVertex> vertices{// Planar open shell (eligible)
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Non-planar open shell (ineligible for aggressive closure)
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.1}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Eligible planar shell edges.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Ineligible non-planar shell edges.
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop planarLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop nonPlanarLoop(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface nonPlanarSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace planarFace(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), planarLoop);
    const SCBrepFace nonPlanarFace(std::shared_ptr<ISCSurface>(nonPlanarSupport.Clone().release()), nonPlanarLoop);
    const SCBrepBody mixedOpenBody(
        vertices, edges, {SCBrepShell({planarFace}, false), SCBrepShell({nonPlanarFace}, false)});
    ASSERT_TRUE(mixedOpenBody.IsValid());
    ASSERT_FALSE(mixedOpenBody.ShellAt(0).IsClosed());
    ASSERT_FALSE(mixedOpenBody.ShellAt(1).IsClosed());
    ASSERT_EQ(mixedOpenBody.FaceCount(), 2);

    const BrepHealing3d healed = Heal(mixedOpenBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(1).IsClosed());
    // Shell 0 is mirrored (1->2 faces), shell 1 remains single-face open.
    ASSERT_EQ(healed.body.FaceCount(), 3);
}

TEST(Healing3dCapabilityTest, AggressiveHealingRejectsNonPlanarSharedEdgeShellForBoundaryCap)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.5}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.5})};

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
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    const SCBrepLoop firstLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop secondLoop(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});

    const SCPlaneSurface firstSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface secondSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{-0.5, 0.0, 1.0}));

    const SCBrepFace firstFace(std::shared_ptr<ISCSurface>(firstSurface.Clone().release()), firstLoop);
    const SCBrepFace secondFace(std::shared_ptr<ISCSurface>(secondSurface.Clone().release()), secondLoop);
    const SCBrepBody body(vertices, edges, {SCBrepShell({firstFace, secondFace}, false)});
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 1);
    ASSERT_FALSE(body.ShellAt(0).IsClosed());
    ASSERT_EQ(body.ShellAt(0).FaceCount(), 2);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_FALSE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.FaceCount(), 2);
}

// Demonstrates deterministic mixed-shell behavior on three shells: preserve
// closed shell, close eligible open shell, and keep ineligible open shell.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellMixedDeterministicBehavior)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible open shell vertices
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       // Ineligible open shell vertices
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.15}),
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

    // Closed shell.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible open shell.
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Ineligible open shell.
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleOpen(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop ineligibleOpen(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{6.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace eligibleFace(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleOpen);
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligibleOpen);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFace}, false),
                              SCBrepShell({ineligibleFace}, false),
                          });
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.ShellCount(), 3);
    ASSERT_TRUE(body.ShellAt(0).IsClosed());
    ASSERT_FALSE(body.ShellAt(1).IsClosed());
    ASSERT_FALSE(body.ShellAt(2).IsClosed());
    ASSERT_EQ(body.FaceCount(), 4);

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    // Closed shell keeps 2 faces, eligible open shell becomes 2 faces,
    // ineligible remains 1.
    ASSERT_EQ(healed.body.FaceCount(), 5);
}

// Demonstrates mixed three-shell behavior remains deterministic even when the
// eligible open shell also requires conservative trim backfill.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellMixedWithEligibleTrimBackfill)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible open shell vertices
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       // Ineligible open shell vertices
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.12}),
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

    // Closed shell.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible open shell.
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Ineligible open shell.
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleOpen(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop ineligibleOpen(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{6.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    // Omit trims on eligible face to force conservative backfill before
    // aggressive closure.
    const SCBrepFace eligibleFace(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleOpen);
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligibleOpen);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFace}, false),
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
    ASSERT_EQ(healed.body.FaceCount(), 5);
    const auto eligibleFrontFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleFrontFace.OuterTrim().IsValid());
}
