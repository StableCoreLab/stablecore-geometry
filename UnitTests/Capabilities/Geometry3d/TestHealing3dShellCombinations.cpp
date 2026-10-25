
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

// Demonstrates aggressive closure also supports a recoverable holed planar
// single-face open shell by mirroring outer and hole loops.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableHoledOpenShell)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 3.0, 0.0})};

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
    addEdge(3, 0);
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeLoop(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace frontFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({frontFace}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 2);
}

// Demonstrates aggressive closure composes with conservative trim backfill on
// a holed open shell where both outer/hole trims are initially missing.
TEST(Healing3dCapabilityTest, AggressiveHealingCompositeHoledOpenShellWithMissingTrims)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 3.0, 0.0})};

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
    addEdge(3, 0);
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeLoop(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    // Intentionally omit outer/hole trims to force conservative backfill first.
    const SCBrepFace frontFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({frontFace}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 2);
    const auto firstFace = healed.body.ShellAt(0).FaceAt(0);
    ASSERT_TRUE(firstFace.OuterTrim().IsValid());
    ASSERT_EQ(firstFace.HoleTrims().size(), 1);
    ASSERT_TRUE(firstFace.HoleTrims()[0].IsValid());
}

// Demonstrates aggressive policy can deterministically close multiple
// recoverable open shells within the same SCBrepBody.
TEST(Healing3dCapabilityTest, AggressiveHealingClosesMultipleOpenShells)
{
    std::vector<SCBrepVertex> vertices{// Shell A
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Shell B
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
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

    // Shell A edges
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Shell B edges
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop loopA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop loopB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), loopA);
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), loopB);
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({faceA}, false), SCBrepShell({faceB}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());
    ASSERT_FALSE(openBody.ShellAt(1).IsClosed());

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    // Each open single-face shell is mirrored into 2 faces.
    ASSERT_EQ(healed.body.FaceCount(), 4);
}

// Demonstrates aggressive policy can close only the recoverable open shell in
// a mixed input while keeping already-closed shells topologically stable.
TEST(Healing3dCapabilityTest, AggressiveHealingPreservesClosedShellAndClosesOpenShell)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Open shell vertices
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
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

    // Closed shell edges.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Open shell edges.
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop openOuter(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace openFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), openOuter);

    const SCBrepBody mixedBody(
        vertices, edges, {SCBrepShell({closedFaceA, closedFaceB}, true), SCBrepShell({openFace}, false)});
    ASSERT_TRUE(mixedBody.IsValid());
    ASSERT_EQ(mixedBody.ShellCount(), 2);
    ASSERT_TRUE(mixedBody.ShellAt(0).IsClosed());
    ASSERT_FALSE(mixedBody.ShellAt(1).IsClosed());
    ASSERT_EQ(mixedBody.FaceCount(), 3);

    const BrepHealing3d healed = Heal(mixedBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    // Closed shell keeps 2 faces, open shell is mirrored from 1 to 2 faces.
    ASSERT_EQ(healed.body.FaceCount(), 4);
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

// Demonstrates deterministic mixed-shell behavior when the eligible shell is
// a multi-face planar open sheet.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellWithEligibleMultiFaceOpenSheet)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible open-sheet vertices (two faces)
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       // Ineligible open shell vertices
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 1.0, 0.14}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.0})};

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
    // Eligible open sheet edges: face A (4-5-6-7), face B (5-8-9-6).
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(5, 8);
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(6, 5);
    // Ineligible open shell edges.
    addEdge(10, 11);
    addEdge(11, 12);
    addEdge(12, 13);
    addEdge(13, 10);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleA(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop eligibleB(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop ineligibleLoop(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(15, false)});

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{7.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace eligibleFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleA);
    const SCBrepFace eligibleFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleB);
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFaceA, eligibleFaceB}, false),
                              SCBrepShell({ineligibleFace}, false),
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
    // Closed shell stays 2 faces; eligible shell mirrors 2->4; ineligible
    // stays 1.
    ASSERT_EQ(healed.body.FaceCount(), 7);
}

// Demonstrates three-shell mixed behavior with eligible holed shell where
// conservative trim backfill and aggressive closure compose deterministically.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellWithEligibleHoledShellAndMissingTrims)
{
    std::vector<SCBrepVertex> vertices{// Closed shell vertices
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible holed open shell vertices
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 3.0, 0.0}),
                                       // Ineligible open shell vertices
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

    // Closed shell.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible holed shell (outer + hole).
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);
    // Ineligible shell.
    addEdge(12, 13);
    addEdge(13, 14);
    addEdge(14, 15);
    addEdge(15, 12);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleOuter(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop eligibleHole(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop ineligibleLoop(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(15, false)});

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{9.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    // Omit trims on eligible holed face to force backfill of outer/hole trims.
    const SCBrepFace eligibleHoledFace(
        std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleOuter, {eligibleHole});
    const SCBrepFace ineligibleFace(std::shared_ptr<ISCSurface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleHoledFace}, false),
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
    // Closed shell stays 2 faces; eligible holed shell mirrors 1->2; ineligible
    // stays 1.
    ASSERT_EQ(healed.body.FaceCount(), 5);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
    ASSERT_EQ(eligibleHealedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates three-shell mixed behavior where eligible shell is multi-face
// and includes a holed face with missing trims before aggressive closure.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellEligibleMultiFaceHoledMissingTrims)
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
                                       SCBrepVertex(SCPoint3d{12.0, 1.0, 0.13}),
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

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{11.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace eligibleFaceA(
        std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleHoledOuter, {eligibleHoledHole});
    const SCBrepFace eligibleFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligiblePlain);
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
    // closed:2, eligible multi-face:2->4, ineligible:1 => total 7.
    ASSERT_EQ(healed.body.FaceCount(), 7);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
    ASSERT_EQ(eligibleHealedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates deterministic composition when eligible multi-face shell has
// both holed and plain faces with missing trims before aggressive closure.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellEligibleMultiFaceBothTrimsMissing)
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
                                       SCBrepVertex(SCPoint3d{12.0, 1.0, 0.11}),
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

    const SCPlaneSurface planarSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSupport =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{11.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), closedOuterReversed);
    // Both eligible faces intentionally omit trims.
    const SCBrepFace eligibleFaceA(
        std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligibleHoledOuter, {eligibleHoledHole});
    const SCBrepFace eligibleFaceB(std::shared_ptr<ISCSurface>(planarSurface.Clone().release()), eligiblePlain);
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
    const auto holedHealedFace = healed.body.ShellAt(1).FaceAt(0);
    const auto plainHealedFace = healed.body.ShellAt(1).FaceAt(1);
    ASSERT_TRUE(holedHealedFace.OuterTrim().IsValid());
    ASSERT_EQ(holedHealedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(holedHealedFace.HoleTrims()[0].IsValid());
    ASSERT_TRUE(plainHealedFace.OuterTrim().IsValid());
}
