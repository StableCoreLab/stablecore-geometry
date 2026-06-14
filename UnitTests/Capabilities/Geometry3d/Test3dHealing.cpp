
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

// Demonstrates that the conservative healing pass preserves an already-valid
// PolyhedronBody without altering face count or validity.
// The "aggressive shell repair policy" (topology-changing repair) and
// "multi-step mesh/body joint healing" remain open gaps.
TEST(Healing3dCapabilityTest, UnitCubePolyhedronBodyHealingPreservesAllSixFaces)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(cubeBody.FaceCount(), 6);

    const PolyhedronHealing3d healed = Heal(cubeBody);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.FaceCount(), 6);
}

// Demonstrates trim backfill on a planar line-edge SCBrepFace where topology is
// present but trim curves are intentionally omitted.
TEST(Healing3dCapabilityTest, PlanarBrepFaceWithoutTrimIsHealedWithBackfilledTrim)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       0,
                       1);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       1,
                       2);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       2,
                       3);
    edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                           SCLine3d::FromOriginAndDirection(SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
                           SCIntervald{0.0, 1.0})),
                       3,
                       0);

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace face(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop);
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

// Demonstrates conservative healing can backfill both outer and hole trims for
// a planar line-edge SCBrepFace when trims are omitted.
TEST(Healing3dCapabilityTest, PlanarHoledBrepFaceWithoutAnyTrimIsHealed)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 2.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 2.0, 0.0})};

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
    const SCBrepFace face(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});

    ASSERT_TRUE(body.IsValid());
    ASSERT_FALSE(face.OuterTrim().IsValid());
    ASSERT_TRUE(face.HoleTrims().empty());

    const BrepHealing3d healed = Heal(body);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.FaceCount(), 1);
    const auto healedFace = healed.body.ShellAt(0).FaceAt(0);
    ASSERT_TRUE(healedFace.OuterTrim().IsValid());
    ASSERT_EQ(healedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(healedFace.HoleTrims()[0].IsValid());
    ASSERT_EQ(healedFace.HoleTrims()[0].PointCount(), 4);
}

// Demonstrates a deterministic minimal aggressive policy: a recoverable open
// planar single-face shell can be closed by adding an opposite-oriented face.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableSingleFaceShell)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0})};

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

    const SCBrepLoop outerLoop(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace face(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop);
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({face}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 2);
}

// Demonstrates aggressive policy also closes a recoverable planar multi-face
// open-sheet shell where each edge is single-use before repair.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableMultiFaceOpenSheet)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Face A edges: 0-1-2-3-0
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face B edges: 1-4-5-2-1 (does not share edges with face A)
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);
    addEdge(2, 1);

    const SCBrepLoop outerA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA);
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB);
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());
    ASSERT_EQ(openBody.FaceCount(), 2);

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 4);
}

// Demonstrates aggressive policy also supports a planar multi-face open sheet
// whose adjacent faces share an interior edge before closure. In this case the
// aggressive policy closes the standalone shell by adding one boundary-cap face
// instead of mirroring both front faces.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseSharedEdgeOpenSheet)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0})};

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

    const SCBrepLoop outerA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA);
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB);
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 3);
    const auto capFace = healed.body.ShellAt(0).FaceAt(2);
    ASSERT_TRUE(capFace.OuterTrim().IsValid());
    ASSERT_TRUE(capFace.HoleTrims().empty());
}

// Demonstrates the more general standalone aggressive boundary-cap subset:
// a shared-edge multi-face shell with support-plane mismatch and missing trims
// is first backfilled by conservative healing, then closed by adding a single
// holed cap face that consumes the shell boundary loops.
TEST(Healing3dCapabilityTest, AggressiveHealingCanBoundaryCapSharedEdgeHoledShellWithMissingTrims)
{
    std::vector<SCBrepVertex> vertices{SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.5, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.5, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0})};

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
    addEdge(1, 8);
    addEdge(8, 9);
    addEdge(9, 2);
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeA(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop outerB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});

    const SCPlaneSurface mismatchedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace holedFace(std::shared_ptr<ISCSurface>(mismatchedSurface.Clone().release()), outerA, {holeA});
    const SCBrepFace plainFace(std::shared_ptr<ISCSurface>(mismatchedSurface.Clone().release()), outerB);
    const SCBrepBody openBody(vertices, edges, {SCBrepShell({holedFace, plainFace}, false)});
    ASSERT_TRUE(openBody.IsValid());
    ASSERT_FALSE(openBody.ShellAt(0).IsClosed());
    ASSERT_EQ(openBody.FaceCount(), 2);

    const BrepHealing3d healed = Heal(openBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 1);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_EQ(healed.body.FaceCount(), 3);

    const auto healedHoledFace = healed.body.ShellAt(0).FaceAt(0);
    const auto healedPlainFace = healed.body.ShellAt(0).FaceAt(1);
    const auto capFace = healed.body.ShellAt(0).FaceAt(2);
    ASSERT_TRUE(healedHoledFace.OuterTrim().IsValid());
    ASSERT_EQ(healedHoledFace.HoleTrims().size(), 1);
    ASSERT_TRUE(healedHoledFace.HoleTrims()[0].IsValid());
    ASSERT_TRUE(healedPlainFace.OuterTrim().IsValid());
    ASSERT_TRUE(capFace.OuterTrim().IsValid());
    ASSERT_EQ(capFace.HoleTrims().size(), 1);
    ASSERT_TRUE(capFace.HoleTrims()[0].IsValid());
}

// Demonstrates the shared-edge boundary-cap fallback also works when the
// eligible shell is only one shell inside a mixed body: a closed shell stays
// unchanged while the open shared-edge shell is closed in place.
TEST(Healing3dCapabilityTest, AggressiveHealingCanBoundaryCapSharedEdgeShellInsideMixedBody)
{
    std::vector<SCBrepVertex> vertices{// Closed shell.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible shared-edge shell.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0})};

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
    // Eligible shared-edge shell.
    addEdge(4, 5);
    addEdge(5, 6);  // shared interior edge
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(5, 8);
    addEdge(8, 9);
    addEdge(9, 6);

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop outerA(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop outerB(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(5, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA);
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB);

    const SCBrepBody mixedBody(vertices,
                               edges,
                               {
                                   SCBrepShell({closedFaceA, closedFaceB}, true),
                                   SCBrepShell({faceA, faceB}, false),
                               });
    ASSERT_TRUE(mixedBody.IsValid());
    ASSERT_EQ(mixedBody.ShellCount(), 2);
    ASSERT_TRUE(mixedBody.ShellAt(0).IsClosed());
    ASSERT_FALSE(mixedBody.ShellAt(1).IsClosed());

    const BrepHealing3d healed = Heal(mixedBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 2);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 5);
    const auto capFace = healed.body.ShellAt(1).FaceAt(2);
    ASSERT_TRUE(capFace.OuterTrim().IsValid());
    ASSERT_TRUE(capFace.HoleTrims().empty());
}

// Demonstrates the mixed-body shared-edge boundary-cap subset scales to more
// than one eligible shell: each eligible shared-edge shell is capped
// independently while an already-closed shell remains unchanged.
TEST(Healing3dCapabilityTest, AggressiveHealingCanBoundaryCapTwoSharedEdgeShellsInsideMixedBody)
{
    std::vector<SCBrepVertex> vertices{// Closed shell.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible shared-edge shell A.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       // Eligible shared-edge shell B.
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 0.0, 0.0}),
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
    // Eligible shell A.
    addEdge(4, 5);
    addEdge(5, 6);  // shared interior edge
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(5, 8);
    addEdge(8, 9);
    addEdge(9, 6);
    // Eligible shell B.
    addEdge(10, 11);
    addEdge(11, 12);  // shared interior edge
    addEdge(12, 13);
    addEdge(13, 10);
    addEdge(11, 14);
    addEdge(14, 15);
    addEdge(15, 12);

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});

    const SCBrepLoop outerA0(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop outerA1(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(5, true)});

    const SCBrepLoop outerB0(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false)});
    const SCBrepLoop outerB1(
        {SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false), SCBrepCoedge(12, true)});

    const SCBrepBody mixedBody(
        vertices,
        edges,
        {
            SCBrepShell(
                {
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuter),
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuterReversed),
                },
                true),
            SCBrepShell(
                {
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA0),
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA1),
                },
                false),
            SCBrepShell(
                {
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB0),
                    SCBrepFace(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB1),
                },
                false),
        });
    ASSERT_TRUE(mixedBody.IsValid());
    ASSERT_EQ(mixedBody.ShellCount(), 3);
    ASSERT_TRUE(mixedBody.ShellAt(0).IsClosed());
    ASSERT_FALSE(mixedBody.ShellAt(1).IsClosed());
    ASSERT_FALSE(mixedBody.ShellAt(2).IsClosed());

    const BrepHealing3d healed = Heal(mixedBody, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_EQ(healed.issue, HealingIssue3d::None);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 3);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 8);
    ASSERT_TRUE(healed.body.ShellAt(1).FaceAt(2).OuterTrim().IsValid());
    ASSERT_TRUE(healed.body.ShellAt(2).FaceAt(2).OuterTrim().IsValid());
}

// Demonstrates conservative multi-shell arbitration on the aggressive
// boundary-cap path: an independent eligible shell still closes, but eligible
// shells that share a boundary edge with another open shell are left open.
TEST(Healing3dCapabilityTest, AggressiveHealingSkipsCompetingSharedBoundaryEdgeEligibleShells)
{
    std::vector<SCBrepVertex> vertices{// Independent eligible shell.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Competing eligible shell A.
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{5.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       // Competing eligible shell B shares boundary edge (10,11) with shell A.
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 1.0, 0.0}),
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

    // Independent shell.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Competing shell A.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    // Competing shell B shares boundary edge (10,11) with shell A.
    addEdge(10, 12);
    addEdge(12, 13);  // shared interior edge
    addEdge(13, 11);
    addEdge(11, 10);
    addEdge(12, 14);
    addEdge(14, 15);
    addEdge(15, 13);

    const SCBrepLoop independentA(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop independentB(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});
    const SCBrepLoop competingA0(
        {SCBrepCoedge(7, false), SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false)});
    const SCBrepLoop competingA1(
        {SCBrepCoedge(11, false), SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(8, true)});
    const SCBrepLoop competingB0(
        {SCBrepCoedge(14, false), SCBrepCoedge(15, false), SCBrepCoedge(16, false), SCBrepCoedge(17, false)});
    const SCBrepLoop competingB1(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(15, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace independentFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentA);
    const SCBrepFace independentFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), independentB);
    const SCBrepFace competingFaceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), competingA0);
    const SCBrepFace competingFaceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), competingA1);
    const SCBrepFace competingFaceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), competingB0);
    const SCBrepFace competingFaceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), competingB1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({independentFaceA, independentFaceB}, false),
                              SCBrepShell({competingFaceA0, competingFaceA1}, false),
                              SCBrepShell({competingFaceB0, competingFaceB1}, false),
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
    std::vector<SCBrepVertex> vertices{// Partial-overlap shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Partial-overlap shell B. Its boundary only overlaps shell A on a
                                       // sub-interval, never on a whole boundary edge.
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

// Demonstrates the conservative shared-boundary-edge arbitration still composes
// with non-competing shells in the same body: an independent shell and a
// vertex-touch shell can both close while the competing pair stays open.
TEST(Healing3dCapabilityTest, AggressiveHealingClosesIndependentAndVertexTouchShellsWhileSkippingCompetingPair)
{
    std::vector<SCBrepVertex> vertices{// Independent shell.
                                       SCBrepVertex(SCPoint3d{-4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-2.0, 1.0, 0.0}),
                                       // Competing shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Competing shell B shares boundary edge (10,11) with shell A.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       // Vertex-touch shell C shares only vertex 15 with shell B.
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

    // Independent shell.
    addEdge(0, 1);
    addEdge(1, 2);  // shared interior edge
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    // Competing shell A.
    addEdge(6, 7);
    addEdge(7, 8);  // shared interior edge
    addEdge(8, 9);
    addEdge(9, 6);
    addEdge(7, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    // Competing shell B shares boundary edge (10,11) with shell A.
    addEdge(10, 12);
    addEdge(12, 13);  // shared interior edge
    addEdge(13, 11);
    addEdge(11, 10);
    addEdge(12, 14);
    addEdge(14, 15);
    addEdge(15, 13);

    // Vertex-touch shell C shares only vertex 15 with shell B.
    addEdge(15, 16);
    addEdge(16, 17);  // shared interior edge
    addEdge(17, 18);
    addEdge(18, 15);
    addEdge(16, 19);
    addEdge(19, 20);
    addEdge(20, 17);

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

// Demonstrates the same arbitration still composes in a mixed body containing
// an already-closed shell: the closed shell stays unchanged, the competing pair
// stays open, and the vertex-touch shell can still close.
TEST(Healing3dCapabilityTest, AggressiveHealingKeepsClosedShellAndCompetingPairWhileClosingVertexTouchShell)
{
    std::vector<SCBrepVertex> vertices{// Closed shell.
                                       SCBrepVertex(SCPoint3d{-4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{-4.0, 1.0, 0.0}),
                                       // Competing shell A.
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{2.0, 1.0, 0.0}),
                                       // Competing shell B shares boundary edge (8,9) with shell A.
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       // Vertex-touch shell C shares only vertex 13 with shell B.
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

    // Closed shell.
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);

    // Competing shell A.
    addEdge(4, 5);
    addEdge(5, 6);  // shared interior edge
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(5, 8);
    addEdge(8, 9);
    addEdge(9, 6);

    // Competing shell B shares boundary edge (8,9) with shell A.
    addEdge(8, 10);
    addEdge(10, 11);  // shared interior edge
    addEdge(11, 9);
    addEdge(9, 8);
    addEdge(10, 12);
    addEdge(12, 13);
    addEdge(13, 11);

    // Vertex-touch shell C shares only vertex 13 with shell B.
    addEdge(13, 14);
    addEdge(14, 15);  // shared interior edge
    addEdge(15, 16);
    addEdge(16, 13);
    addEdge(14, 17);
    addEdge(17, 18);
    addEdge(18, 15);

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
    const SCBrepLoop shellC0(
        {SCBrepCoedge(18, false), SCBrepCoedge(19, false), SCBrepCoedge(20, false), SCBrepCoedge(21, false)});
    const SCBrepLoop shellC1(
        {SCBrepCoedge(22, false), SCBrepCoedge(23, false), SCBrepCoedge(24, false), SCBrepCoedge(19, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), closedOuterReversed);
    const SCBrepFace faceA0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA0);
    const SCBrepFace faceA1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellA1);
    const SCBrepFace faceB0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB0);
    const SCBrepFace faceB1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellB1);
    const SCBrepFace faceC0(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC0);
    const SCBrepFace faceC1(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), shellC1);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
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
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(3).FaceCount(), 3);
    ASSERT_EQ(healed.body.FaceCount(), 9);
    ASSERT_TRUE(healed.body.ShellAt(3).FaceAt(2).OuterTrim().IsValid());
}

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

// Demonstrates deterministic mixed behavior is preserved when the eligible
// support-mismatch shell also requires trim backfill before closure.
TEST(Healing3dCapabilityTest, AggressiveHealingMixedSupportMismatchWithTrimBackfillAndIneligibleMultiFace)
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

    // Closed shell
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Eligible shell
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Ineligible shell with shared edge 9->10
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
    // Omit eligible trims to force conservative backfill under support
    // mismatch.
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

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    ASSERT_EQ(healed.body.FaceCount(), 6);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
}

// Demonstrates deterministic behavior when eligible holed shell has
// support-mismatch + missing trims and coexists with ineligible multiface
// shell.
TEST(Healing3dCapabilityTest, AggressiveHealingSupportMismatchHoledEligibleWithIneligibleMultiFace)
{
    std::vector<SCBrepVertex> vertices{// Closed shell
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible holed shell
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 3.0, 0.0}),
                                       // Ineligible multi-face shell
                                       SCBrepVertex(SCPoint3d{9.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{10.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{10.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{11.0, 0.0, 0.0}),
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
    // Eligible holed shell
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);
    // Ineligible multiface shell (shared edge 13->14)
    addEdge(12, 13);  // 12
    addEdge(13, 14);  // 13 shared
    addEdge(14, 15);  // 14
    addEdge(15, 12);  // 15
    addEdge(13, 16);  // 16
    addEdge(16, 17);  // 17
    addEdge(17, 14);  // 18

    const SCBrepLoop closedOuter(
        {SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop closedOuterReversed(
        {SCBrepCoedge(3, true), SCBrepCoedge(2, true), SCBrepCoedge(1, true), SCBrepCoedge(0, true)});
    const SCBrepLoop eligibleOuter(
        {SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop eligibleHole(
        {SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});
    const SCBrepLoop ineligibleA(
        {SCBrepCoedge(12, false), SCBrepCoedge(13, false), SCBrepCoedge(14, false), SCBrepCoedge(15, false)});
    const SCBrepLoop ineligibleB(
        {SCBrepCoedge(16, false), SCBrepCoedge(17, false), SCBrepCoedge(18, false), SCBrepCoedge(13, true)});

    const SCPlaneSurface closedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface eligibleMismatch =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{9.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuterReversed);
    // Omit trims for eligible holed shell to force backfill before closure.
    const SCBrepFace eligibleFace(
        std::shared_ptr<ISCSurface>(eligibleMismatch.Clone().release()), eligibleOuter, {eligibleHole});
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

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    // closed:2, eligible holed:1->2, ineligible multiface:2 unchanged.
    ASSERT_EQ(healed.body.FaceCount(), 6);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    ASSERT_TRUE(eligibleHealedFace.OuterTrim().IsValid());
    ASSERT_EQ(eligibleHealedFace.HoleTrims().size(), 1);
    ASSERT_TRUE(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates deterministic behavior when support-mismatch eligible
// multi-face shell (holed + plain) omits trims and coexists with
// an ineligible multi-face shell.
TEST(Healing3dCapabilityTest, AggressiveHealingSupportMismatchEligibleMultiFaceMissingTrimsWithIneligibleMultiFace)
{
    std::vector<SCBrepVertex> vertices{// Closed shell
                                       SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{0.0, 1.0, 0.0}),
                                       // Eligible multi-face shell: face A holed
                                       SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{7.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{3.0, 4.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{6.0, 3.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{4.0, 3.0, 0.0}),
                                       // Eligible face B plain
                                       SCBrepVertex(SCPoint3d{8.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{9.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{8.0, 1.0, 0.0}),
                                       // Ineligible multi-face shell
                                       SCBrepVertex(SCPoint3d{11.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{12.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{12.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{11.0, 1.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{13.0, 0.0, 0.0}),
                                       SCBrepVertex(SCPoint3d{13.0, 1.0, 0.0})};

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
    // Ineligible multi-face shell with shared edge 17->18
    addEdge(16, 17);  // 16
    addEdge(17, 18);  // 17 shared
    addEdge(18, 19);  // 18
    addEdge(19, 16);  // 19
    addEdge(17, 20);  // 20
    addEdge(20, 21);  // 21
    addEdge(21, 18);  // 22

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
    const SCBrepLoop ineligibleA(
        {SCBrepCoedge(16, false), SCBrepCoedge(17, false), SCBrepCoedge(18, false), SCBrepCoedge(19, false)});
    const SCBrepLoop ineligibleB(
        {SCBrepCoedge(20, false), SCBrepCoedge(21, false), SCBrepCoedge(22, false), SCBrepCoedge(17, true)});

    const SCPlaneSurface closedSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface eligibleMismatch =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}));
    const SCPlaneSurface ineligibleSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{11.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    const SCBrepFace closedFaceA(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuter);
    const SCBrepFace closedFaceB(std::shared_ptr<ISCSurface>(closedSurface.Clone().release()), closedOuterReversed);
    // Omit trims for both eligible faces to force conservative backfill before
    // closure.
    const SCBrepFace eligibleFaceA(
        std::shared_ptr<ISCSurface>(eligibleMismatch.Clone().release()), eligibleHoledOuter, {eligibleHoledHole});
    const SCBrepFace eligibleFaceB(std::shared_ptr<ISCSurface>(eligibleMismatch.Clone().release()), eligiblePlain);
    const SCBrepFace ineligibleFaceA(std::shared_ptr<ISCSurface>(ineligibleSurface.Clone().release()), ineligibleA);
    const SCBrepFace ineligibleFaceB(std::shared_ptr<ISCSurface>(ineligibleSurface.Clone().release()), ineligibleB);

    const SCBrepBody body(vertices,
                          edges,
                          {
                              SCBrepShell({closedFaceA, closedFaceB}, true),
                              SCBrepShell({eligibleFaceA, eligibleFaceB}, false),
                              SCBrepShell({ineligibleFaceA, ineligibleFaceB}, false),
                          });
    ASSERT_TRUE(body.IsValid());

    const BrepHealing3d healed = Heal(body, SCGeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    ASSERT_TRUE(healed.success);
    ASSERT_TRUE(healed.body.IsValid());
    ASSERT_EQ(healed.body.ShellCount(), 3);
    ASSERT_TRUE(healed.body.ShellAt(0).IsClosed());
    ASSERT_TRUE(healed.body.ShellAt(1).IsClosed());
    ASSERT_FALSE(healed.body.ShellAt(2).IsClosed());
    ASSERT_EQ(healed.body.ShellAt(0).FaceCount(), 2);
    ASSERT_EQ(healed.body.ShellAt(1).FaceCount(), 4);
    ASSERT_EQ(healed.body.ShellAt(2).FaceCount(), 2);
    // closed:2, eligible multi-face:2->4, ineligible multiface:2 unchanged.
    ASSERT_EQ(healed.body.FaceCount(), 8);
    const auto eligibleHoledFace = healed.body.ShellAt(1).FaceAt(0);
    const auto eligiblePlainFace = healed.body.ShellAt(1).FaceAt(1);
    ASSERT_TRUE(eligibleHoledFace.OuterTrim().IsValid());
    ASSERT_EQ(eligibleHoledFace.HoleTrims().size(), 1);
    ASSERT_TRUE(eligibleHoledFace.HoleTrims()[0].IsValid());
    ASSERT_TRUE(eligiblePlainFace.OuterTrim().IsValid());
}

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
