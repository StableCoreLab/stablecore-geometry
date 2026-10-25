
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
