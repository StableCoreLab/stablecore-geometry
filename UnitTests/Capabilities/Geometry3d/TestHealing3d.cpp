
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <vector>

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
