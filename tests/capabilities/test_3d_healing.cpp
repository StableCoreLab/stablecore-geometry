#include <cassert>

#include <gtest/gtest.h>

#include "sdk/Geometry.h"
#include "support/Fixtures3d.h"

using geometry::sdk::Heal;
using geometry::sdk::HealingIssue3d;
using geometry::sdk::HealingPolicy3d;
using geometry::sdk::Line3d;
using geometry::sdk::LineCurve3d;
using geometry::sdk::Plane;
using geometry::sdk::PlaneSurface;
using geometry::sdk::PolyhedronBody;
using geometry::sdk::PolyhedronHealing3d;
using geometry::sdk::Point3d;
using geometry::sdk::Vector3d;
using geometry::sdk::BrepBody;
using geometry::sdk::BrepCoedge;
using geometry::sdk::BrepEdge;
using geometry::sdk::BrepFace;
using geometry::sdk::BrepLoop;
using geometry::sdk::BrepShell;
using geometry::sdk::BrepVertex;
using geometry::sdk::BrepHealing3d;
using geometry::sdk::Intervald;
using geometry::sdk::Surface;

// Demonstrates that the conservative healing pass preserves an already-valid
// PolyhedronBody without altering face count or validity.
// The "aggressive shell repair policy" (topology-changing repair) and
// "multi-step mesh/body joint healing" remain open gaps.
TEST(Healing3dCapabilityTest, UnitCubePolyhedronBodyHealingPreservesAllSixFaces)
{
    const PolyhedronBody cubeBody = geometry::test::BuildUnitCubeBody();
    assert(cubeBody.IsValid());
    assert(cubeBody.FaceCount() == 6);

    const PolyhedronHealing3d healed = Heal(cubeBody);
    assert(healed.success);
    assert(healed.issue == HealingIssue3d::None);
    assert(healed.body.IsValid());
    assert(healed.body.FaceCount() == 6);
}

// Demonstrates trim backfill on a planar line-edge BrepFace where topology is
// present but trim curves are intentionally omitted.
TEST(Healing3dCapabilityTest, PlanarBrepFaceWithoutTrimIsHealedWithBackfilledTrim)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0})};

    std::vector<BrepEdge> edges;
    edges.emplace_back(
        std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
            Line3d::FromOriginAndDirection(Point3d{0.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
            Intervald{0.0, 1.0})),
        0,
        1);
    edges.emplace_back(
        std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
            Line3d::FromOriginAndDirection(Point3d{1.0, 0.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
            Intervald{0.0, 1.0})),
        1,
        2);
    edges.emplace_back(
        std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
            Line3d::FromOriginAndDirection(Point3d{1.0, 1.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
            Intervald{0.0, 1.0})),
        2,
        3);
    edges.emplace_back(
        std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
            Line3d::FromOriginAndDirection(Point3d{0.0, 1.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
            Intervald{0.0, 1.0})),
        3,
        0);

    const BrepLoop outerLoop({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const BrepFace face(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerLoop);
    const BrepBody body(vertices, edges, {BrepShell({face}, false)});

    assert(body.IsValid());
    assert(!face.OuterTrim().IsValid());

    const BrepHealing3d healed = Heal(body);
    assert(healed.success);
    assert(healed.issue == HealingIssue3d::None);
    assert(healed.body.IsValid());
    assert(healed.body.FaceCount() == 1);
    const auto healedFace = healed.body.ShellAt(0).FaceAt(0);
    assert(healedFace.OuterTrim().IsValid());
    assert(healedFace.OuterTrim().PointCount() == 4);
}

// Demonstrates conservative healing can backfill both outer and hole trims for
// a planar line-edge BrepFace when trims are omitted.
TEST(Healing3dCapabilityTest, PlanarHoledBrepFaceWithoutAnyTrimIsHealed)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{3.0, 3.0, 0.0}),
        BrepVertex(Point3d{0.0, 3.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{2.0, 1.0, 0.0}),
        BrepVertex(Point3d{2.0, 2.0, 0.0}),
        BrepVertex(Point3d{1.0, 2.0, 0.0})};

    std::vector<BrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const Point3d first = vertices[start].Point();
        const Point3d second = vertices[end].Point();
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
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

    const BrepLoop outerLoop({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop holeLoop({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const BrepFace face(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const BrepBody body(vertices, edges, {BrepShell({face}, false)});

    assert(body.IsValid());
    assert(!face.OuterTrim().IsValid());
    assert(face.HoleTrims().empty());

    const BrepHealing3d healed = Heal(body);
    assert(healed.success);
    assert(healed.issue == HealingIssue3d::None);
    assert(healed.body.IsValid());
    assert(healed.body.FaceCount() == 1);
    const auto healedFace = healed.body.ShellAt(0).FaceAt(0);
    assert(healedFace.OuterTrim().IsValid());
    assert(healedFace.HoleTrims().size() == 1);
    assert(healedFace.HoleTrims()[0].IsValid());
    assert(healedFace.HoleTrims()[0].PointCount() == 4);
}

// Demonstrates a deterministic minimal aggressive policy: a recoverable open
// planar single-face shell can be closed by adding an opposite-oriented face.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableSingleFaceShell)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0})};

    std::vector<BrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const Point3d first = vertices[start].Point();
        const Point3d second = vertices[end].Point();
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
            start,
            end);
    };
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);

    const BrepLoop outerLoop({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const BrepFace face(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerLoop);
    const BrepBody openBody(vertices, edges, {BrepShell({face}, false)});
    assert(openBody.IsValid());
    assert(!openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.issue == HealingIssue3d::None);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 1);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.FaceCount() == 2);
}

// Demonstrates aggressive policy also closes a recoverable planar multi-face
// open-sheet shell where each edge is single-use before repair.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableMultiFaceOpenSheet)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        BrepVertex(Point3d{2.0, 0.0, 0.0}),
        BrepVertex(Point3d{2.0, 1.0, 0.0})};

    std::vector<BrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const Point3d first = vertices[start].Point();
        const Point3d second = vertices[end].Point();
        edges.emplace_back(
            std::make_shared<LineCurve3d>(LineCurve3d::FromLine(
                Line3d::FromOriginAndDirection(first, second - first),
                Intervald{0.0, 1.0})),
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

    const BrepLoop outerA({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop outerB({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace faceA(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerA);
    const BrepFace faceB(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerB);
    const BrepBody openBody(vertices, edges, {BrepShell({faceA, faceB}, false)});
    assert(openBody.IsValid());
    assert(!openBody.ShellAt(0).IsClosed());
    assert(openBody.FaceCount() == 2);

    const BrepHealing3d healed = Heal(openBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 1);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.FaceCount() == 4);
}
