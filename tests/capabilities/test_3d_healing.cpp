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

// Demonstrates aggressive closure also supports a recoverable holed planar
// single-face open shell by mirroring outer and hole loops.
TEST(Healing3dCapabilityTest, AggressiveHealingCanCloseRecoverableHoledOpenShell)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 4.0, 0.0}),
        BrepVertex(Point3d{0.0, 4.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 3.0, 0.0}),
        BrepVertex(Point3d{1.0, 3.0, 0.0})};

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
    const BrepFace frontFace(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const BrepBody openBody(vertices, edges, {BrepShell({frontFace}, false)});
    assert(openBody.IsValid());
    assert(!openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 1);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.FaceCount() == 2);
}

// Demonstrates aggressive closure composes with conservative trim backfill on
// a holed open shell where both outer/hole trims are initially missing.
TEST(Healing3dCapabilityTest, AggressiveHealingCompositeHoledOpenShellWithMissingTrims)
{
    std::vector<BrepVertex> vertices{
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 4.0, 0.0}),
        BrepVertex(Point3d{0.0, 4.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 3.0, 0.0}),
        BrepVertex(Point3d{1.0, 3.0, 0.0})};

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

    // Intentionally omit outer/hole trims to force conservative backfill first.
    const BrepFace frontFace(std::shared_ptr<Surface>(planeSurface.Clone().release()), outerLoop, {holeLoop});
    const BrepBody openBody(vertices, edges, {BrepShell({frontFace}, false)});
    assert(openBody.IsValid());
    assert(!openBody.ShellAt(0).IsClosed());

    const BrepHealing3d healed = Heal(openBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 1);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.FaceCount() == 2);
    const auto firstFace = healed.body.ShellAt(0).FaceAt(0);
    assert(firstFace.OuterTrim().IsValid());
    assert(firstFace.HoleTrims().size() == 1);
    assert(firstFace.HoleTrims()[0].IsValid());
}

// Demonstrates aggressive policy can deterministically close multiple
// recoverable open shells within the same BrepBody.
TEST(Healing3dCapabilityTest, AggressiveHealingClosesMultipleOpenShells)
{
    std::vector<BrepVertex> vertices{
        // Shell A
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Shell B
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0})};

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

    const BrepLoop loopA({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop loopB({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace faceA(std::shared_ptr<Surface>(planeSurface.Clone().release()), loopA);
    const BrepFace faceB(std::shared_ptr<Surface>(planeSurface.Clone().release()), loopB);
    const BrepBody openBody(vertices, edges, {BrepShell({faceA}, false), BrepShell({faceB}, false)});
    assert(openBody.IsValid());
    assert(!openBody.ShellAt(0).IsClosed());
    assert(!openBody.ShellAt(1).IsClosed());

    const BrepHealing3d healed = Heal(openBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 2);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    // Each open single-face shell is mirrored into 2 faces.
    assert(healed.body.FaceCount() == 4);
}

// Demonstrates aggressive policy can close only the recoverable open shell in
// a mixed input while keeping already-closed shells topologically stable.
TEST(Healing3dCapabilityTest, AggressiveHealingPreservesClosedShellAndClosesOpenShell)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Open shell vertices
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop openOuter({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});

    const PlaneSurface planeSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const BrepFace closedFaceA(std::shared_ptr<Surface>(planeSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planeSurface.Clone().release()), closedOuterReversed);
    const BrepFace openFace(std::shared_ptr<Surface>(planeSurface.Clone().release()), openOuter);

    const BrepBody mixedBody(
        vertices,
        edges,
        {BrepShell({closedFaceA, closedFaceB}, true), BrepShell({openFace}, false)});
    assert(mixedBody.IsValid());
    assert(mixedBody.ShellCount() == 2);
    assert(mixedBody.ShellAt(0).IsClosed());
    assert(!mixedBody.ShellAt(1).IsClosed());
    assert(mixedBody.FaceCount() == 3);

    const BrepHealing3d healed = Heal(mixedBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 2);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    // Closed shell keeps 2 faces, open shell is mirrored from 1 to 2 faces.
    assert(healed.body.FaceCount() == 4);
}

// Demonstrates aggressive policy can partially repair a mixed open-shell set:
// eligible planar shell is closed, unsupported non-planar shell remains open.
TEST(Healing3dCapabilityTest, AggressiveHealingPartiallyRepairsMixedOpenShells)
{
    std::vector<BrepVertex> vertices{
        // Planar open shell (eligible)
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Non-planar open shell (ineligible for aggressive closure)
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.1}),
        BrepVertex(Point3d{3.0, 1.0, 0.0})};

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

    const BrepLoop planarLoop({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop nonPlanarLoop({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface nonPlanarSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{3.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace planarFace(std::shared_ptr<Surface>(planarSurface.Clone().release()), planarLoop);
    const BrepFace nonPlanarFace(std::shared_ptr<Surface>(nonPlanarSupport.Clone().release()), nonPlanarLoop);
    const BrepBody mixedOpenBody(vertices, edges, {BrepShell({planarFace}, false), BrepShell({nonPlanarFace}, false)});
    assert(mixedOpenBody.IsValid());
    assert(!mixedOpenBody.ShellAt(0).IsClosed());
    assert(!mixedOpenBody.ShellAt(1).IsClosed());
    assert(mixedOpenBody.FaceCount() == 2);

    const BrepHealing3d healed = Heal(mixedOpenBody, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 2);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(!healed.body.ShellAt(1).IsClosed());
    // Shell 0 is mirrored (1->2 faces), shell 1 remains single-face open.
    assert(healed.body.FaceCount() == 3);
}

// Demonstrates deterministic mixed-shell behavior on three shells: preserve
// closed shell, close eligible open shell, and keep ineligible open shell.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellMixedDeterministicBehavior)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible open shell vertices
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        // Ineligible open shell vertices
        BrepVertex(Point3d{6.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 1.0, 0.15}),
        BrepVertex(Point3d{6.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleOpen({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop ineligibleOpen({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{6.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    const BrepFace eligibleFace(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleOpen);
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleOpen);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleFace}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());
    assert(body.ShellCount() == 3);
    assert(body.ShellAt(0).IsClosed());
    assert(!body.ShellAt(1).IsClosed());
    assert(!body.ShellAt(2).IsClosed());
    assert(body.FaceCount() == 4);

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    // Closed shell keeps 2 faces, eligible open shell becomes 2 faces, ineligible remains 1.
    assert(healed.body.FaceCount() == 5);
}

// Demonstrates mixed three-shell behavior remains deterministic even when the
// eligible open shell also requires conservative trim backfill.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellMixedWithEligibleTrimBackfill)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible open shell vertices
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        // Ineligible open shell vertices
        BrepVertex(Point3d{6.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 1.0, 0.12}),
        BrepVertex(Point3d{6.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleOpen({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop ineligibleOpen({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{6.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    // Omit trims on eligible face to force conservative backfill before aggressive closure.
    const BrepFace eligibleFace(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleOpen);
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleOpen);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleFace}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    assert(healed.body.FaceCount() == 5);
    const auto eligibleFrontFace = healed.body.ShellAt(1).FaceAt(0);
    assert(eligibleFrontFace.OuterTrim().IsValid());
}

// Demonstrates deterministic mixed-shell behavior when the eligible shell is
// a multi-face planar open sheet.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellWithEligibleMultiFaceOpenSheet)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible open-sheet vertices (two faces)
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 0.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{3.0, 1.0, 0.0}),
        BrepVertex(Point3d{5.0, 0.0, 0.0}),
        BrepVertex(Point3d{5.0, 1.0, 0.0}),
        // Ineligible open shell vertices
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{8.0, 0.0, 0.0}),
        BrepVertex(Point3d{8.0, 1.0, 0.14}),
        BrepVertex(Point3d{7.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleA({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop eligibleB({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});
    const BrepLoop ineligibleLoop({BrepCoedge(12, false), BrepCoedge(13, false), BrepCoedge(14, false), BrepCoedge(15, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{7.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    const BrepFace eligibleFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleA);
    const BrepFace eligibleFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleB);
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleFaceA, eligibleFaceB}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());
    assert(body.FaceCount() == 5);

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    // Closed shell stays 2 faces; eligible shell mirrors 2->4; ineligible stays 1.
    assert(healed.body.FaceCount() == 7);
}

// Demonstrates three-shell mixed behavior with eligible holed shell where
// conservative trim backfill and aggressive closure compose deterministically.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellWithEligibleHoledShellAndMissingTrims)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible holed open shell vertices
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 4.0, 0.0}),
        BrepVertex(Point3d{3.0, 4.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 3.0, 0.0}),
        BrepVertex(Point3d{4.0, 3.0, 0.0}),
        // Ineligible open shell vertices
        BrepVertex(Point3d{9.0, 0.0, 0.0}),
        BrepVertex(Point3d{10.0, 0.0, 0.0}),
        BrepVertex(Point3d{10.0, 1.0, 0.15}),
        BrepVertex(Point3d{9.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleOuter({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop eligibleHole({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});
    const BrepLoop ineligibleLoop({BrepCoedge(12, false), BrepCoedge(13, false), BrepCoedge(14, false), BrepCoedge(15, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{9.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    // Omit trims on eligible holed face to force backfill of outer/hole trims.
    const BrepFace eligibleHoledFace(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleOuter, {eligibleHole});
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleHoledFace}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    // Closed shell stays 2 faces; eligible holed shell mirrors 1->2; ineligible stays 1.
    assert(healed.body.FaceCount() == 5);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    assert(eligibleHealedFace.OuterTrim().IsValid());
    assert(eligibleHealedFace.HoleTrims().size() == 1);
    assert(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates three-shell mixed behavior where eligible shell is multi-face
// and includes a holed face with missing trims before aggressive closure.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellEligibleMultiFaceHoledMissingTrims)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible face A (holed)
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 4.0, 0.0}),
        BrepVertex(Point3d{3.0, 4.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 3.0, 0.0}),
        BrepVertex(Point3d{4.0, 3.0, 0.0}),
        // Eligible face B (plain)
        BrepVertex(Point3d{8.0, 0.0, 0.0}),
        BrepVertex(Point3d{9.0, 0.0, 0.0}),
        BrepVertex(Point3d{9.0, 1.0, 0.0}),
        BrepVertex(Point3d{8.0, 1.0, 0.0}),
        // Ineligible shell
        BrepVertex(Point3d{11.0, 0.0, 0.0}),
        BrepVertex(Point3d{12.0, 0.0, 0.0}),
        BrepVertex(Point3d{12.0, 1.0, 0.13}),
        BrepVertex(Point3d{11.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleHoledOuter({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop eligibleHoledHole({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});
    const BrepLoop eligiblePlain({BrepCoedge(12, false), BrepCoedge(13, false), BrepCoedge(14, false), BrepCoedge(15, false)});
    const BrepLoop ineligibleLoop({BrepCoedge(16, false), BrepCoedge(17, false), BrepCoedge(18, false), BrepCoedge(19, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{11.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    const BrepFace eligibleFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleHoledOuter, {eligibleHoledHole});
    const BrepFace eligibleFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligiblePlain);
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleFaceA, eligibleFaceB}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    // closed:2, eligible multi-face:2->4, ineligible:1 => total 7.
    assert(healed.body.FaceCount() == 7);
    const auto eligibleHealedFace = healed.body.ShellAt(1).FaceAt(0);
    assert(eligibleHealedFace.OuterTrim().IsValid());
    assert(eligibleHealedFace.HoleTrims().size() == 1);
    assert(eligibleHealedFace.HoleTrims()[0].IsValid());
}

// Demonstrates deterministic composition when eligible multi-face shell has
// both holed and plain faces with missing trims before aggressive closure.
TEST(Healing3dCapabilityTest, AggressiveHealingThreeShellEligibleMultiFaceBothTrimsMissing)
{
    std::vector<BrepVertex> vertices{
        // Closed shell vertices
        BrepVertex(Point3d{0.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 0.0, 0.0}),
        BrepVertex(Point3d{1.0, 1.0, 0.0}),
        BrepVertex(Point3d{0.0, 1.0, 0.0}),
        // Eligible face A (holed)
        BrepVertex(Point3d{3.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 0.0, 0.0}),
        BrepVertex(Point3d{7.0, 4.0, 0.0}),
        BrepVertex(Point3d{3.0, 4.0, 0.0}),
        BrepVertex(Point3d{4.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 1.0, 0.0}),
        BrepVertex(Point3d{6.0, 3.0, 0.0}),
        BrepVertex(Point3d{4.0, 3.0, 0.0}),
        // Eligible face B (plain)
        BrepVertex(Point3d{8.0, 0.0, 0.0}),
        BrepVertex(Point3d{9.0, 0.0, 0.0}),
        BrepVertex(Point3d{9.0, 1.0, 0.0}),
        BrepVertex(Point3d{8.0, 1.0, 0.0}),
        // Ineligible shell
        BrepVertex(Point3d{11.0, 0.0, 0.0}),
        BrepVertex(Point3d{12.0, 0.0, 0.0}),
        BrepVertex(Point3d{12.0, 1.0, 0.11}),
        BrepVertex(Point3d{11.0, 1.0, 0.0})};

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

    const BrepLoop closedOuter({BrepCoedge(0, false), BrepCoedge(1, false), BrepCoedge(2, false), BrepCoedge(3, false)});
    const BrepLoop closedOuterReversed({BrepCoedge(3, true), BrepCoedge(2, true), BrepCoedge(1, true), BrepCoedge(0, true)});
    const BrepLoop eligibleHoledOuter({BrepCoedge(4, false), BrepCoedge(5, false), BrepCoedge(6, false), BrepCoedge(7, false)});
    const BrepLoop eligibleHoledHole({BrepCoedge(8, false), BrepCoedge(9, false), BrepCoedge(10, false), BrepCoedge(11, false)});
    const BrepLoop eligiblePlain({BrepCoedge(12, false), BrepCoedge(13, false), BrepCoedge(14, false), BrepCoedge(15, false)});
    const BrepLoop ineligibleLoop({BrepCoedge(16, false), BrepCoedge(17, false), BrepCoedge(18, false), BrepCoedge(19, false)});

    const PlaneSurface planarSurface = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));
    const PlaneSurface ineligibleSupport = PlaneSurface::FromPlane(
        Plane::FromPointAndNormal(Point3d{11.0, 0.0, 0.0}, Vector3d{0.0, 0.0, 1.0}));

    const BrepFace closedFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuter);
    const BrepFace closedFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), closedOuterReversed);
    // Both eligible faces intentionally omit trims.
    const BrepFace eligibleFaceA(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligibleHoledOuter, {eligibleHoledHole});
    const BrepFace eligibleFaceB(std::shared_ptr<Surface>(planarSurface.Clone().release()), eligiblePlain);
    const BrepFace ineligibleFace(std::shared_ptr<Surface>(ineligibleSupport.Clone().release()), ineligibleLoop);

    const BrepBody body(
        vertices,
        edges,
        {
            BrepShell({closedFaceA, closedFaceB}, true),
            BrepShell({eligibleFaceA, eligibleFaceB}, false),
            BrepShell({ineligibleFace}, false),
        });
    assert(body.IsValid());

    const BrepHealing3d healed = Heal(body, geometry::sdk::GeometryTolerance3d{}, HealingPolicy3d::Aggressive);
    assert(healed.success);
    assert(healed.body.IsValid());
    assert(healed.body.ShellCount() == 3);
    assert(healed.body.ShellAt(0).IsClosed());
    assert(healed.body.ShellAt(1).IsClosed());
    assert(!healed.body.ShellAt(2).IsClosed());
    assert(healed.body.FaceCount() == 7);
    const auto holedHealedFace = healed.body.ShellAt(1).FaceAt(0);
    const auto plainHealedFace = healed.body.ShellAt(1).FaceAt(1);
    assert(holedHealedFace.OuterTrim().IsValid());
    assert(holedHealedFace.HoleTrims().size() == 1);
    assert(holedHealedFace.HoleTrims()[0].IsValid());
    assert(plainHealedFace.OuterTrim().IsValid());
}
