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
