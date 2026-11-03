#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/ConversionFixturesBasic3d.h"

using Geometry::SCBrepBody;
using Geometry::SCBrepCoedge;
using Geometry::BrepConversionIssue3d;
using Geometry::SCBrepEdge;
using Geometry::SCBrepFace;
using Geometry::SCBrepLoop;
using Geometry::SCBrepShell;
using Geometry::SCBrepVertex;
using Geometry::ComputeTriangleConnectedComponents;
using Geometry::ConvertToBrepBody;
using Geometry::ConvertToTriangleMesh;
using Geometry::SCCurveOnSurface;
using Geometry::SCIntervald;
using Geometry::SCLine3d;
using Geometry::SCLineCurve3d;
using Geometry::SCPlane;
using Geometry::SCPlaneSurface;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronBrepBodyConversion3d;
using Geometry::PolyhedronMeshConversion3d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::ISCSurface;
using Geometry::SCVector3d;

using namespace Geometry::Test::ConversionFixtures;

// Demonstrates planar holed SCBrepBody conversion keeps representative area by
// honoring hole trims in mesh triangulation.
TEST(Conversion3dCapabilityTest, PlanarHoledBrepBodyConvertsToMeshWithExpectedArea)
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

    const SCBrepLoop outerLoop({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeLoop({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    auto supportSurface = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());
    const SCCurveOnSurface outerTrim(supportSurface,
                                   SCPolyline2d(
                                       {
                                           SCPoint2d{0.0, 0.0},
                                           SCPoint2d{4.0, 0.0},
                                           SCPoint2d{4.0, 4.0},
                                           SCPoint2d{0.0, 4.0},
                                       },
                                       SCPolylineClosure::Closed));
    const SCCurveOnSurface holeTrim(supportSurface,
                                  SCPolyline2d(
                                      {
                                          SCPoint2d{1.0, 1.0},
                                          SCPoint2d{3.0, 1.0},
                                          SCPoint2d{3.0, 3.0},
                                          SCPoint2d{1.0, 3.0},
                                      },
                                      SCPolylineClosure::Closed));

    const SCBrepFace face(
        std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerLoop, {holeLoop}, outerTrim, {holeTrim});
    const SCBrepBody body(vertices, edges, {SCBrepShell({face}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: 4x4 outer minus 2x2 hole = 12.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 12.0), 1e-8);
}

// Demonstrates planar multi-face SCBrepBody conversion keeps representative
// aggregate area across disconnected faces.
TEST(Conversion3dCapabilityTest, PlanarMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<SCBrepVertex> vertices{// Face A square: area 4
                                     SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{2.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 2.0, 0.0}),
                                     // Face B rectangle: area 2
                                     SCBrepVertex(SCPoint3d{3.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Face A
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face B
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    auto supportA = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());
    auto supportB = std::shared_ptr<ISCSurface>(planeSurface.Clone().release());

    const SCCurveOnSurface trimA(supportA,
                               SCPolyline2d(
                                   {
                                       SCPoint2d{0.0, 0.0},
                                       SCPoint2d{2.0, 0.0},
                                       SCPoint2d{2.0, 2.0},
                                       SCPoint2d{0.0, 2.0},
                                   },
                                   SCPolylineClosure::Closed));
    const SCCurveOnSurface trimB(supportB,
                               SCPolyline2d(
                                   {
                                       SCPoint2d{3.0, 0.0},
                                       SCPoint2d{4.0, 0.0},
                                       SCPoint2d{4.0, 2.0},
                                       SCPoint2d{3.0, 2.0},
                                   },
                                   SCPolylineClosure::Closed));

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA, {}, trimA, {});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB, {}, trimB, {});
    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: 4 + 2 = 6.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 6.0), 1e-8);
}

// Demonstrates representative shared-edge Brep->mesh conversion can preserve
// shared geometric vertices globally instead of duplicating each face mesh.
TEST(Conversion3dCapabilityTest, PlanarSharedEdgeBrepBodyConvertsToMeshWithSharedVertices)
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
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    addEdge(1, 4);
    addEdge(4, 5);
    addEdge(5, 2);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop outerB({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(1, true)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<SCPoint2d>& points) {
        return SCCurveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                              SCPolyline2d(points, SCPolylineClosure::Closed));
    };

    const SCBrepFace faceA(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                         outerA,
                         {},
                         makeTrim({
                             SCPoint2d{0.0, 0.0},
                             SCPoint2d{1.0, 0.0},
                             SCPoint2d{1.0, 1.0},
                             SCPoint2d{0.0, 1.0},
                         }),
                         {});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                         outerB,
                         {},
                         makeTrim({
                             SCPoint2d{1.0, 0.0},
                             SCPoint2d{2.0, 0.0},
                             SCPoint2d{2.0, 1.0},
                             SCPoint2d{1.0, 1.0},
                         }),
                         {});

    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    ASSERT_EQ(mesh.mesh.TriangleCount(), 4);
    ASSERT_EQ(mesh.mesh.VertexCount(), 6);
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 2.0), 1e-8);
}

// Demonstrates planar mixed content conversion: one holed face plus one
// disjoint face preserve aggregate representative area.
TEST(Conversion3dCapabilityTest, PlanarHoledAndMultiFaceBrepBodyConvertsToMeshWithExpectedArea)
{
    std::vector<SCBrepVertex> vertices{// Face A outer square (area 16)
                                     SCBrepVertex(SCPoint3d{0.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{4.0, 4.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{0.0, 4.0, 0.0}),
                                     // Face A hole square (area 4)
                                     SCBrepVertex(SCPoint3d{1.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 1.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{3.0, 3.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{1.0, 3.0, 0.0}),
                                     // Face B rectangle (area 2)
                                     SCBrepVertex(SCPoint3d{5.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{6.0, 0.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{6.0, 2.0, 0.0}),
                                     SCBrepVertex(SCPoint3d{5.0, 2.0, 0.0})};

    std::vector<SCBrepEdge> edges;
    auto addEdge = [&](std::size_t start, std::size_t end) {
        const SCPoint3d first = vertices[start].Point();
        const SCPoint3d second = vertices[end].Point();
        edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                               SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                           start,
                           end);
    };

    // Face A outer
    addEdge(0, 1);
    addEdge(1, 2);
    addEdge(2, 3);
    addEdge(3, 0);
    // Face A hole
    addEdge(4, 5);
    addEdge(5, 6);
    addEdge(6, 7);
    addEdge(7, 4);
    // Face B
    addEdge(8, 9);
    addEdge(9, 10);
    addEdge(10, 11);
    addEdge(11, 8);

    const SCBrepLoop outerA({SCBrepCoedge(0, false), SCBrepCoedge(1, false), SCBrepCoedge(2, false), SCBrepCoedge(3, false)});
    const SCBrepLoop holeA({SCBrepCoedge(4, false), SCBrepCoedge(5, false), SCBrepCoedge(6, false), SCBrepCoedge(7, false)});
    const SCBrepLoop outerB({SCBrepCoedge(8, false), SCBrepCoedge(9, false), SCBrepCoedge(10, false), SCBrepCoedge(11, false)});

    const SCPlaneSurface planeSurface =
        SCPlaneSurface::FromPlane(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));

    auto makeTrim = [&](const std::vector<SCPoint2d>& points) {
        return SCCurveOnSurface(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()),
                              SCPolyline2d(points, SCPolylineClosure::Closed));
    };

    const SCCurveOnSurface outerTrimA = makeTrim({
        SCPoint2d{0.0, 0.0},
        SCPoint2d{4.0, 0.0},
        SCPoint2d{4.0, 4.0},
        SCPoint2d{0.0, 4.0},
    });
    const SCCurveOnSurface holeTrimA = makeTrim({
        SCPoint2d{1.0, 1.0},
        SCPoint2d{3.0, 1.0},
        SCPoint2d{3.0, 3.0},
        SCPoint2d{1.0, 3.0},
    });
    const SCCurveOnSurface outerTrimB = makeTrim({
        SCPoint2d{5.0, 0.0},
        SCPoint2d{6.0, 0.0},
        SCPoint2d{6.0, 2.0},
        SCPoint2d{5.0, 2.0},
    });

    const SCBrepFace faceA(
        std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerA, {holeA}, outerTrimA, {holeTrimA});
    const SCBrepFace faceB(std::shared_ptr<ISCSurface>(planeSurface.Clone().release()), outerB, {}, outerTrimB, {});
    const SCBrepBody body(vertices, edges, {SCBrepShell({faceA, faceB}, false)});
    ASSERT_TRUE(body.IsValid());

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    // Expected area: (16 - 4) + 2 = 14.
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 14.0), 1e-8);
}

// Demonstrates disconnected closed-shell Brep->mesh conversion preserves
// component separation (feature-level connectivity) in addition to geometry.
TEST(Conversion3dCapabilityTest, TwoSeparatedCubeBrepBodyConvertsToMeshWithTwoComponents)
{
    const PolyhedronBody twoCubes = BuildTwoSeparatedUnitCubeBody();
    ASSERT_TRUE(twoCubes.IsValid());
    ASSERT_EQ(twoCubes.FaceCount(), 12);

    const PolyhedronBrepBodyConversion3d brep = ConvertToBrepBody(twoCubes);
    ASSERT_TRUE(brep.success);
    ASSERT_EQ(brep.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(brep.body.IsValid());
    ASSERT_EQ(brep.body.FaceCount(), 12);

    const PolyhedronMeshConversion3d mesh = ConvertToTriangleMesh(brep.body);
    ASSERT_TRUE(mesh.success);
    ASSERT_TRUE(mesh.mesh.IsValid());
    ASSERT_EQ(mesh.mesh.TriangleCount(), 24);
    ASSERT_LT(std::abs(mesh.mesh.SurfaceArea() - 12.0), 1e-8);

    const auto components = ComputeTriangleConnectedComponents(mesh.mesh);
    ASSERT_EQ(components.size(), 2);
    ASSERT_EQ(components[0].size(), 12);
    ASSERT_EQ(components[1].size(), 12);
}
