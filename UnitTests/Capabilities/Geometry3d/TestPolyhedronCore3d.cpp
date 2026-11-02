#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(PolyhedronCore3dTest, ValidatesFaceAndBodyBounds)
{
    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    ASSERT_TRUE(outerLoop.IsValid());
    ASSERT_EQ(outerLoop.VertexCount(), 4);
    const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), outerLoop);
    ASSERT_TRUE(face.IsValid());
    ASSERT_EQ(face.HoleCount(), 0);
    const SCBox3d faceBounds = face.Bounds();
    ASSERT_TRUE(faceBounds.IsValid());
    ASSERT_TRUE(faceBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(faceBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    const SCBox3d faceMeasureBounds = Bounds(face);
    ASSERT_TRUE(faceMeasureBounds.IsValid());
    ASSERT_TRUE(faceMeasureBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    const PolyhedronBody body({face});
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 1);
    const auto bodyValidation = Validate(body);
    ASSERT_TRUE(bodyValidation.valid);
    ASSERT_EQ(bodyValidation.issue, SCPolyhedronValidationIssue3d::None);
    const SCBox3d bodyBounds = body.Bounds();
    ASSERT_TRUE(bodyBounds.IsValid());
    ASSERT_TRUE(bodyBounds.MinPoint().AlmostEquals(SCPoint3d{0.0, 0.0, 0.0}, 1e-12));
    ASSERT_TRUE(bodyBounds.MaxPoint().AlmostEquals(SCPoint3d{2.0, 2.0, 0.0}, 1e-12));
    EXPECT_NEAR(Area(face), 4.0, 1e-12);
}

TEST(PolyhedronCore3dTest, RejectsNonPlanarFace)
{
    const PolyhedronFace3d invalidFace(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                                       PolyhedronLoop3d({
                                           SCPoint3d{0.0, 0.0, 0.0},
                                           SCPoint3d{1.0, 0.0, 1.0},
                                           SCPoint3d{0.0, 1.0, 0.0},
                                       }));
    const auto invalidBodyValidation = Validate(PolyhedronBody({invalidFace}));
    ASSERT_FALSE(invalidBodyValidation.valid);
    ASSERT_EQ(invalidBodyValidation.issue, SCPolyhedronValidationIssue3d::InvalidFace);
}

TEST(PolyhedronCore3dTest, HealsOpenBody)
{
    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), outerLoop);
    const PolyhedronBody body({face});
    const PolyhedronHealing3d healedBody = Heal(body);
    ASSERT_TRUE(healedBody.success);
    ASSERT_EQ(healedBody.body.FaceCount(), 1);
}

TEST(PolyhedronCore3dTest, TessellatesPlaneSurface)
{
    const SCPlane supportPlane = Geometry::Test::BuildSupportPlaneAtZ5();
    const SCPlaneSurface planeSurface = Geometry::Test::BuildFinitePlaneSurfaceAtZ5();
    const TriangleMesh surfaceMesh = Tessellate(planeSurface, 2, 3);
    ASSERT_TRUE(surfaceMesh.IsValid());
    ASSERT_EQ(surfaceMesh.VertexCount(), 12);
    ASSERT_EQ(surfaceMesh.TriangleCount(), 12);
    const auto surfaceMeshValidation = Validate(surfaceMesh);
    ASSERT_TRUE(surfaceMeshValidation.valid);
    ASSERT_FALSE(IsClosedTriangleMesh(surfaceMesh));
    for (const SCPoint3d& vertex : surfaceMesh.Vertices())
    {
        EXPECT_NEAR(supportPlane.SignedDistanceTo(vertex), 0.0, 1e-12);
    }
    EXPECT_NEAR(surfaceMesh.SurfaceArea(), 16.0, 1e-12);
}

TEST(PolyhedronCore3dTest, ConvertsAndQueriesFace)
{
    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), outerLoop);
    const auto faceMesh = ConvertToTriangleMesh(face);
    ASSERT_TRUE(faceMesh.success);
    ASSERT_EQ(faceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(faceMesh.mesh.IsValid());
    ASSERT_EQ(faceMesh.mesh.TriangleCount(), 2);
    EXPECT_NEAR(faceMesh.mesh.SurfaceArea(), 4.0, 1e-12);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, face), SCPointContainment3d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{2.0, 1.0, 0.0}, face), SCPointContainment3d::OnBoundary);
    ASSERT_EQ(LocatePoint(SCPoint3d{3.0, 1.0, 0.0}, face), SCPointContainment3d::Outside);
    const auto polyFaceProjection = ProjectPointToPolyhedronFace(SCPoint3d{1.0, 1.0, 3.0}, face);
    ASSERT_TRUE(polyFaceProjection.success);
    ASSERT_TRUE(polyFaceProjection.IsValid());
    ASSERT_TRUE(polyFaceProjection.onFace);
    ASSERT_TRUE(polyFaceProjection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{1.0, 1.0, 3.0}, face), 3.0, 1e-12);
    const auto polyFaceLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), face);
    ASSERT_TRUE(polyFaceLineIntersection.intersects);
    ASSERT_TRUE(polyFaceLineIntersection.IsValid());
    ASSERT_TRUE(polyFaceLineIntersection.point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
}

TEST(PolyhedronCore3dTest, ConvertsAndQueriesBody)
{
    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}), outerLoop);
    const PolyhedronBody body({face});
    const auto bodyMesh = ConvertToTriangleMesh(body);
    ASSERT_TRUE(bodyMesh.success);
    ASSERT_EQ(bodyMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(bodyMesh.mesh.IsValid());
    ASSERT_EQ(bodyMesh.mesh.TriangleCount(), 2);
    const auto polyBodyProjection = ProjectPointToPolyhedronBody(SCPoint3d{3.0, 1.0, 1.0}, body);
    ASSERT_TRUE(polyBodyProjection.success);
    ASSERT_TRUE(polyBodyProjection.IsValid());
    ASSERT_EQ(polyBodyProjection.faceIndex, 0);
    ASSERT_TRUE(polyBodyProjection.projection.point.AlmostEquals(SCPoint3d{2.0, 1.0, 0.0}, 1e-12));
    EXPECT_NEAR(Distance(SCPoint3d{3.0, 1.0, 1.0}, body), std::sqrt(2.0), 1e-12);
    const auto polyBodyLineIntersection =
        Intersect(SCLine3d::FromOriginAndDirection(SCPoint3d{1.0, 1.0, 3.0}, SCVector3d{0.0, 0.0, -1.0}), body);
    ASSERT_TRUE(polyBodyLineIntersection.intersects);
    ASSERT_TRUE(polyBodyLineIntersection.IsValid());
    ASSERT_EQ(polyBodyLineIntersection.faceIndices.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.hits.size(), 1);
    ASSERT_EQ(polyBodyLineIntersection.faceIndices[0], 0);
    ASSERT_TRUE(polyBodyLineIntersection.hits[0].point.AlmostEquals(SCPoint3d{1.0, 1.0, 0.0}, 1e-12));
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 1.0, 0.0}, body), SCPointContainment3d::OnBoundary);
}

TEST(PolyhedronCore3dTest, ProjectsAndMeshesHoledFace)
{
    const PolyhedronLoop3d outerLoop({
        SCPoint3d{0.0, 0.0, 0.0},
        SCPoint3d{2.0, 0.0, 0.0},
        SCPoint3d{2.0, 2.0, 0.0},
        SCPoint3d{0.0, 2.0, 0.0},
    });
    const PolyhedronFace3d holedFace(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                                     outerLoop,
                                     {PolyhedronLoop3d({
                                         SCPoint3d{0.5, 0.5, 0.0},
                                         SCPoint3d{1.5, 0.5, 0.0},
                                         SCPoint3d{1.5, 1.5, 0.0},
                                         SCPoint3d{0.5, 1.5, 0.0},
                                     })});
    const auto projectedFace = ProjectFaceToPolygon2d(holedFace);
    ASSERT_TRUE(projectedFace.success);
    ASSERT_TRUE(projectedFace.polygon.IsValid());
    ASSERT_EQ(projectedFace.polygon.HoleCount(), 1);
    EXPECT_NEAR(projectedFace.origin.z, 0.0, 1e-12);
    EXPECT_NEAR(Geometry::Dot(projectedFace.uAxis, projectedFace.vAxis), 0.0, 1e-12);

    const auto holedFaceMesh = ConvertToTriangleMesh(holedFace);
    ASSERT_TRUE(holedFaceMesh.success);
    ASSERT_EQ(holedFaceMesh.issue, MeshConversionIssue3d::None);
    ASSERT_TRUE(holedFaceMesh.mesh.IsValid());
    EXPECT_NEAR(holedFaceMesh.mesh.SurfaceArea(), projectedFace.polygon.Area(), 1e-12);
}
