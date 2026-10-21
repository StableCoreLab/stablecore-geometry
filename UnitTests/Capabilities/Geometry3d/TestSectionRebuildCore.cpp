#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using namespace Geometry;

TEST(GeometryTest, CoversSectionRebuildCapabilities)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());
    ASSERT_EQ(LocatePoint(SCPoint3d{0.5, 0.5, 0.5}, cubeBody), SCPointContainment3d::Inside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.5, 0.5, 0.5}, cubeBody), SCPointContainment3d::Outside);
    ASSERT_EQ(LocatePoint(SCPoint3d{1.0, 0.5, 0.5}, cubeBody), SCPointContainment3d::OnBoundary);
    const SCPolyhedronSection3d middleSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(middleSection.success);
    ASSERT_EQ(middleSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(middleSection.IsValid());
    const SCSectionValidation3d middleSectionValidation = Validate(middleSection);
    ASSERT_TRUE(middleSectionValidation.valid);
    ASSERT_EQ(middleSectionValidation.issue, SCSectionValidationIssue3d::None);
    const SCSectionTopology3d middleSectionTopology = BuildSectionTopology(middleSection);
    ASSERT_TRUE(middleSectionTopology.IsValid());
    ASSERT_EQ(middleSectionTopology.Count(), 1);
    ASSERT_EQ(middleSectionTopology.Roots().size(), 1);
    ASSERT_EQ(middleSectionTopology.Node(0).depth, 0);
    const SCSectionComponents3d middleSectionComponents = BuildSectionComponents(middleSection);
    ASSERT_TRUE(middleSectionComponents.IsValid());
    ASSERT_EQ(middleSectionComponents.components.size(), 1);
    ASSERT_EQ(middleSectionComponents.components[0].rootPolygonIndex, 0);
    ASSERT_EQ(middleSectionComponents.components[0].polygonIndices.size(), 1);
    ASSERT_EQ(middleSectionComponents.components[0].faceIndices.size(), 1);
    ASSERT_EQ(ClassifySectionContent(middleSection), SCSectionContentKind3d::Area);
    ASSERT_GE(middleSection.segments.size(), 4);
    ASSERT_EQ(middleSection.contours.size(), 1);
    ASSERT_TRUE(middleSection.contours[0].closed);
    ASSERT_EQ(middleSection.contours[0].points.size(), 4);
    ASSERT_EQ(middleSection.polygons.size(), 1);
    EXPECT_NEAR(middleSection.polygons[0].Area(), 1.0, 1e-12);
    EXPECT_NEAR(middleSection.origin.z, 0.5, 1e-12);
    const SCSectionFaceRebuildResult3d rebuiltMiddleFaces = RebuildSectionFaces(middleSection);
    ASSERT_TRUE(rebuiltMiddleFaces.success);
    ASSERT_EQ(rebuiltMiddleFaces.issue, SCSectionFaceRebuildIssue3d::None);
    ASSERT_TRUE(rebuiltMiddleFaces.IsValid());
    ASSERT_EQ(rebuiltMiddleFaces.faces.size(), 1);
    const SCSectionBrepFaceRebuildResult3d rebuiltMiddleBrepFaces = RebuildSectionBrepFaces(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.success);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepFaces.faces.size(), 1);
    ASSERT_TRUE(rebuiltMiddleBrepFaces.faces[0].OuterTrim().IsValid());
    const auto rebuiltMiddleMesh = ConvertToTriangleMesh(rebuiltMiddleFaces.faces[0]);
    ASSERT_TRUE(rebuiltMiddleMesh.success);
    EXPECT_NEAR(rebuiltMiddleMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SCSectionBodyRebuildResult3d rebuiltMiddleBody = RebuildSectionBody(middleSection);
    ASSERT_TRUE(rebuiltMiddleBody.success);
    ASSERT_EQ(rebuiltMiddleBody.issue, SCSectionBodyRebuildIssue3d::None);
    ASSERT_TRUE(rebuiltMiddleBody.IsValid());
    ASSERT_EQ(rebuiltMiddleBody.body.FaceCount(), 1);
    const SCSectionBrepBodyRebuildResult3d rebuiltMiddleBrepBody = RebuildSectionBrepBody(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepBody.success);
    ASSERT_TRUE(rebuiltMiddleBrepBody.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepBody.body.FaceCount(), 1);
    const SCSectionBrepBodySetRebuildResult3d rebuiltMiddleBrepBodies = RebuildSectionBrepBodies(middleSection);
    ASSERT_TRUE(rebuiltMiddleBrepBodies.success);
    ASSERT_TRUE(rebuiltMiddleBrepBodies.IsValid());
    ASSERT_EQ(rebuiltMiddleBrepBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMiddleBrepBodies.rootPolygonIndices.size(), 1);
    const SCSectionBodySetRebuildResult3d rebuiltMiddleBodies = RebuildSectionBodies(middleSection);
    ASSERT_TRUE(rebuiltMiddleBodies.success);
    ASSERT_TRUE(rebuiltMiddleBodies.IsValid());
    ASSERT_EQ(rebuiltMiddleBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMiddleBodies.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMiddleBodies.rootPolygonIndices[0], 0);
    const SCSectionMeshConversionResult3d middleSectionMesh = ConvertSectionToTriangleMesh(middleSection);
    ASSERT_TRUE(middleSectionMesh.success);
    ASSERT_TRUE(middleSectionMesh.IsValid());
    EXPECT_NEAR(middleSectionMesh.mesh.SurfaceArea(), 1.0, 1e-12);
    const SCSectionMeshSetConversionResult3d middleSectionMeshes = ConvertSectionToTriangleMeshes(middleSection);
    ASSERT_TRUE(middleSectionMeshes.success);
    ASSERT_TRUE(middleSectionMeshes.IsValid());
    ASSERT_EQ(middleSectionMeshes.meshes.size(), 1);
    ASSERT_EQ(middleSectionMeshes.rootPolygonIndices.size(), 1);
    ASSERT_EQ(middleSectionMeshes.rootPolygonIndices[0], 0);

    const SCPolyhedronSection3d disjointSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(disjointSection.success);
    ASSERT_EQ(disjointSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(disjointSection.segments.empty());
    ASSERT_TRUE(disjointSection.contours.empty());
    ASSERT_TRUE(disjointSection.polygons.empty());
    ASSERT_EQ(ClassifySectionContent(disjointSection), SCSectionContentKind3d::Empty);

    const SCPolyhedronSection3d coplanarSection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}));
    ASSERT_TRUE(coplanarSection.success);
    ASSERT_EQ(coplanarSection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(coplanarSection.IsValid());
    ASSERT_EQ(coplanarSection.contours.size(), 1);
    ASSERT_EQ(coplanarSection.polygons.size(), 1);
    EXPECT_NEAR(coplanarSection.polygons[0].Area(), 1.0, 1e-12);
    const SCSectionFaceRebuildResult3d rebuiltCoplanarFaces = RebuildSectionFaces(coplanarSection);
    ASSERT_TRUE(rebuiltCoplanarFaces.success);
    ASSERT_EQ(rebuiltCoplanarFaces.faces.size(), 1);

    const SCPolyhedronSection3d edgeOnlySection =
        Section(cubeBody, SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{1.0, 1.0, 0.0}));
    ASSERT_TRUE(edgeOnlySection.success);
    ASSERT_EQ(edgeOnlySection.issue, SCSectionIssue3d::None);
    ASSERT_TRUE(edgeOnlySection.IsValid());
    ASSERT_EQ(edgeOnlySection.segments.size(), 1);
    ASSERT_EQ(edgeOnlySection.contours.size(), 1);
    ASSERT_FALSE(edgeOnlySection.contours[0].closed);
    ASSERT_EQ(edgeOnlySection.contours[0].points.size(), 2);
    ASSERT_TRUE(edgeOnlySection.polygons.empty());
    const SCSectionTopology3d edgeOnlyTopology = BuildSectionTopology(edgeOnlySection);
    ASSERT_TRUE(edgeOnlyTopology.IsValid());
    ASSERT_TRUE(edgeOnlyTopology.IsEmpty());
    const SCSectionComponents3d edgeOnlyComponents = BuildSectionComponents(edgeOnlySection);
    ASSERT_TRUE(edgeOnlyComponents.IsValid());
    ASSERT_TRUE(edgeOnlyComponents.components.empty());
    ASSERT_EQ(ClassifySectionContent(edgeOnlySection), SCSectionContentKind3d::Curve);
    const SCSectionFaceRebuildResult3d rebuiltEdgeOnlyFaces = RebuildSectionFaces(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyFaces.success);
    ASSERT_TRUE(rebuiltEdgeOnlyFaces.faces.empty());
    const SCSectionBrepFaceRebuildResult3d rebuiltEdgeOnlyBrepFaces = RebuildSectionBrepFaces(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepFaces.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepFaces.faces.empty());
    const SCSectionBodyRebuildResult3d rebuiltEdgeOnlyBody = RebuildSectionBody(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBody.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBody.body.IsEmpty());
    const SCSectionBrepBodyRebuildResult3d rebuiltEdgeOnlyBrepBody = RebuildSectionBrepBody(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBody.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBody.body.IsEmpty());
    const SCSectionBrepBodySetRebuildResult3d rebuiltEdgeOnlyBrepBodies = RebuildSectionBrepBodies(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBodies.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBrepBodies.bodies.empty());
    const SCSectionBodySetRebuildResult3d rebuiltEdgeOnlyBodies = RebuildSectionBodies(edgeOnlySection);
    ASSERT_TRUE(rebuiltEdgeOnlyBodies.success);
    ASSERT_TRUE(rebuiltEdgeOnlyBodies.bodies.empty());
    const SCSectionMeshConversionResult3d edgeOnlySectionMesh = ConvertSectionToTriangleMesh(edgeOnlySection);
    ASSERT_TRUE(edgeOnlySectionMesh.success);
    ASSERT_TRUE(edgeOnlySectionMesh.mesh.IsEmpty());
    const SCSectionMeshSetConversionResult3d edgeOnlySectionMeshes = ConvertSectionToTriangleMeshes(edgeOnlySection);
    ASSERT_TRUE(edgeOnlySectionMeshes.success);
    ASSERT_TRUE(edgeOnlySectionMeshes.meshes.empty());

    SCPolyhedronSection3d nestedSection{};
    nestedSection.success = true;
    nestedSection.origin = SCPoint3d{0.0, 0.0, 0.5};
    nestedSection.uAxis = SCVector3d{1.0, 0.0, 0.0};
    nestedSection.vAxis = SCVector3d{0.0, 1.0, 0.0};
    nestedSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{0.0, 0.0},
            SCPoint2d{4.0, 0.0},
            SCPoint2d{4.0, 4.0},
            SCPoint2d{0.0, 4.0},
        },
        SCPolylineClosure::Closed)));
    nestedSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{1.0, 1.0},
            SCPoint2d{1.0, 3.0},
            SCPoint2d{3.0, 3.0},
            SCPoint2d{3.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    const SCSectionFaceRebuildResult3d rebuiltMergedFaces = RebuildSectionFaces(nestedSection);
    ASSERT_TRUE(rebuiltMergedFaces.success);
    ASSERT_TRUE(rebuiltMergedFaces.IsValid());
    ASSERT_EQ(rebuiltMergedFaces.faces.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].outerPolygonIndex, 0);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].holePolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedFaces.mappings[0].holePolygonIndices[0], 1);
    ASSERT_EQ(rebuiltMergedFaces.faces[0].HoleCount(), 1);
    const SCSectionTopology3d nestedTopology = BuildSectionTopology(nestedSection);
    ASSERT_TRUE(nestedTopology.IsValid());
    ASSERT_EQ(nestedTopology.Count(), 2);
    ASSERT_EQ(nestedTopology.Roots().size(), 1);
    ASSERT_EQ(nestedTopology.ParentOf(1), 0);
    ASSERT_EQ(nestedTopology.Node(0).depth, 0);
    ASSERT_EQ(nestedTopology.Node(1).depth, 1);
    const SCSectionComponents3d nestedComponents = BuildSectionComponents(nestedSection);
    ASSERT_TRUE(nestedComponents.IsValid());
    ASSERT_EQ(nestedComponents.components.size(), 1);
    ASSERT_EQ(nestedComponents.components[0].polygonIndices.size(), 2);
    ASSERT_EQ(nestedComponents.components[0].faceIndices.size(), 1);
    const auto rebuiltMergedMesh = ConvertToTriangleMesh(rebuiltMergedFaces.faces[0]);
    ASSERT_TRUE(rebuiltMergedMesh.success);
    EXPECT_NEAR(rebuiltMergedMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SCSectionBodyRebuildResult3d rebuiltMergedBody = RebuildSectionBody(nestedSection);
    ASSERT_TRUE(rebuiltMergedBody.success);
    ASSERT_EQ(rebuiltMergedBody.body.FaceCount(), 1);
    const SCSectionBrepFaceRebuildResult3d rebuiltMergedBrepFaces = RebuildSectionBrepFaces(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepFaces.success);
    ASSERT_EQ(rebuiltMergedBrepFaces.faces.size(), 1);
    ASSERT_EQ(rebuiltMergedBrepFaces.faces[0].HoleCount(), 1);
    const SCSectionBrepBodyRebuildResult3d rebuiltMergedBrepBody = RebuildSectionBrepBody(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepBody.success);
    ASSERT_EQ(rebuiltMergedBrepBody.body.FaceCount(), 1);
    const SCSectionBrepBodySetRebuildResult3d rebuiltMergedBrepBodies = RebuildSectionBrepBodies(nestedSection);
    ASSERT_TRUE(rebuiltMergedBrepBodies.success);
    ASSERT_EQ(rebuiltMergedBrepBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMergedBrepBodies.rootPolygonIndices.size(), 1);
    const SCSectionBodySetRebuildResult3d rebuiltMergedBodies = RebuildSectionBodies(nestedSection);
    ASSERT_TRUE(rebuiltMergedBodies.success);
    ASSERT_EQ(rebuiltMergedBodies.bodies.size(), 1);
    ASSERT_EQ(rebuiltMergedBodies.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedBodies.rootPolygonIndices[0], 0);
    const SCSectionMeshConversionResult3d rebuiltMergedSectionMesh = ConvertSectionToTriangleMesh(nestedSection);
    ASSERT_TRUE(rebuiltMergedSectionMesh.success);
    EXPECT_NEAR(rebuiltMergedSectionMesh.mesh.SurfaceArea(), 12.0, 1e-12);
    const SCSectionMeshSetConversionResult3d rebuiltMergedSectionMeshes = ConvertSectionToTriangleMeshes(nestedSection);
    ASSERT_TRUE(rebuiltMergedSectionMeshes.success);
    ASSERT_EQ(rebuiltMergedSectionMeshes.meshes.size(), 1);
    ASSERT_EQ(rebuiltMergedSectionMeshes.rootPolygonIndices.size(), 1);
    ASSERT_EQ(rebuiltMergedSectionMeshes.rootPolygonIndices[0], 0);
    ASSERT_EQ(ClassifySectionContent(nestedSection), SCSectionContentKind3d::Area);

    SCPolyhedronSection3d disjointAreaSection{};
    disjointAreaSection.success = true;
    disjointAreaSection.origin = SCPoint3d{0.0, 0.0, 0.5};
    disjointAreaSection.uAxis = SCVector3d{1.0, 0.0, 0.0};
    disjointAreaSection.vAxis = SCVector3d{0.0, 1.0, 0.0};
    disjointAreaSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{0.0, 0.0},
            SCPoint2d{1.0, 0.0},
            SCPoint2d{1.0, 1.0},
            SCPoint2d{0.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    disjointAreaSection.polygons.push_back(SCPolygon2d(SCPolyline2d(
        {
            SCPoint2d{3.0, 0.0},
            SCPoint2d{4.0, 0.0},
            SCPoint2d{4.0, 1.0},
            SCPoint2d{3.0, 1.0},
        },
        SCPolylineClosure::Closed)));
    const SCSectionTopology3d disjointAreaTopology = BuildSectionTopology(disjointAreaSection);
    ASSERT_TRUE(disjointAreaTopology.IsValid());
    ASSERT_EQ(disjointAreaTopology.Roots().size(), 2);
    const SCSectionComponents3d disjointAreaComponents = BuildSectionComponents(disjointAreaSection);
    ASSERT_TRUE(disjointAreaComponents.IsValid());
    ASSERT_EQ(disjointAreaComponents.components.size(), 2);
    ASSERT_EQ(disjointAreaComponents.components[0].polygonIndices.size(), 1);
    ASSERT_EQ(disjointAreaComponents.components[1].polygonIndices.size(), 1);
    const SCSectionBodySetRebuildResult3d rebuiltDisjointBodies = RebuildSectionBodies(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointBodies.success);
    ASSERT_TRUE(rebuiltDisjointBodies.IsValid());
    ASSERT_EQ(rebuiltDisjointBodies.bodies.size(), 2);
    ASSERT_EQ(rebuiltDisjointBodies.bodies[0].FaceCount(), 1);
    ASSERT_EQ(rebuiltDisjointBodies.bodies[1].FaceCount(), 1);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointBodies.rootPolygonIndices[1], 1);
    const SCSectionBrepBodySetRebuildResult3d rebuiltDisjointBrepBodies = RebuildSectionBrepBodies(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointBrepBodies.success);
    ASSERT_EQ(rebuiltDisjointBrepBodies.bodies.size(), 2);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointBrepBodies.rootPolygonIndices[1], 1);
    const SCSectionMeshSetConversionResult3d rebuiltDisjointMeshes = ConvertSectionToTriangleMeshes(disjointAreaSection);
    ASSERT_TRUE(rebuiltDisjointMeshes.success);
    ASSERT_TRUE(rebuiltDisjointMeshes.IsValid());
    ASSERT_EQ(rebuiltDisjointMeshes.meshes.size(), 2);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices.size(), 2);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices[0], 0);
    ASSERT_EQ(rebuiltDisjointMeshes.rootPolygonIndices[1], 1);

    SCPolyhedronSection3d mixedSection = edgeOnlySection;
    mixedSection.polygons.push_back(middleSection.polygons[0]);
    ASSERT_EQ(ClassifySectionContent(mixedSection), SCSectionContentKind3d::Mixed);

    SCPolyhedronSection3d invalidSection = middleSection;
    invalidSection.uAxis = SCVector3d{0.0, 0.0, 0.0};
    const SCSectionValidation3d invalidSectionValidation = Validate(invalidSection);
    ASSERT_FALSE(invalidSectionValidation.valid);
    ASSERT_EQ(invalidSectionValidation.issue, SCSectionValidationIssue3d::InvalidBasis);
}
