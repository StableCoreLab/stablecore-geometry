#include "Support/BodyBooleanFixtures3d.h"

TEST(BodyBoolean3dCapabilityTest, PublicCapabilitiesDescribeTheSupportedSubsets)
{
    const auto capabilities = Geometry::GetBodyBooleanCapabilities3d();
    EXPECT_TRUE(capabilities.equivalentClosedBodies);
    EXPECT_TRUE(capabilities.boundsDisjointClosedBodies);
    EXPECT_TRUE(capabilities.axisAlignedBoxIntersection);
    EXPECT_TRUE(capabilities.nonCoplanarConvexPolyhedralIntersection);
    EXPECT_TRUE(capabilities.axisAlignedBoxUnion);
    EXPECT_TRUE(capabilities.axisAlignedBoxDifferenceWhenSingleBoxOrEmpty);
    EXPECT_TRUE(capabilities.coplanarSupportPlaneOperations);
    EXPECT_FALSE(capabilities.nonPlanarSupportSurfaces);
    EXPECT_FALSE(capabilities.nonConvexBodies);
    EXPECT_FALSE(capabilities.multiShellBodies);
    EXPECT_FALSE(capabilities.curvedBoundaries);
    EXPECT_FALSE(capabilities.generalUnion);
    EXPECT_FALSE(capabilities.generalDifference);
    EXPECT_FALSE(capabilities.generalBrepCsg);
}

TEST(BodyBoolean3dCapabilityTest, InvalidInputContractIsStableForEmptyBodies)
{
    const PolyhedronBody first;
    const PolyhedronBody second;
    const BodyBooleanOptions3d options{};

    const auto intersection = IntersectBodies(first, second, options);
    const auto united = UnionBodies(first, second, options);
    const auto difference = DifferenceBodies(first, second, options);

    EXPECT_EQ(intersection.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(united.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::InvalidInput);
    EXPECT_FALSE(intersection.IsSuccess());
    EXPECT_FALSE(united.IsSuccess());
    EXPECT_FALSE(difference.IsSuccess());
}

TEST(BodyBoolean3dCapabilityTest, OpenBrepInputReturnsInvalidInputInsteadOfForcingClosedShell)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    const Geometry::SCBrepBody openBody(converted.body.Vertices(), converted.body.Edges(),
                                        {Geometry::SCBrepShell(shell.Faces(), false)});

    EXPECT_TRUE(openBody.IsValid());
    EXPECT_FALSE(openBody.ShellAt(0).IsClosed());
    ExpectInvalidInput(IntersectBodies(openBody, converted.body));
    ExpectInvalidInput(UnionBodies(openBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(openBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, NonManifoldBrepInputReturnsInvalidInputEvenWhenClosedFlagIsSet)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces = shell.Faces();
    faces.push_back(faces.front());
    const Geometry::SCBrepBody nonManifold(converted.body.Vertices(), converted.body.Edges(),
                                           {Geometry::SCBrepShell(std::move(faces), true)});

    EXPECT_TRUE(nonManifold.IsValid());
    EXPECT_TRUE(nonManifold.ShellAt(0).IsClosed());
    ExpectInvalidInput(IntersectBodies(nonManifold, converted.body));
    ExpectInvalidInput(UnionBodies(nonManifold, converted.body));
    ExpectInvalidInput(DifferenceBodies(nonManifold, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, ValidMultiShellBrepReturnsUnsupportedOperation)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    const Geometry::SCBrepBody multiShell(converted.body.Vertices(), converted.body.Edges(), {shell, shell});

    ASSERT_TRUE(multiShell.IsValid());
    ExpectUnsupportedOperation(IntersectBodies(multiShell, converted.body));
    ExpectUnsupportedOperation(UnionBodies(multiShell, converted.body));
    ExpectUnsupportedOperation(DifferenceBodies(multiShell, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, OperateOnShellsOptionReturnsUnsupportedOperation)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);

    BodyBooleanOptions3d options;
    options.operateOnShells = true;
    ExpectUnsupportedOperation(IntersectBodies(converted.body, converted.body, options));
    ExpectUnsupportedOperation(UnionBodies(converted.body, converted.body, options));
    ExpectUnsupportedOperation(DifferenceBodies(converted.body, converted.body, options));
}

TEST(BodyBoolean3dCapabilityTest, CurvedBrepEdgeReturnsUnsupportedInsteadOfUsingItsChord)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    const SCPoint3d start = converted.body.VertexAt(original.StartVertexIndex()).Point();
    const SCPoint3d end = converted.body.VertexAt(original.EndVertexIndex()).Point();
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCNurbsCurve3d>(
            1, std::vector<SCPoint3d>{start, end}, std::vector<double>{0.0, 0.0, 1.0, 1.0}),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody curvedBody(converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(curvedBody.IsValid());
    ExpectUnsupportedOperation(IntersectBodies(curvedBody, converted.body));
    ExpectUnsupportedOperation(UnionBodies(curvedBody, converted.body));
    ExpectUnsupportedOperation(DifferenceBodies(curvedBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, NonPlanarSupportSurfaceReturnsUnsupportedOperation)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces = shell.Faces();
    const Geometry::SCBrepFace original = faces.front();
    auto nonPlanarSurface = std::make_shared<Geometry::SCNurbsSurface>(
        1, 1, 2, 2,
        std::vector<SCPoint3d>{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 0.0, 0.0},
                               SCPoint3d{0.0, 1.0, 0.25}, SCPoint3d{1.0, 1.0, 0.0}},
        std::vector<double>{0.0, 0.0, 1.0, 1.0}, std::vector<double>{0.0, 0.0, 1.0, 1.0});
    faces.front() = Geometry::SCBrepFace(std::move(nonPlanarSurface), original.OuterLoop(), original.HoleLoops(),
                                         original.OuterTrim(), original.HoleTrims());
    const Geometry::SCBrepBody nonPlanarBody(converted.body.Vertices(), converted.body.Edges(),
                                             {Geometry::SCBrepShell(std::move(faces), true)});

    ASSERT_TRUE(nonPlanarBody.IsValid());
    ExpectUnsupportedOperation(IntersectBodies(nonPlanarBody, converted.body));
    ExpectUnsupportedOperation(UnionBodies(nonPlanarBody, converted.body));
    ExpectUnsupportedOperation(DifferenceBodies(nonPlanarBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, OpenCurvedBrepReturnsInvalidInputBeforeUnsupportedFeatureCheck)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    const SCPoint3d start = converted.body.VertexAt(original.StartVertexIndex()).Point();
    const SCPoint3d end = converted.body.VertexAt(original.EndVertexIndex()).Point();
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCNurbsCurve3d>(
            1, std::vector<SCPoint3d>{start, end}, std::vector<double>{0.0, 0.0, 1.0, 1.0}),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepShell openShell(converted.body.ShellAt(0).Faces(), false);
    const Geometry::SCBrepBody openCurvedBody(converted.body.Vertices(), std::move(edges), {openShell});

    ASSERT_TRUE(openCurvedBody.IsValid());
    ExpectInvalidInput(IntersectBodies(openCurvedBody, converted.body));
    ExpectInvalidInput(UnionBodies(openCurvedBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(openCurvedBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, EdgeCurveEndpointMismatchReturnsInvalidInput)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCLineCurve3d>(Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection(SCPoint3d{10.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
            Geometry::SCIntervald{0.0, 1.0})),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody inconsistentBody(converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(inconsistentBody.IsValid());
    ExpectInvalidInput(IntersectBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(UnionBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(inconsistentBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, ExplicitZeroDistanceToleranceRemainsStrictAndValid)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    std::vector<Geometry::SCBrepEdge> edges = converted.body.Edges();
    const Geometry::SCBrepEdge original = edges.front();
    const SCPoint3d start = converted.body.VertexAt(original.StartVertexIndex()).Point();
    const SCPoint3d end = converted.body.VertexAt(original.EndVertexIndex()).Point();
    const SCPoint3d shiftedStart{start.x + 0.5 * Geometry::kDefaultEpsilon, start.y, start.z};
    edges.front() = Geometry::SCBrepEdge(
        std::make_shared<Geometry::SCLineCurve3d>(Geometry::SCLineCurve3d::FromLine(
            Geometry::SCLine3d::FromOriginAndDirection(shiftedStart, end - shiftedStart), Geometry::SCIntervald{0.0, 1.0})),
        original.StartVertexIndex(), original.EndVertexIndex());
    const Geometry::SCBrepBody nearlyConsistentBody(
        converted.body.Vertices(), std::move(edges), converted.body.Shells());

    ASSERT_TRUE(nearlyConsistentBody.IsValid());
    EXPECT_EQ(IntersectBodies(nearlyConsistentBody, converted.body).issue, BodyBooleanIssue3d::None);
    BodyBooleanOptions3d strictOptions;
    strictOptions.tolerance.distanceEpsilon = 0.0;
    ExpectInvalidInput(IntersectBodies(nearlyConsistentBody, converted.body, strictOptions));
}

TEST(BodyBoolean3dCapabilityTest, PlaneSupportMismatchReturnsInvalidInput)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces = shell.Faces();
    const Geometry::SCBrepFace original = faces.front();
    const auto* originalSurface = dynamic_cast<const Geometry::SCPlaneSurface*>(original.SupportSurface());
    ASSERT_NE(originalSurface, nullptr);
    SCPlane shiftedPlane = originalSurface->SupportPlane();
    shiftedPlane.origin = shiftedPlane.origin + shiftedPlane.UnitNormal() * 0.25;
    faces.front() = Geometry::SCBrepFace(std::make_shared<Geometry::SCPlaneSurface>(
                                              Geometry::SCPlaneSurface::FromPlane(shiftedPlane)),
                                          original.OuterLoop(), original.HoleLoops(), original.OuterTrim(),
                                          original.HoleTrims());
    const Geometry::SCBrepBody inconsistentBody(converted.body.Vertices(), converted.body.Edges(),
                                                {Geometry::SCBrepShell(std::move(faces), true)});

    ASSERT_TRUE(inconsistentBody.IsValid());
    ExpectInvalidInput(IntersectBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(UnionBodies(inconsistentBody, converted.body));
    ExpectInvalidInput(DifferenceBodies(inconsistentBody, converted.body));
}

TEST(BodyBoolean3dCapabilityTest, EquivalentPlaneFacesIgnoreParameterDomainBounds)
{
    const auto converted = ConvertToBrepBody(Geometry::Test::BuildUnitCubeBody());
    ASSERT_TRUE(converted.success);
    const Geometry::SCBrepShell shell = converted.body.ShellAt(0);
    std::vector<Geometry::SCBrepFace> faces;
    faces.reserve(shell.FaceCount());
    for (const Geometry::SCBrepFace& original : shell.Faces())
    {
        const auto* originalSurface = dynamic_cast<const Geometry::SCPlaneSurface*>(original.SupportSurface());
        ASSERT_NE(originalSurface, nullptr);
        faces.emplace_back(std::make_shared<Geometry::SCPlaneSurface>(Geometry::SCPlaneSurface::FromPlane(
                               originalSurface->SupportPlane(), Geometry::SCIntervald{-10.0, 10.0},
                               Geometry::SCIntervald{-10.0, 10.0})),
                           original.OuterLoop(), original.HoleLoops());
    }
    const Geometry::SCBrepBody reparameterizedBody(converted.body.Vertices(), converted.body.Edges(),
                                                   {Geometry::SCBrepShell(std::move(faces), true)});

    ASSERT_TRUE(reparameterizedBody.IsValid());
    EXPECT_FALSE(reparameterizedBody.Bounds().MinPoint().AlmostEquals(converted.body.Bounds().MinPoint(), 1e-9));
    EXPECT_EQ(IntersectBodies(converted.body, reparameterizedBody).issue, BodyBooleanIssue3d::None);
    EXPECT_EQ(UnionBodies(converted.body, reparameterizedBody).issue, BodyBooleanIssue3d::None);
    const auto difference = DifferenceBodies(converted.body, reparameterizedBody);
    EXPECT_EQ(difference.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(difference.producedEmptyResult);
}

TEST(BodyBoolean3dCapabilityTest, AngularToleranceControlsNearCoplanarSupportClassification)
{
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedPrismaticBox(0.0, 0.0, 0.2, 1.0, 1.0, 1.2, 5e-10);
    BodyBooleanOptions3d defaultAngleOptions;
    defaultAngleOptions.tolerance.distanceEpsilon = 1e-12;
    const auto defaultAngleResult = IntersectBodies(first, second, defaultAngleOptions);

    BodyBooleanOptions3d strictAngleOptions = defaultAngleOptions;
    strictAngleOptions.tolerance.angleEpsilon = 0.0;
    const auto strictAngleResult = IntersectBodies(first, second, strictAngleOptions);

    ExpectUnsupportedOperation(defaultAngleResult);
    EXPECT_EQ(strictAngleResult.issue, BodyBooleanIssue3d::None);
    EXPECT_TRUE(strictAngleResult.IsSuccess());
}

TEST(BodyBoolean3dCapabilityTest, AngularToleranceComparesRadiansForSlantedSupportPlanes)
{
    constexpr double angle = 0.5;
    const double centerX = 0.5 * (std::cos(angle) - std::sin(angle));
    const double minX = centerX - 0.5;
    const double maxX = centerX + 0.5;
    const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
    const PolyhedronBody second = BuildRotatedPrismaticBox(minX, 0.0, 0.2, maxX, 1.0, 1.2, angle);

    BodyBooleanOptions3d options;
    options.tolerance.angleEpsilon = 0.49;
    const auto result = IntersectBodies(first, second, options);
    ASSERT_EQ(result.issue, BodyBooleanIssue3d::None);
    ASSERT_TRUE(result.IsSuccess());
    EXPECT_NEAR(Geometry::Volume(result.body),
                RotatedPrismaticBoxIntersectionVolumeOracle(minX, 0.0, 0.2, maxX, 1.0, 1.2, angle), 1e-8);
}

TEST(BodyBoolean3dCapabilityTest, InvalidToleranceReturnsInvalidInput)
{
    BodyBooleanOptions3d options;
    options.tolerance.angleEpsilon = -1.0;

    ExpectInvalidInput(IntersectBodies(Geometry::Test::BuildUnitCubeBody(), Geometry::Test::BuildUnitCubeBody(), options));
}
