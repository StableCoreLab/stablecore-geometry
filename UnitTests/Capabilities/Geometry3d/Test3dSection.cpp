
#include <cmath>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

using Geometry::BrepConversionIssue3d;
using Geometry::BuildSectionComponents;
using Geometry::BuildSectionTopology;
using Geometry::ClassifySectionContent;
using Geometry::ConvertToBrepBody;
using Geometry::SCPlane;
using Geometry::SCPoint3d;
using Geometry::PolyhedronBody;
using Geometry::PolyhedronFace3d;
using Geometry::PolyhedronLoop3d;
using Geometry::Section;
using SectionContentKind3d = Geometry::SCSectionContentKind3d;
using Geometry::SCVector3d;

namespace
{
    PolyhedronBody BuildAdjacentCoplanarFaceBody()
    {
        return PolyhedronBody(
            {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                              PolyhedronLoop3d({
                                  SCPoint3d{0.0, 0.0, 0.0},
                                  SCPoint3d{1.0, 0.0, 0.0},
                                  SCPoint3d{1.0, 1.0, 0.0},
                                  SCPoint3d{0.0, 1.0, 0.0},
                              })),
             PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                              PolyhedronLoop3d({
                                  SCPoint3d{1.0, 0.0, 0.0},
                                  SCPoint3d{2.0, 0.0, 0.0},
                                  SCPoint3d{2.0, 1.0, 0.0},
                                  SCPoint3d{1.0, 1.0, 0.0},
                              }))});
    }

    PolyhedronBody BuildThreeCoplanarStripFacesBody()
    {
        return PolyhedronBody(
            {PolyhedronFace3d(
                 SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                 PolyhedronLoop3d(
                     {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 0.0, 0.0}, SCPoint3d{1.0, 1.0, 0.0}, SCPoint3d{0.0, 1.0, 0.0}})),
             PolyhedronFace3d(
                 SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                 PolyhedronLoop3d(
                     {SCPoint3d{1.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 1.0, 0.0}, SCPoint3d{1.0, 1.0, 0.0}})),
             PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                              PolyhedronLoop3d({SCPoint3d{2.0, 0.0, 0.0},
                                                SCPoint3d{3.0, 0.0, 0.0},
                                                SCPoint3d{3.0, 1.0, 0.0},
                                                SCPoint3d{2.0, 1.0, 0.0}}))});
    }

    PolyhedronBody BuildCoplanarFrameFacesBody()
    {
        return PolyhedronBody(
            {PolyhedronFace3d(
                 SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                 PolyhedronLoop3d(
                     {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{3.0, 0.0, 0.0}, SCPoint3d{3.0, 1.0, 0.0}, SCPoint3d{0.0, 1.0, 0.0}})),
             PolyhedronFace3d(
                 SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                 PolyhedronLoop3d(
                     {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{3.0, 2.0, 0.0}, SCPoint3d{3.0, 3.0, 0.0}, SCPoint3d{0.0, 3.0, 0.0}})),
             PolyhedronFace3d(
                 SCPlane::FromPointAndNormal(SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                 PolyhedronLoop3d(
                     {SCPoint3d{0.0, 1.0, 0.0}, SCPoint3d{1.0, 1.0, 0.0}, SCPoint3d{1.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}})),
             PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 1.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                              PolyhedronLoop3d({SCPoint3d{2.0, 1.0, 0.0},
                                                SCPoint3d{3.0, 1.0, 0.0},
                                                SCPoint3d{3.0, 2.0, 0.0},
                                                SCPoint3d{2.0, 2.0, 0.0}}))});
    }

    PolyhedronLoop3d TranslateLoop(const PolyhedronLoop3d& loop, const SCVector3d& delta)
    {
        std::vector<SCPoint3d> vertices;
        vertices.reserve(loop.VertexCount());
        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
        {
            vertices.push_back(loop.VertexAt(i) + delta);
        }
        return PolyhedronLoop3d(std::move(vertices));
    }

    PolyhedronFace3d TranslateFace(const PolyhedronFace3d& face, const SCVector3d& delta)
    {
        const SCPlane translatedPlane =
            SCPlane::FromPointAndNormal(face.SupportPlane().origin + delta, face.SupportPlane().normal);

        PolyhedronLoop3d outer = TranslateLoop(face.OuterLoop(), delta);
        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            holes.push_back(TranslateLoop(face.HoleAt(i), delta));
        }

        return PolyhedronFace3d(translatedPlane, std::move(outer), std::move(holes));
    }

    PolyhedronBody BuildTwoSeparatedUnitCubesBody()
    {
        const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = first.Faces();

        const SCVector3d delta{3.0, 0.0, 0.0};
        for (const PolyhedronFace3d& face : first.Faces())
        {
            faces.push_back(TranslateFace(face, delta));
        }

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarFrameAndIntersectingCubeBody()
    {
        const PolyhedronBody frame = BuildCoplanarFrameFacesBody();
        std::vector<PolyhedronFace3d> faces = frame.Faces();

        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        const SCVector3d delta{5.0, 0.0, -0.5};
        for (const PolyhedronFace3d& face : cube.Faces())
        {
            faces.push_back(TranslateFace(face, delta));
        }

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedAreaAndOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 0.0, 0.0},
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                               SCPoint3d{3.0, 0.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildVertexAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 1.0, 0.0},
                               SCPoint3d{1.0, 2.0, 0.0},
                               SCPoint3d{1.0, 2.0, 1.0},
                               SCPoint3d{1.0, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.5, 1.0, 0.0},
                               SCPoint3d{0.5, 2.0, 0.0},
                               SCPoint3d{0.5, 2.0, 1.0},
                               SCPoint3d{0.5, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarAdjacentAndNonPlanarSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripAndNonPlanarSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.5, 1.0, 0.0},
                               SCPoint3d{0.5, 2.0, 0.0},
                               SCPoint3d{0.5, 2.0, 1.0},
                               SCPoint3d{0.5, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripAndVertexAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 1.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripAndDetachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{5.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{5.0, 0.0, 0.0},
                               SCPoint3d{5.0, 1.0, 0.0},
                               SCPoint3d{5.0, 1.0, 1.0},
                               SCPoint3d{5.0, 0.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripWithVertexAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 1.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{3.0, 0.0, 0.5},
                               SCPoint3d{3.0, 1.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 0.0},
                               SCPoint3d{4.0, 1.0, 1.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{5.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{5.0, 0.0, 0.0},
                               SCPoint3d{5.0, 1.0, 0.0},
                               SCPoint3d{5.0, 1.0, 1.0},
                               SCPoint3d{5.0, 0.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildTwoEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.75, -1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.75, -1.0, 0.0},
                               SCPoint3d{0.75, 0.0, 0.0},
                               SCPoint3d{0.75, 0.0, 1.0},
                               SCPoint3d{0.75, -1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildVertexAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 1.0, 0.0},
                               SCPoint3d{2.0, 1.0, 0.0},
                               SCPoint3d{2.0, 1.0, 1.0},
                               SCPoint3d{1.0, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildDetachedVertexAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 0.0, 0.0},
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                               SCPoint3d{3.0, 0.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 1.0, 0.0},
                               SCPoint3d{2.0, 1.0, 0.0},
                               SCPoint3d{2.0, 1.0, 1.0},
                               SCPoint3d{1.0, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildDetachedLeftAndEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{-2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{-2.0, 0.0, 0.0},
                               SCPoint3d{-2.0, 1.0, 0.0},
                               SCPoint3d{-2.0, 1.0, 1.0},
                               SCPoint3d{-2.0, 0.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildDetachedAndTwoEdgeAttachedOpenSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.0, 0.0, 0.0},
                               SCPoint3d{3.0, 1.0, 0.0},
                               SCPoint3d{3.0, 1.0, 1.0},
                               SCPoint3d{3.0, 0.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.25, 1.0, 0.0},
                               SCPoint3d{0.25, 2.0, 0.0},
                               SCPoint3d{0.25, 2.0, 1.0},
                               SCPoint3d{0.25, 1.0, 1.0},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.75, -1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.75, -1.0, 0.0},
                               SCPoint3d{0.75, 0.0, 0.0},
                               SCPoint3d{0.75, 0.0, 1.0},
                               SCPoint3d{0.75, -1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildNonPlanarLoopWithInteriorOpenSpurSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 1.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.5, 1.0, 0.0},
                               SCPoint3d{0.5, 0.25, 0.0},
                               SCPoint3d{0.5, 0.25, 1.0},
                               SCPoint3d{0.5, 1.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildLCornerCoplanarPatchAndNonPlanarSectionBody()
    {
        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        std::vector<PolyhedronFace3d> faces = cube.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{1.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{1.0, 0.0, 0.5},
                               SCPoint3d{2.0, 0.0, 0.5},
                               SCPoint3d{2.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                           }));

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 1.0, 0.5}, SCVector3d{0.0, 0.0, 1.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{0.0, 1.0, 0.5},
                               SCPoint3d{1.0, 1.0, 0.5},
                               SCPoint3d{1.0, 2.0, 0.5},
                               SCPoint3d{0.0, 2.0, 0.5},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody()
    {
        const PolyhedronBody frame = BuildCoplanarFrameFacesBody();
        std::vector<PolyhedronFace3d> faces;
        faces.reserve(frame.FaceCount() + 6);

        const SCVector3d frameDelta{0.0, 0.0, 0.5};
        for (const PolyhedronFace3d& face : frame.Faces())
        {
            faces.push_back(TranslateFace(face, frameDelta));
        }

        const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
        const SCVector3d cubeDelta{3.0, 1.0, 0.0};
        for (const PolyhedronFace3d& face : cube.Faces())
        {
            faces.push_back(TranslateFace(face, cubeDelta));
        }

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedMergedHoleAndAdjacentNonPlanarWithDetachedOpenSectionBody()
    {
        PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
        std::vector<PolyhedronFace3d> faces = body.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{5.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{5.0, 0.0, 0.0},
                               SCPoint3d{5.0, 1.0, 0.0},
                               SCPoint3d{5.0, 1.0, 1.0},
                               SCPoint3d{5.0, 0.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedMergedHoleAndAdjacentNonPlanarWithEdgeAttachedOpenSectionBody()
    {
        PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
        std::vector<PolyhedronFace3d> faces = body.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{3.5, 2.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{3.5, 2.0, 0.0},
                               SCPoint3d{3.5, 3.0, 0.0},
                               SCPoint3d{3.5, 3.0, 1.0},
                               SCPoint3d{3.5, 2.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }

    PolyhedronBody BuildMixedMergedHoleAndAdjacentNonPlanarWithVertexAttachedOpenSectionBody()
    {
        PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
        std::vector<PolyhedronFace3d> faces = body.Faces();

        faces.emplace_back(SCPlane::FromPointAndNormal(SCPoint3d{4.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
                           PolyhedronLoop3d({
                               SCPoint3d{4.0, 2.0, 0.0},
                               SCPoint3d{5.0, 2.0, 0.0},
                               SCPoint3d{5.0, 2.0, 1.0},
                               SCPoint3d{4.0, 2.0, 1.0},
                           }));

        return PolyhedronBody(std::move(faces));
    }
}  // namespace

TEST(Section3dCapabilityTest, SlantedCubeSectionBuildsSingleAreaComponent)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const SCPlane slantedCut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{1.0, 0.0, 1.0});
    const auto section = Section(cubeBody, slantedCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_FALSE(section.segments.empty());
    ASSERT_FALSE(section.polygons.empty());

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Count(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components.front().polygonIndices.size(), 1);
    ASSERT_EQ(components.components.front().faceIndices.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates non-axis-aligned multi-face sectioning remains stable on a
// generic cube cut: contour/polygon counts stay deterministic.
TEST(Section3dCapabilityTest, NonAxisAlignedCubeSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 1.0, 1.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    // For x+y+z=1.5 slicing a unit cube, the expected closed contour has 6
    // corners.
    ASSERT_EQ(section.contours[0].points.size(), 6);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates Section(SCBrepBody, SCPlane) remains stable on an oblique cut:
// after Polyhedron->Brep conversion, sectioning still yields one closed area
// contour with deterministic vertex count.
TEST(Section3dCapabilityTest, BrepBodyObliqueSectionHasSingleHexLikeContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 1.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates coplanar fragment merge also holds for Section(SCBrepBody,
// SCPlane): after Polyhedron->Brep conversion, adjacent coplanar faces are still
// merged into one area polygon instead of staying split.
TEST(Section3dCapabilityTest, BrepBodyAdjacentCoplanarFacesMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildAdjacentCoplanarFaceBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 2);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);
}

TEST(Section3dCapabilityTest, TwoDisjointNonPlanarLoopsStaySeparatedInSameSectionPlane)
{
    const PolyhedronBody body = BuildTwoSeparatedUnitCubesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 12);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        totalArea += polygon.Area();
    }
    ASSERT_LT(std::abs(totalArea - 2.0), 1e-12);
}

// Demonstrates Section(SCBrepBody, SCPlane) also supports multi-component area
// output: two separated cubes sectioned at z=0.5 produce two independent
// section polygons/components.
TEST(Section3dCapabilityTest, BrepBodySectionBuildsTwoAreaComponents)
{
    const PolyhedronBody polyBody = BuildTwoSeparatedUnitCubesBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 12);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates coplanar adjacent face fragments are merged into one area
// polygon instead of remaining as two disjoint coplanar pieces.
TEST(Section3dCapabilityTest, AdjacentCoplanarFacesMergeIntoSingleSectionPolygon)
{
    const PolyhedronBody body = BuildAdjacentCoplanarFaceBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 2);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components[0].faceIndices.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);
}

// Demonstrates that the face-merge subset extends beyond two adjacent faces:
// three coplanar faces arranged in an L-strip (each pair sharing an edge)
// are all merged into a single section polygon at the coplanar cut.
// Validates that coplanar fragment merge works for chains of 3+ faces, not
// just a single adjacent pair.
TEST(Section3dCapabilityTest, ThreeCoplanarFacesInLStripMergeIntoSinglePolygon)
{
    const PolyhedronBody body = BuildThreeCoplanarStripFacesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 3);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    // Expected: 3 coplanar adjacent faces merge into 1 polygon (area = 3.0 x 1)
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    // Rectangle 3猫锟? 茅藛?4 corners
    ASSERT_EQ(section.contours[0].points.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(components.components[0].faceIndices.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
}

// Demonstrates that the same three-face coplanar strip merge semantics are
// preserved on the Brep path: Polyhedron->Brep conversion does not fragment
// the section result into multiple polygons.
TEST(Section3dCapabilityTest, BrepThreeCoplanarFacesInStripMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildThreeCoplanarStripFacesBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 3);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
}

// Demonstrates higher-order coplanar fragment merge semantics beyond strips:
// four coplanar faces arranged as a rectangular frame merge into one polygon
// with one rectangular hole instead of remaining split into four fragments.
TEST(Section3dCapabilityTest, CoplanarFrameFacesMergeIntoSinglePolygonWithHole)
{
    const PolyhedronBody body = BuildCoplanarFrameFacesBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 4);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.segments.size(), 9);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 8.0), 1e-12);
}

// Demonstrates mixed section semantics: a coplanar frame component and a
// separate non-planar cube mid-section can coexist in one result instead of
// the coplanar component short-circuiting the non-planar contour stitching.
TEST(Section3dCapabilityTest, MixedCoplanarAndNonPlanarSectionBuildsTwoAreaComponents)
{
    const PolyhedronBody body = BuildMixedCoplanarFrameAndIntersectingCubeBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 10);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 12);

    std::size_t polygonsWithHole = 0;
    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        if (polygon.HoleCount() == 1)
        {
            ++polygonsWithHole;
        }
        totalArea += polygon.Area();
    }
    ASSERT_EQ(polygonsWithHole, 1);
    ASSERT_LT(std::abs(totalArea - 9.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates the same mixed coplanar + non-planar section subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepMixedCoplanarAndNonPlanarSectionBuildsTwoAreaComponents)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarFrameAndIntersectingCubeBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 10);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 2);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 12);

    std::size_t polygonsWithHole = 0;
    double totalArea = 0.0;
    for (const auto& polygon : section.polygons)
    {
        if (polygon.HoleCount() == 1)
        {
            ++polygonsWithHole;
        }
        totalArea += polygon.Area();
    }
    ASSERT_EQ(polygonsWithHole, 1);
    ASSERT_LT(std::abs(totalArea - 9.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 2);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 2);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates mixed section arbitration now keeps a representative detached
// open contour alongside a closed area component instead of forcing the result
// into area-only output.
TEST(Section3dCapabilityTest, MixedAreaAndOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedAreaAndOpenSectionBody();
    ASSERT_TRUE(body.IsValid());
    ASSERT_EQ(body.FaceCount(), 7);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates the same representative mixed area/open arbitration also holds
// on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepMixedAreaAndOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedAreaAndOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());
    ASSERT_EQ(polyBody.FaceCount(), 7);

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates adjacency arbitration now keeps a representative open contour
// even when it touches the closed area at a single polygon vertex.
TEST(Section3dCapabilityTest, VertexAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildVertexAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates the same vertex-attached mixed area/open arbitration also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepVertexAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildVertexAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates adjacency arbitration also keeps a representative open contour
// when it touches the closed area at the midpoint of a polygon edge.
TEST(Section3dCapabilityTest, EdgeAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody body = BuildEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepEdgeAttachedOpenContourSectionBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates final section-area merge now also reconciles edge-adjacent
// coplanar and non-planar polygons into one deterministic area result.
TEST(Section3dCapabilityTest, MixedCoplanarAdjacentAndNonPlanarSectionMergesIntoSinglePolygon)
{
    const PolyhedronBody body = BuildMixedCoplanarAdjacentAndNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarAdjacentAndNonPlanarSectionMergesIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarAdjacentAndNonPlanarSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates mixed coplanar/non-planar adjacency merge also remains stable
// when the coplanar side is a two-face strip, not just a single edge-adjacent
// face: cube mid-section + adjacent strip collapses into one 3x1 polygon.
TEST(Section3dCapabilityTest, MixedCoplanarStripAndNonPlanarSectionMergesIntoSinglePolygon)
{
    const PolyhedronBody body = BuildMixedCoplanarStripAndNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripAndNonPlanarSectionMergesIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripAndNonPlanarSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates mixed-content arbitration still survives the broader strip
// merge subset: after the area side expands from one face to a strip, the
// edge-attached open contour is still preserved as Mixed content.
TEST(Section3dCapabilityTest, MixedCoplanarStripAndEdgeAttachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripAndEdgeAttachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 5);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 0.5), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 2.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedCoplanarStripAndVertexAttachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripAndVertexAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 3.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 4.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 1.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripAndVertexAttachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripAndVertexAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 3.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.front().y - 1.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 4.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().y - 1.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedCoplanarStripAndDetachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripAndDetachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 5.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 5.0), 1e-12);
            const double endpointSum = contour.points.front().y + contour.points.back().y;
            const double endpointDelta = std::abs(contour.points.front().y - contour.points.back().y);
            ASSERT_LT(std::abs(endpointSum - 1.0), 1e-12);
            ASSERT_LT(std::abs(endpointDelta - 1.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripAndDetachedOpenContourBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripAndDetachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
        } else
        {
            ++openContours;
            ASSERT_EQ(contour.points.size(), 2);
            ASSERT_LT(std::abs(contour.points.front().x - 5.0), 1e-12);
            ASSERT_LT(std::abs(contour.points.back().x - 5.0), 1e-12);
            const double endpointSum = contour.points.front().y + contour.points.back().y;
            const double endpointDelta = std::abs(contour.points.front().y - contour.points.back().y);
            ASSERT_LT(std::abs(endpointSum - 1.0), 1e-12);
            ASSERT_LT(std::abs(endpointDelta - 1.0), 1e-12);
        }
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedCoplanarStripWithVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripWithVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    bool sawEdgeAttached = false;
    bool sawVertexAttached = false;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
            continue;
        }

        ++openContours;
        ASSERT_EQ(contour.points.size(), 2);
        if (std::abs(contour.points.front().x - 0.25) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 0.25) < 1e-12 && std::abs(contour.points.back().y - 2.0) < 1e-12)
        {
            sawEdgeAttached = true;
            continue;
        }

        if (std::abs(contour.points.front().x - 3.0) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 4.0) < 1e-12 && std::abs(contour.points.back().y - 1.0) < 1e-12)
        {
            sawVertexAttached = true;
            continue;
        }

        ASSERT_TRUE(false);
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 2);
    ASSERT_TRUE(sawEdgeAttached);
    ASSERT_TRUE(sawVertexAttached);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripWithVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripWithVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);

    std::size_t closedContours = 0;
    std::size_t openContours = 0;
    bool sawEdgeAttached = false;
    bool sawVertexAttached = false;
    for (const auto& contour : section.contours)
    {
        if (contour.closed)
        {
            ++closedContours;
            ASSERT_EQ(contour.points.size(), 4);
            continue;
        }

        ++openContours;
        ASSERT_EQ(contour.points.size(), 2);
        if (std::abs(contour.points.front().x - 0.25) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 0.25) < 1e-12 && std::abs(contour.points.back().y - 2.0) < 1e-12)
        {
            sawEdgeAttached = true;
            continue;
        }

        if (std::abs(contour.points.front().x - 3.0) < 1e-12 && std::abs(contour.points.front().y - 1.0) < 1e-12 &&
            std::abs(contour.points.back().x - 4.0) < 1e-12 && std::abs(contour.points.back().y - 1.0) < 1e-12)
        {
            sawVertexAttached = true;
            continue;
        }

        ASSERT_TRUE(false);
    }

    ASSERT_EQ(closedContours, 1);
    ASSERT_EQ(openContours, 2);
    ASSERT_TRUE(sawEdgeAttached);
    ASSERT_TRUE(sawVertexAttached);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody body = BuildMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenContoursBuildsMixedContent)
{
    const PolyhedronBody polyBody = BuildMixedCoplanarStripWithDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates mixed open-curve/area arbitration also keeps two representative
// edge-attached open contours on the same area result, with stable endpoint
// direction/order.
TEST(Section3dCapabilityTest, TwoEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 9);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepTwoEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, VertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, VertexTouchThenEdgeTouchOpenContoursDoNotCollapseIntoSinglePolyline)
{
    const PolyhedronBody body = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepVertexTouchThenEdgeTouchOpenContoursDoNotCollapseIntoSinglePolyline)
{
    const PolyhedronBody polyBody = BuildVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 7);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, NonPlanarLoopWithInteriorOpenSpurKeepsClosedContourAndOpenContourSeparate)
{
    const PolyhedronBody body = BuildNonPlanarLoopWithInteriorOpenSpurSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 2);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 0.25), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepNonPlanarLoopWithInteriorOpenSpurKeepsClosedContourAndOpenContourSeparate)
{
    const PolyhedronBody polyBody = BuildNonPlanarLoopWithInteriorOpenSpurSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 5);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 2);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 0.25), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, LCornerCoplanarPatchAndNonPlanarAreaMergeIntoSinglePolygon)
{
    const PolyhedronBody body = BuildLCornerCoplanarPatchAndNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepLCornerCoplanarPatchAndNonPlanarAreaMergeIntoSinglePolygon)
{
    const PolyhedronBody polyBody = BuildLCornerCoplanarPatchAndNonPlanarSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 6);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 3.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleStaysSinglePolygonWithHole)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 12);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleStaysSinglePolygonWithHole)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 2);
    ASSERT_EQ(section.segments.size(), 12);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndDetachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithDetachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndDetachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithDetachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndEdgeAttachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndEdgeAttachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 3.5), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, MixedMergedAreaWithInteriorHoleAndVertexAttachedOpenContourStaysMixed)
{
    const PolyhedronBody body = BuildMixedMergedHoleAndAdjacentNonPlanarWithVertexAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepMixedMergedAreaWithInteriorHoleAndVertexAttachedOpenContourStaysMixed)
{
    const PolyhedronBody polyBody = BuildMixedMergedHoleAndAdjacentNonPlanarWithVertexAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.polygons[0].HoleCount(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 13);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 8);
    ASSERT_TRUE(section.contours[1].closed);
    ASSERT_EQ(section.contours[1].points.size(), 4);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_EQ(section.contours[2].points.size(), 2);
    ASSERT_LT(std::abs(section.contours[2].points.front().x - 4.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 5.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 9.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, DetachedVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody body = BuildDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepDetachedVertexAndEdgeAttachedOpenContoursBuildStableMixedContent)
{
    const PolyhedronBody polyBody = BuildDetachedVertexAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates mixed open-curve/area arbitration now sorts boundary-attached
// open contours ahead of detached ones, even when the detached contour is
// lexicographically earlier in world coordinates.
TEST(Section3dCapabilityTest, BoundaryAttachedOpenContourSortsAheadOfDetachedContour)
{
    const PolyhedronBody body = BuildDetachedLeftAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepBoundaryAttachedOpenContourSortsAheadOfDetachedContour)
{
    const PolyhedronBody polyBody = BuildDetachedLeftAndEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 3);
    ASSERT_EQ(section.segments.size(), 6);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x + 2.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, DetachedPlusTwoEdgeAttachedContoursRemainSeparateFromMergedArea)
{
    const PolyhedronBody body = BuildDetachedAndTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

TEST(Section3dCapabilityTest, BrepDetachedPlusTwoEdgeAttachedContoursRemainSeparateFromMergedArea)
{
    const PolyhedronBody polyBody = BuildDetachedAndTwoEdgeAttachedOpenSectionBody();
    ASSERT_TRUE(polyBody.IsValid());

    const auto converted = ConvertToBrepBody(polyBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_EQ(section.contours.size(), 4);
    ASSERT_EQ(section.segments.size(), 8);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_FALSE(section.contours[1].closed);
    ASSERT_FALSE(section.contours[2].closed);
    ASSERT_FALSE(section.contours[3].closed);

    ASSERT_LT(std::abs(section.contours[1].points.front().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.front().y - 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().x - 0.25), 1e-12);
    ASSERT_LT(std::abs(section.contours[1].points.back().y - 2.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[2].points.front().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.front().y + 1.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().x - 0.75), 1e-12);
    ASSERT_LT(std::abs(section.contours[2].points.back().y - 0.0), 1e-12);

    ASSERT_LT(std::abs(section.contours[3].points.front().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.front().y - 0.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().x - 3.0), 1e-12);
    ASSERT_LT(std::abs(section.contours[3].points.back().y - 1.0), 1e-12);

    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Mixed);
}

// Demonstrates that a mid-plane cut through a unit cube (whose 4 intersected
// faces are mutually non-coplanar) stitches into exactly one closed contour
// with exactly 4 corner points 茅藛?proving that segment stitching across
// non-coplanar face pairs is deterministic for convex closed bodies.
// This narrows the NonPlanarDominantSectionGraphRemainsOpen gap to the
// specific subset: convex-body oblique-face-to-face stitching.
TEST(Section3dCapabilityTest, UnitCubeMidPlaneSectionYieldsFourSegmentClosedContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // y=0.5 mid-plane cuts left/right/bottom/top faces (all mutually
    // non-coplanar).
    const SCPlane midCut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section = Section(cubeBody, midCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly one closed rectangular contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Exactly 4 segments (one per intersected face).
    ASSERT_EQ(section.segments.size(), 4);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    // Area of the 1猫锟? rectangle in the xz plane.
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Count(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same unit-cube y=0.5 mid-plane subset also holds on the
// Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeMidPlaneSectionYieldsFourSegmentClosedContour)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane midCut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section = Section(converted.body, midCut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Count(), 1);
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components2 = BuildSectionComponents(section);
    ASSERT_TRUE(components2.IsValid());
    ASSERT_EQ(components2.components.size(), 1);
}

// Demonstrates that Section() on a closed prism-like polyhedron where the
// cut plane is oblique to all faces produces a deterministic closed contour
// and that the sum of contour edge lengths (total rebar perimeter) is stable.
TEST(Section3dCapabilityTest, ObliquePrismSectionYieldsDeterministicContourLength)
{
    // Right triangular prism: two triangular faces (top/bottom) + three quad
    // side faces, all z-aligned, base triangle in z=0 plane.
    const SCPoint3d a0{0.0, 0.0, 0.0};
    const SCPoint3d b0{1.0, 0.0, 0.0};
    const SCPoint3d c0{0.5, 0.866, 0.0};
    const SCPoint3d a1{0.0, 0.0, 1.0};
    const SCPoint3d b1{1.0, 0.0, 1.0};
    const SCPoint3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism(
        {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                          PolyhedronLoop3d({a0, c0, b0})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                          PolyhedronLoop3d({a1, b1, c1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.0, 0.5}, SCVector3d{0.0, -1.0, 0.0}),
                          PolyhedronLoop3d({a0, b0, b1, a1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.75, 0.433, 0.5}, SCVector3d{0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({b0, c0, c1, b1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 0.433, 0.5}, SCVector3d{-0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({c0, a0, a1, c1}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 5);

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Horizontal mid-section of a triangular prism = triangle 茅藛?closed
    // contour
    ASSERT_FALSE(section.contours.empty());
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.segments.size(), 3);

    // Perimeter of the cross-section triangle should equal side-length sum.
    // Equilateral triangle of side 1: perimeter = 3.0 (approximately, since
    // c0 uses 0.866 茅藛?sqrt(3)/2).
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Perimeter 茅藛?3.0 (equilateral triangle side=1)
    ASSERT_GT(totalLength, 2.5);
    ASSERT_LT(totalLength, 3.5);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates the same triangular-prism deterministic contour-length subset
// also holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepObliquePrismSectionYieldsDeterministicContourLength)
{
    const SCPoint3d a0{0.0, 0.0, 0.0};
    const SCPoint3d b0{1.0, 0.0, 0.0};
    const SCPoint3d c0{0.5, 0.866, 0.0};
    const SCPoint3d a1{0.0, 0.0, 1.0};
    const SCPoint3d b1{1.0, 0.0, 1.0};
    const SCPoint3d c1{0.5, 0.866, 1.0};

    const PolyhedronBody prism(
        {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
                          PolyhedronLoop3d({a0, c0, b0})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
                          PolyhedronLoop3d({a1, b1, c1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.0, 0.5}, SCVector3d{0.0, -1.0, 0.0}),
                          PolyhedronLoop3d({a0, b0, b1, a1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.75, 0.433, 0.5}, SCVector3d{0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({b0, c0, c1, b1})),
         PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.25, 0.433, 0.5}, SCVector3d{-0.866, 0.5, 0.0}),
                          PolyhedronLoop3d({c0, a0, a1, c1}))});
    ASSERT_TRUE(prism.IsValid());

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.289, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_FALSE(section.contours.empty());
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.segments.size(), 3);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 2.5);
    ASSERT_LT(totalLength, 3.5);
    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
}

// Demonstrates that cutting the unit cube with a plane perpendicular to the
// x-axis (x=0.5) produces a deterministic 1猫锟? square cross-section with
// known perimeter (4.0) and area (1.0). Extends rebar-length coverage to
// a third axis direction, complementing the y-axis mid-plane and the
// triangular-prism tests already present.
TEST(Section3dCapabilityTest, UnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // x=0.5 plane intersects the four faces whose vertex x-coordinates span
    // the range (front/back/bottom/top), leaving left (x=0) and right (x=1)
    // parallel and uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly four segments (one per intersected face).
    ASSERT_EQ(section.segments.size(), 4);

    // Exactly one closed contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Perimeter of the 1猫锟? square cross-section in the yz-plane = 4.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same x-axis deterministic rebar perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates that cutting a 2猫锟?猫锟? rectangular prism (box) at its z
// mid-plane (z=0.5) yields a deterministic 2猫锟? square cross-section with
// perimeter 8.0 and area 4.0. Validates that the rebar-line perimeter assertion
// scales to non-square cross-sections, extending 猫鹿鈥∶モ€樎疵︼拷?6 rebar
// coverage beyond unit triangular-prism and unit-cube subsets.
TEST(Section3dCapabilityTest, RectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    // 2猫锟?猫锟? rectangular box: x in [0,2], y in [0,2], z in [0,1].
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    // z=0.5 mid-plane cuts all four side faces (front/back/left/right),
    // yielding a 2猫锟? square cross-section; bottom and top remain parallel
    // and uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    // Perimeter of 2猫锟? square cross-section = 8.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 7.0);
    ASSERT_LT(totalLength, 9.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 4.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same rectangular-prism deterministic perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepRectangularPrismMidSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 7.0);
    ASSERT_LT(totalLength, 9.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 4.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset covers an
// orthogonal axis on the Polyhedron path: x=1 mid-cut yields a 2猫锟? rectangle
// (y茅藛颅氓艙录0,2], z茅藛颅氓艙录0,1]) with stable segments/perimeter/area.
TEST(Section3dCapabilityTest, RectangularPrismXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    // x=1 cuts front(y=0)/back(y=2)/bottom(z=0)/top(z=1) 茅藛?4 faces, leaving
    // the two x-normal caps parallel and uncut 茅藛?2猫锟? rectangle in the
    // yz-plane.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(prism, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    // Perimeter of 2猫锟? rectangle = 6.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 5.0);
    ASSERT_LT(totalLength, 7.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset also covers an
// orthogonal axis on the Brep path: x=1 mid-cut yields a 2x1 rectangle with
// stable segments/perimeter/area.
TEST(Section3dCapabilityTest, BrepRectangularPrismXAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{1.0, 0.0, 0.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);
    ASSERT_EQ(section.segments.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 5.0);
    ASSERT_LT(totalLength, 7.0);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 2.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates rectangular-prism deterministic perimeter subset covers the
// y-axis on the Polyhedron path: y=1 mid-cut yields a 2猫锟? rectangle
// (x茅藛颅氓艙录0,2], z茅藛颅氓艙录0,1]) with stable segments/perimeter/area.
TEST(Section3dCapabilityTest, RectangularPrismYAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    // y=1 cuts left(x=0)/right(x=2)/bottom(z=0)/top(z=1) 茅藛?2猫锟? rectangle
    // in xz.
    const SCPlane cut2 = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section2 = Section(prism, cut2);
    ASSERT_TRUE(section2.success);
    ASSERT_TRUE(section2.IsValid());

    ASSERT_EQ(section2.contours.size(), 1);
    ASSERT_TRUE(section2.contours[0].closed);
    ASSERT_EQ(section2.contours[0].points.size(), 4);
    ASSERT_EQ(section2.segments.size(), 4);

    double totalLength2 = 0.0;
    const auto& pts2 = section2.contours[0].points;
    for (std::size_t i = 0; i < pts2.size(); ++i)
    {
        const SCPoint3d& p0 = pts2[i];
        const SCPoint3d& p1 = pts2[(i + 1) % pts2.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength2 += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength2, 5.0);
    ASSERT_LT(totalLength2, 7.0);

    ASSERT_EQ(ClassifySectionContent(section2), SectionContentKind3d::Area);
    ASSERT_EQ(section2.polygons.size(), 1);
    ASSERT_LT(std::abs(section2.polygons[0].Area() - 2.0), 1e-12);

    const auto topology2 = BuildSectionTopology(section2);
    ASSERT_TRUE(topology2.IsValid());
    ASSERT_EQ(topology2.Roots().size(), 1);

    const auto components2 = BuildSectionComponents(section2);
    ASSERT_TRUE(components2.IsValid());
    ASSERT_EQ(components2.components.size(), 1);
}

// Demonstrates the same rectangular-prism y-axis subset also holds on the
// Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepRectangularPrismYAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody prism(
        {PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 1.0}, SCPoint3d{0.0, 0.0, 1.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 2.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 2.0, 0.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{0.0, 0.0, 1.0}, SCPoint3d{0.0, 2.0, 1.0}, SCPoint3d{0.0, 2.0, 0.0}})),
         PolyhedronFace3d(
             SCPlane::FromPointAndNormal(SCPoint3d{2.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}),
             PolyhedronLoop3d(
                 {SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{2.0, 2.0, 0.0}, SCPoint3d{2.0, 2.0, 1.0}, SCPoint3d{2.0, 0.0, 1.0}}))});
    ASSERT_TRUE(prism.IsValid());
    ASSERT_EQ(prism.FaceCount(), 6);

    const auto converted = ConvertToBrepBody(prism);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut3 = SCPlane::FromPointAndNormal(SCPoint3d{1.0, 1.0, 0.5}, SCVector3d{0.0, 1.0, 0.0});
    const auto section3 = Section(converted.body, cut3);
    ASSERT_TRUE(section3.success);
    ASSERT_TRUE(section3.IsValid());

    ASSERT_EQ(section3.contours.size(), 1);
    ASSERT_TRUE(section3.contours[0].closed);
    ASSERT_EQ(section3.contours[0].points.size(), 4);
    ASSERT_EQ(section3.segments.size(), 4);

    double totalLength3 = 0.0;
    const auto& pts3 = section3.contours[0].points;
    for (std::size_t i = 0; i < pts3.size(); ++i)
    {
        const SCPoint3d& p0 = pts3[i];
        const SCPoint3d& p1 = pts3[(i + 1) % pts3.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength3 += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength3, 5.0);
    ASSERT_LT(totalLength3, 7.0);

    ASSERT_EQ(ClassifySectionContent(section3), SectionContentKind3d::Area);
    ASSERT_EQ(section3.polygons.size(), 1);
    ASSERT_LT(std::abs(section3.polygons[0].Area() - 2.0), 1e-12);

    const auto topology3 = BuildSectionTopology(section3);
    ASSERT_TRUE(topology3.IsValid());
    ASSERT_EQ(topology3.Roots().size(), 1);

    const auto components3 = BuildSectionComponents(section3);
    ASSERT_TRUE(components3.IsValid());
    ASSERT_EQ(components3.components.size(), 1);
}

// Demonstrates that cutting a unit cube (x,y,z 茅藛?[0,1]) at z=0.5 (horizontal
// mid-plane) yields a deterministic 1猫锟? square cross-section with
// perimeter 4.0 and area 1.0, extending the rebar-line coverage to the z-axis
// direction on the PolyhedronBody path.
TEST(Section3dCapabilityTest, UnitCubeZAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    // z=0.5 plane intersects the four lateral faces whose vertex z-coordinates
    // span the range, leaving bottom (z=0) and top (z=1) uncut.
    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(cubeBody, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    // Exactly four segments (one per intersected lateral face).
    ASSERT_EQ(section.segments.size(), 4);

    // Exactly one closed contour with 4 corner points.
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    // Perimeter of the 1猫锟? square cross-section in the xy-plane = 4.0.
    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}

// Demonstrates the same z-axis deterministic rebar perimeter subset also
// holds on the Brep path after Polyhedron->Brep conversion.
TEST(Section3dCapabilityTest, BrepUnitCubeZAxisSectionYieldsDeterministicRebarPerimeter)
{
    const PolyhedronBody cubeBody = Geometry::Test::BuildUnitCubeBody();
    ASSERT_TRUE(cubeBody.IsValid());

    const auto converted = ConvertToBrepBody(cubeBody);
    ASSERT_TRUE(converted.success);
    ASSERT_EQ(converted.issue, BrepConversionIssue3d::None);
    ASSERT_TRUE(converted.body.IsValid());

    const SCPlane cut = SCPlane::FromPointAndNormal(SCPoint3d{0.5, 0.5, 0.5}, SCVector3d{0.0, 0.0, 1.0});
    const auto section = Section(converted.body, cut);
    ASSERT_TRUE(section.success);
    ASSERT_TRUE(section.IsValid());

    ASSERT_EQ(section.segments.size(), 4);
    ASSERT_EQ(section.contours.size(), 1);
    ASSERT_TRUE(section.contours[0].closed);
    ASSERT_EQ(section.contours[0].points.size(), 4);

    double totalLength = 0.0;
    const auto& pts = section.contours[0].points;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const SCPoint3d& p0 = pts[i];
        const SCPoint3d& p1 = pts[(i + 1) % pts.size()];
        const double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    ASSERT_GT(totalLength, 3.5);
    ASSERT_LT(totalLength, 4.5);

    ASSERT_EQ(ClassifySectionContent(section), SectionContentKind3d::Area);
    ASSERT_EQ(section.polygons.size(), 1);
    ASSERT_LT(std::abs(section.polygons[0].Area() - 1.0), 1e-12);

    const auto topology = BuildSectionTopology(section);
    ASSERT_TRUE(topology.IsValid());
    ASSERT_EQ(topology.Roots().size(), 1);

    const auto components = BuildSectionComponents(section);
    ASSERT_TRUE(components.IsValid());
    ASSERT_EQ(components.components.size(), 1);
}
