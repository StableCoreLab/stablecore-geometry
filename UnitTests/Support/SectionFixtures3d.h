#pragma once

#include <algorithm>
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
