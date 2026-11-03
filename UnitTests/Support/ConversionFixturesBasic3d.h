#pragma once

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry.h"
#include "Support/Fixtures3d.h"

namespace Geometry::Test::ConversionFixtures
{
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
    using Geometry::MeshConversionIssue3d;
    using Geometry::SCPlane;
    using Geometry::SCPlaneSurface;
    using Geometry::SCPoint2d;
    using Geometry::SCPoint3d;
    using Geometry::PolyhedronBody;
    using Geometry::PolyhedronBrepBodyConversion3d;
    using Geometry::PolyhedronFace3d;
    using Geometry::PolyhedronLoop3d;
    using Geometry::PolyhedronMeshConversion3d;
    using Geometry::SCPolyline2d;
    using Geometry::SCPolylineClosure;
    using Geometry::ISCSurface;
    using Geometry::SCVector3d;

    namespace
    {
        SCPoint3d SkewPoint(const SCPoint3d& point)
        {
            // Affine skew keeps each face planar while making the body
            // non-axis-aligned.
            return SCPoint3d{point.x + 0.3 * point.z, point.y + 0.2 * point.x, point.z};
        }

        SCPlane SkewPlane(const SCPlane& plane)
        {
            // For p' = A p (with A defined by SkewPoint), normal transforms as n' =
            // (A^{-1})^T n.
            const SCVector3d n = plane.normal;
            const SCVector3d transformedNormal{n.x - 0.2 * n.y, n.y, -0.3 * n.x + 0.06 * n.y + n.z};

            return SCPlane::FromPointAndNormal(SkewPoint(plane.origin), transformedNormal);
        }

        PolyhedronLoop3d SkewLoop(const PolyhedronLoop3d& loop)
        {
            std::vector<SCPoint3d> vertices;
            const std::vector<SCPoint3d>& source = loop.Vertices();
            vertices.reserve(source.size());
            for (const SCPoint3d& point : source)
            {
                vertices.push_back(SkewPoint(point));
            }
            return PolyhedronLoop3d(std::move(vertices));
        }

        std::vector<PolyhedronLoop3d> CopyFaceHoles(const PolyhedronFace3d& face)
        {
            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(face.HoleCount());
            for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
            {
                holes.push_back(face.HoleAt(holeIndex));
            }
            return holes;
        }

        SCPlane SupportPlaneFromLoop(const PolyhedronLoop3d& loop)
        {
            const std::vector<SCPoint3d>& vertices = loop.Vertices();
            if (vertices.size() < 3)
            {
                return SCPlane{};
            }

            const SCPoint3d& p0 = vertices[0];
            const SCPoint3d& p1 = vertices[1];
            const SCPoint3d& p2 = vertices[2];
            return SCPlane::FromPointAndNormal(p0, Cross(p1 - p0, p2 - p0));
        }

        PolyhedronFace3d SkewFace(const PolyhedronFace3d& face)
        {
            PolyhedronLoop3d outer = SkewLoop(face.OuterLoop());
            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                holes.push_back(SkewLoop(face.HoleAt(i)));
            }

            return PolyhedronFace3d(SkewPlane(face.SupportPlane()), std::move(outer), std::move(holes));
        }

        PolyhedronBody BuildSkewedUnitCubeBody()
        {
            const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
            std::vector<PolyhedronFace3d> faces;
            faces.reserve(cube.FaceCount());
            for (const PolyhedronFace3d& face : cube.Faces())
            {
                faces.push_back(SkewFace(face));
            }
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildSupportPlaneMismatchedCubeBody()
        {
            try
            {
                const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
                std::vector<PolyhedronFace3d> faces = cube.Faces();

                if (faces.empty())
                {
                    return PolyhedronBody();
                }

                const PolyhedronFace3d first = faces.front();
                faces.front() = PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.1}, SCVector3d{0.0, 0.0, 1.0}),
                                                 first.OuterLoop());

                return PolyhedronBody(std::move(faces));
            } catch (const std::exception&)
            {
                return PolyhedronBody();
            }
        }

        PolyhedronLoop3d TranslateLoop3d(const PolyhedronLoop3d& loop, const SCVector3d& delta)
        {
            try
            {
                std::vector<SCPoint3d> vertices;
                vertices.reserve(loop.VertexCount());
                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    vertices.push_back(loop.VertexAt(i) + delta);
                }
                return PolyhedronLoop3d(std::move(vertices));
            } catch (const std::exception&)
            {
                return PolyhedronLoop3d();
            }
        }

        PolyhedronFace3d TranslateFace3d(const PolyhedronFace3d& face, const SCVector3d& delta)
        {
            try
            {
                const SCPlane translatedPlane =
                    SCPlane::FromPointAndNormal(face.SupportPlane().origin + delta, face.SupportPlane().normal);

                PolyhedronLoop3d outer = TranslateLoop3d(face.OuterLoop(), delta);
                std::vector<PolyhedronLoop3d> holes;
                holes.reserve(face.HoleCount());
                for (std::size_t i = 0; i < face.HoleCount(); ++i)
                {
                    holes.push_back(TranslateLoop3d(face.HoleAt(i), delta));
                }

                return PolyhedronFace3d(translatedPlane, std::move(outer), std::move(holes));
            } catch (const std::exception&)
            {
                return PolyhedronFace3d();
            }
        }

        PolyhedronBody BuildTwoSeparatedUnitCubeBody()
        {
            try
            {
                const PolyhedronBody first = Geometry::Test::BuildUnitCubeBody();
                std::vector<PolyhedronFace3d> faces = first.Faces();

                const SCVector3d delta{3.0, 0.0, 0.0};
                for (const PolyhedronFace3d& face : first.Faces())
                {
                    faces.push_back(TranslateFace3d(face, delta));
                }

                return PolyhedronBody(std::move(faces));
            } catch (const std::exception&)
            {
                return PolyhedronBody();
            }
        }

        PolyhedronBody BuildDeformedUnitCubeBody()
        {
            // V0 is the origin corner shared by bottom / front / left faces.
            // Displacing it makes those three faces non-planar.
            const SCPoint3d v0{0.1, 0.1, -0.1};  // displaced from (0,0,0)
            const SCPoint3d v1{1.0, 0.0, 0.0};
            const SCPoint3d v2{1.0, 1.0, 0.0};
            const SCPoint3d v3{0.0, 1.0, 0.0};
            const SCPoint3d v4{0.0, 0.0, 1.0};
            const SCPoint3d v5{1.0, 0.0, 1.0};
            const SCPoint3d v6{1.0, 1.0, 1.0};
            const SCPoint3d v7{0.0, 1.0, 1.0};

            // Support planes are the original axis-aligned planes of the unit cube
            // 闂?    // intentionally wrong for the three faces that contain v0.
            const SCPlane bottom{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}};
            const SCPlane top{SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}};
            const SCPlane front{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}};
            const SCPlane back{SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}};
            const SCPlane left{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}};
            const SCPlane right{SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}};

            return PolyhedronBody({
                PolyhedronFace3d(bottom, PolyhedronLoop3d({v0, v3, v2, v1}), {}),
                PolyhedronFace3d(top, PolyhedronLoop3d({v4, v5, v6, v7}), {}),
                PolyhedronFace3d(front, PolyhedronLoop3d({v0, v1, v5, v4}), {}),
                PolyhedronFace3d(back, PolyhedronLoop3d({v3, v7, v6, v2}), {}),
                PolyhedronFace3d(left, PolyhedronLoop3d({v0, v4, v7, v3}), {}),
                PolyhedronFace3d(right, PolyhedronLoop3d({v1, v2, v6, v5}), {}),
            });
        }

        // Displaces two opposite cube corners so that all six quad faces become
        // non-planar against their original support planes. This is a stronger
        // variant of the single-displaced-vertex deformed cube.
        PolyhedronBody BuildDualDeformedUnitCubeBody()
        {
            const SCPoint3d v0{0.1, 0.1, -0.1};  // displaced from (0,0,0)
            const SCPoint3d v1{1.0, 0.0, 0.0};
            const SCPoint3d v2{1.0, 1.0, 0.0};
            const SCPoint3d v3{0.0, 1.0, 0.0};
            const SCPoint3d v4{0.0, 0.0, 1.0};
            const SCPoint3d v5{1.0, 0.0, 1.0};
            const SCPoint3d v6{0.9, 0.9, 1.1};  // displaced from (1,1,1)
            const SCPoint3d v7{0.0, 1.0, 1.0};

            const SCPlane bottom{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, -1.0}};
            const SCPlane top{SCPoint3d{0.0, 0.0, 1.0}, SCVector3d{0.0, 0.0, 1.0}};
            const SCPlane front{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, -1.0, 0.0}};
            const SCPlane back{SCPoint3d{0.0, 1.0, 0.0}, SCVector3d{0.0, 1.0, 0.0}};
            const SCPlane left{SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{-1.0, 0.0, 0.0}};
            const SCPlane right{SCPoint3d{1.0, 0.0, 0.0}, SCVector3d{1.0, 0.0, 0.0}};

            return PolyhedronBody({
                PolyhedronFace3d(bottom, PolyhedronLoop3d({v0, v3, v2, v1}), {}),
                PolyhedronFace3d(top, PolyhedronLoop3d({v4, v5, v6, v7}), {}),
                PolyhedronFace3d(front, PolyhedronLoop3d({v0, v1, v5, v4}), {}),
                PolyhedronFace3d(back, PolyhedronLoop3d({v3, v7, v6, v2}), {}),
                PolyhedronFace3d(left, PolyhedronLoop3d({v0, v4, v7, v3}), {}),
                PolyhedronFace3d(right, PolyhedronLoop3d({v1, v2, v6, v5}), {}),
            });
        }

        PolyhedronBody BuildDeformedUnitCubeWithDuplicateLoopBody()
        {
            const PolyhedronBody base = BuildDeformedUnitCubeBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d leftFace = faces[4];
            std::vector<SCPoint3d> loop = leftFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[4] =
                PolyhedronFace3d(leftFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), CopyFaceHoles(leftFace));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildDeformedUnitCubeWithDualDuplicateLoopBody()
        {
            const PolyhedronBody base = BuildDeformedUnitCubeWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d frontFace = faces[2];
            std::vector<SCPoint3d> loop = frontFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[2] =
                PolyhedronFace3d(frontFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), CopyFaceHoles(frontFace));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildDualDeformedUnitCubeWithDuplicateLoopBody()
        {
            const PolyhedronBody base = BuildDualDeformedUnitCubeBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d leftFace = faces[4];
            std::vector<SCPoint3d> loop = leftFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[4] =
                PolyhedronFace3d(leftFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), CopyFaceHoles(leftFace));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildDualDeformedUnitCubeWithDualDuplicateLoopBody()
        {
            const PolyhedronBody base = BuildDualDeformedUnitCubeWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d rightFace = faces[5];
            std::vector<SCPoint3d> loop = rightFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[5] =
                PolyhedronFace3d(rightFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)), CopyFaceHoles(rightFace));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildMildlyNonPlanarCubeFaceBody()
        {
            const PolyhedronBody cube = Geometry::Test::BuildUnitCubeBody();
            std::vector<PolyhedronFace3d> faces = cube.Faces();

            const std::size_t topFaceIndex = 1;
            const PolyhedronFace3d top = faces[topFaceIndex];
            std::vector<SCPoint3d> topVertices;
            topVertices.reserve(top.OuterLoop().VertexCount());
            for (std::size_t i = 0; i < top.OuterLoop().VertexCount(); ++i)
            {
                topVertices.push_back(top.OuterLoop().VertexAt(i));
            }

            // Introduce mild non-coplanarity on one vertex.
            topVertices[2].z += 0.05;
            faces[topFaceIndex] = PolyhedronFace3d(top.SupportPlane(), PolyhedronLoop3d(std::move(topVertices)));

            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildMildlyNonPlanarHoledFaceBody()
        {
            const PolyhedronLoop3d outer({
                SCPoint3d{0.0, 0.0, 0.0},
                SCPoint3d{4.0, 0.0, 0.0},
                SCPoint3d{4.0, 4.0, 0.0},
                SCPoint3d{0.0, 4.0, 0.0},
            });
            std::vector<SCPoint3d> holeVertices{
                SCPoint3d{1.0, 1.0, 0.0}, SCPoint3d{3.0, 1.0, 0.0}, SCPoint3d{3.0, 3.0, 0.0}, SCPoint3d{1.0, 3.0, 0.0}};
            holeVertices[2].z += 0.03;

            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0}),
                                        outer,
                                        {PolyhedronLoop3d(std::move(holeVertices))});
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildHoleDominatedNonPlanarHoledFaceBody()
        {
            std::vector<SCPoint3d> outer{
                SCPoint3d{0.0, 0.0, 0.12}, SCPoint3d{6.0, 0.0, 0.00}, SCPoint3d{6.0, 6.0, 0.10}, SCPoint3d{0.0, 6.0, 0.00}};
            std::vector<SCPoint3d> hole{
                SCPoint3d{1.0, 1.0, 0.0}, SCPoint3d{5.0, 1.0, 0.0}, SCPoint3d{5.0, 5.0, 0.0}, SCPoint3d{1.0, 5.0, 0.0}};

            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d(std::move(outer)),
                                        {PolyhedronLoop3d(std::move(hole))});
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildSharedEdgeHoleDominatedMixedContentBody()
        {
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0});

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{2.0, 0.0, 0.0};
            const SCPoint3d c{2.0, 2.0, 0.0};
            const SCPoint3d d{0.0, 2.0, 0.0};

            const SCPoint3d e{6.0, 0.0, 0.12};
            const SCPoint3d f{6.0, 2.0, 0.10};

            std::vector<SCPoint3d> hole{
                SCPoint3d{2.6, 0.4, 0.0}, SCPoint3d{5.4, 0.4, 0.0}, SCPoint3d{5.4, 1.6, 0.0}, SCPoint3d{2.6, 1.6, 0.0}};

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(
                mismatchedPlane, PolyhedronLoop3d({b, e, f, c}), {PolyhedronLoop3d(std::move(hole))});
            return PolyhedronBody({faceA, faceB});
        }

        PolyhedronBody BuildSharedChainHoleDominatedMixedContentBody()
        {
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0});

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{2.0, 0.0, 0.0};
            const SCPoint3d c{2.0, 2.0, 0.0};
            const SCPoint3d d{0.0, 2.0, 0.0};

            const SCPoint3d e{6.0, 0.0, 0.12};
            const SCPoint3d f{6.0, 2.0, 0.10};

            const SCPoint3d g{8.0, 0.0, 0.0};
            const SCPoint3d h{8.0, 2.0, 0.0};

            std::vector<SCPoint3d> hole{
                SCPoint3d{2.6, 0.4, 0.0}, SCPoint3d{5.4, 0.4, 0.0}, SCPoint3d{5.4, 1.6, 0.0}, SCPoint3d{2.6, 1.6, 0.0}};

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(
                mismatchedPlane, PolyhedronLoop3d({b, e, f, c}), {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentBody()
        {
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0});

            const double dx = 2.0e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{2.0, 0.0, 0.0};
            const SCPoint3d c{2.0, 2.0, 0.0};
            const SCPoint3d d{0.0, 2.0, 0.0};

            const SCPoint3d b2{2.0 + dx, 0.0, 0.0};
            const SCPoint3d c2{2.0 + dx, 2.0, 0.0};
            const SCPoint3d e{6.0 + dx, 0.0, 0.12};
            const SCPoint3d f{6.0 + dx, 2.0, 0.10};

            const SCPoint3d e2{6.0, 0.0, 0.0};
            const SCPoint3d f2{6.0, 2.0, 0.0};
            const SCPoint3d g{8.0, 0.0, 0.0};
            const SCPoint3d h{8.0, 2.0, 0.0};

            std::vector<SCPoint3d> hole{
                SCPoint3d{2.6, 0.4, 0.0}, SCPoint3d{5.4, 0.4, 0.0}, SCPoint3d{5.4, 1.6, 0.0}, SCPoint3d{2.6, 1.6, 0.0}};

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(
                mismatchedPlane, PolyhedronLoop3d({b2, e, f, c2}), {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e2, g, h, f2}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedMixedContentWithDuplicateHoleBody()
        {
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0});

            const double dx = 2.0e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{2.0, 0.0, 0.0};
            const SCPoint3d c{2.0, 2.0, 0.0};
            const SCPoint3d d{0.0, 2.0, 0.0};

            const SCPoint3d b2{2.0 + dx, 0.0, 0.0};
            const SCPoint3d c2{2.0 + dx, 2.0, 0.0};
            const SCPoint3d e{6.0 + dx, 0.0, 0.12};
            const SCPoint3d f{6.0 + dx, 2.0, 0.10};

            const SCPoint3d e2{6.0, 0.0, 0.0};
            const SCPoint3d f2{6.0, 2.0, 0.0};
            const SCPoint3d g{8.0, 0.0, 0.0};
            const SCPoint3d h{8.0, 2.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{2.6, 0.4, 0.0},
                                      SCPoint3d{5.4, 0.4, 0.0},
                                      SCPoint3d{5.4, 0.4, 0.0},
                                      SCPoint3d{5.4, 1.6, 0.0},
                                      SCPoint3d{2.6, 1.6, 0.0}};

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(
                mismatchedPlane, PolyhedronLoop3d({b2, e, f, c2}), {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e2, g, h, f2}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainHoleDominatedFullCompositionBody()
        {
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0});

            const double dx = 2.0e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{2.0, 0.0, 0.0};
            const SCPoint3d c{2.0, 2.0, 0.0};
            const SCPoint3d d{0.0, 2.0, 0.0};

            const SCPoint3d b2{2.0 + dx, 0.0, 0.0};
            const SCPoint3d c2{2.0 + dx, 2.0, 0.0};
            const SCPoint3d m{4.0 + dx, 0.0, 0.0};
            const SCPoint3d e{6.0 + dx, 0.0, 0.12};
            const SCPoint3d f{6.0 + dx, 2.0, 0.10};

            const SCPoint3d e2{6.0, 0.0, 0.0};
            const SCPoint3d f2{6.0, 2.0, 0.0};
            const SCPoint3d g{8.0, 0.0, 0.0};
            const SCPoint3d h{8.0, 2.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{2.6, 0.4, 0.0},
                                      SCPoint3d{5.4, 0.4, 0.0},
                                      SCPoint3d{5.4, 0.4, 0.0},
                                      SCPoint3d{5.4, 1.6, 0.0},
                                      SCPoint3d{2.6, 1.6, 0.0}};

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(
                mismatchedPlane, PolyhedronLoop3d({b2, m, e, f, c2}), {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e2, g, h, f2}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildSupportPlaneMismatchedCollinearLeadingLoopBody()
        {
            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d({
                                            SCPoint3d{0.0, 0.0, 0.0},
                                            SCPoint3d{1.0, 0.0, 0.0},
                                            SCPoint3d{2.0, 0.0, 0.0},
                                            SCPoint3d{2.0, 1.0, 0.0},
                                            SCPoint3d{0.0, 1.0, 0.0},
                                        }));
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildDuplicateVertexLoopBody()
        {
            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d({
                                            SCPoint3d{0.0, 0.0, 0.0},
                                            SCPoint3d{2.0, 0.0, 0.0},
                                            SCPoint3d{2.0, 0.0, 0.0},
                                            SCPoint3d{2.0, 1.0, 0.0},
                                            SCPoint3d{0.0, 1.0, 0.0},
                                        }));
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildDuplicateVertexHoleLoopBody()
        {
            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.2}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d({
                                            SCPoint3d{0.0, 0.0, 0.0},
                                            SCPoint3d{4.0, 0.0, 0.0},
                                            SCPoint3d{4.0, 4.0, 0.0},
                                            SCPoint3d{0.0, 4.0, 0.0},
                                        }),
                                        {
                                            PolyhedronLoop3d({
                                                SCPoint3d{1.0, 1.0, 0.0},
                                                SCPoint3d{3.0, 1.0, 0.0},
                                                SCPoint3d{3.0, 1.0, 0.0},
                                                SCPoint3d{3.0, 3.0, 0.0},
                                                SCPoint3d{1.0, 3.0, 0.0},
                                            }),
                                        });
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildCompositeRepairStressFaceBody()
        {
            std::vector<SCPoint3d> outer{
                SCPoint3d{0.0, 0.0, 0.0},
                SCPoint3d{1.0, 0.0, 0.0},
                SCPoint3d{2.0, 0.0, 0.0},
                SCPoint3d{4.0, 0.0, 0.0},
                SCPoint3d{4.0, 4.0, 0.0},
                SCPoint3d{0.0, 4.0, 0.0},
                SCPoint3d{0.0, 4.0, 0.0},
            };
            // Mildly non-planar disturbance on outer loop.
            outer[4].z += 0.04;

            std::vector<SCPoint3d> hole{
                SCPoint3d{1.0, 1.0, 0.0},
                SCPoint3d{3.0, 1.0, 0.0},
                SCPoint3d{3.0, 1.0, 0.0},
                SCPoint3d{3.0, 3.0, 0.0},
                SCPoint3d{1.0, 3.0, 0.0},
            };
            // Mildly non-planar disturbance on hole loop.
            hole[3].z += 0.03;

            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.25}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d(std::move(outer)),
                                        {PolyhedronLoop3d(std::move(hole))});
            return PolyhedronBody({face});
        }

    }  // namespace

}  // namespace Geometry::Test::ConversionFixtures
