#pragma once

#include "Support/ConversionFixturesBasic3d.h"

namespace Geometry::Test::ConversionFixtures
{
    namespace
    {
        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody()
        {
            // Closed tetrahedron where two logical shared vertices (v0, v1) are
            // each represented by near-equal (<eps) variants across faces.
            const double s = 1e-5;

            const SCPoint3d v0a{0.0, 0.0, 0.0};
            const SCPoint3d v0b{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{-1.0e-7, 0.0, 0.0};

            const SCPoint3d v1a{s + 2.0e-7, 0.0, 1.2e-6};
            const SCPoint3d v1b{s - 1.0e-7, 0.0, 1.2e-6};
            const SCPoint3d v1c{s, 0.0, 1.2e-6};

            const SCPoint3d v2{0.5 * s, 0.866 * s, 0.8e-6};
            const SCPoint3d v3{0.5 * s, 0.289 * s, 0.816 * s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3, v1b})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v3, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2, v3}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody()
        {
            // Reuse dual-vertices near-equal closed-tetra and inject one duplicate
            // leading vertex on a triangular face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 4)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d face = faces[2];
            std::vector<SCPoint3d> loop = face.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[2] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate dual-vertices near-equal closed-tetra
            // variant and inject another duplicate leading vertex on a second face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronDualVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 4)
            {
                return PolyhedronBody();
            }
            const PolyhedronFace3d face = faces[1];
            std::vector<SCPoint3d> loop = face.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }
            loop.insert(loop.begin(), loop.front());
            faces[1] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody()
        {
            // Closed tetrahedron where all four logical shared vertices each have
            // near-equal (<eps) position variants across their three incident
            // faces, exercising simultaneous representative-average placement for
            // every shared vertex at once.
            const double s = 1e-5;

            // v0 near {0,0,0}: faces 0,1,3
            const SCPoint3d v0a{0.0, 0.0, 0.0};
            const SCPoint3d v0b{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{-1.0e-7, 0.0, 0.0};
            // avg x = 0

            // v1 near {s,0,1.2e-6}: faces 0,1,2
            const SCPoint3d v1a{s + 2.0e-7, 0.0, 1.2e-6};
            const SCPoint3d v1b{s - 1.0e-7, 0.0, 1.2e-6};
            const SCPoint3d v1c{s, 0.0, 1.2e-6};
            // avg x = s + 1e-7/3

            // v2 near {0.5*s, 0.866*s, 0.8e-6}: faces 0,2,3
            const SCPoint3d v2a{0.5 * s + 1.0e-7, 0.866 * s, 0.8e-6};
            const SCPoint3d v2b{0.5 * s - 1.0e-7, 0.866 * s, 0.8e-6};
            const SCPoint3d v2c{0.5 * s, 0.866 * s + 1.0e-7, 0.8e-6};
            // avg x = 0.5*s, avg y = 0.866*s + 1e-7/3

            // v3 near {0.5*s, 0.289*s, 0.816*s}: faces 1,2,3
            const SCPoint3d v3a{0.5 * s + 2.0e-7, 0.289 * s, 0.816 * s};
            const SCPoint3d v3b{0.5 * s - 1.0e-7, 0.289 * s, 0.816 * s};
            const SCPoint3d v3c{0.5 * s, 0.289 * s, 0.816 * s};
            // avg x = 0.5*s + 1e-7/3

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2a})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3a, v1b})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v3b, v2b})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2c, v3c}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody()
        {
            // Reuse all-vertices near-equal closed-tetra and inject a duplicate
            // leading vertex on one triangular face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 4)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d face = faces[2];
            std::vector<SCPoint3d> loop = face.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[2] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate all-vertices near-equal closed-tetra
            // variant and inject another duplicate leading vertex on a second face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedTetrahedronAllVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 4)
            {
                return PolyhedronBody();
            }
            const PolyhedronFace3d face = faces[1];
            std::vector<SCPoint3d> loop = face.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }
            loop.insert(loop.begin(), loop.front());
            faces[1] = PolyhedronFace3d(face.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesBody()
        {
            // Closed triangular prism where two non-adjacent shared vertices (v0 at
            // bottom-origin and v4 at top-right) each appear with near-equal (<eps)
            // position variants across their three incident faces. Tests that
            // representative-average placement scales to prism topology.
            // Expected: FaceCount=5 / VertexCount=6 / EdgeCount=9 / closed-shell.
            const double s = 1e-5;

            // v0 near {0,0,0}: bottom + front-side + left-side
            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};
            // avg x = 0

            const SCPoint3d v1{s, 0.0, 0.0};
            const SCPoint3d v2{0.5 * s, 0.866 * s, 0.0};
            const SCPoint3d v3{0.0, 0.0, s};

            // v4 near {s,0,s}: top + front-side + right-side
            const SCPoint3d v4a{s + 2.0e-7, 0.0, s};
            const SCPoint3d v4b{s - 1.0e-7, 0.0, s};
            const SCPoint3d v4c{s, 0.0, s};
            // avg x = s + 1e-7/3

            const SCPoint3d v5{0.5 * s, 0.866 * s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v4a, v5})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v4b, v3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v5, v4c})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v2, v0c, v3, v5}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody()
        {
            // Reuse dual-vertices near-equal closed-prism and inject one
            // consecutive duplicate leading vertex on a quad side face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismDualVerticesBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 5)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d sideFace = faces[2];
            std::vector<SCPoint3d> loop = sideFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[2] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate dual-vertices near-equal closed-prism
            // variant and inject another duplicate leading vertex on a second side
            // face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismDualVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 5)
            {
                return PolyhedronBody();
            }
            const PolyhedronFace3d sideFace = faces[3];
            std::vector<SCPoint3d> loop = sideFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }
            loop.insert(loop.begin(), loop.front());
            faces[3] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesBody()
        {
            // Closed triangular prism where all six logical shared vertices each
            // have near-equal variants across their incident faces.
            const double s = 1e-5;

            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            const SCPoint3d v1a{s + 2.0e-7, 0.0, 0.0};
            const SCPoint3d v1b{s - 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1c{s, 0.0, 0.0};

            const SCPoint3d v2a{0.5 * s + 1.0e-7, 0.866 * s, 0.0};
            const SCPoint3d v2b{0.5 * s - 1.0e-7, 0.866 * s, 0.0};
            const SCPoint3d v2c{0.5 * s, 0.866 * s + 1.0e-7, 0.0};

            const SCPoint3d v3a{0.0, 0.0, s};
            const SCPoint3d v3b{1.0e-7, 0.0, s};
            const SCPoint3d v3c{-1.0e-7, 0.0, s};

            const SCPoint3d v4a{s + 3.0e-7, 0.0, s};
            const SCPoint3d v4b{s - 2.0e-7, 0.0, s};
            const SCPoint3d v4c{s + 1.0e-7, 0.0, s};

            const SCPoint3d v5a{0.5 * s, 0.866 * s, s};
            const SCPoint3d v5b{0.5 * s + 1.0e-7, 0.866 * s, s};
            const SCPoint3d v5c{0.5 * s - 1.0e-7, 0.866 * s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1a, v2a})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3a, v4a, v5a})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v4b, v3b})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2b, v5b, v4c})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v2c, v0c, v3c, v5c}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody()
        {
            // Reuse the all-vertices near-equal closed-prism and inject one
            // consecutive duplicate leading vertex on a quad side face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismAllVerticesBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 5)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d sideFace = faces[2];
            std::vector<SCPoint3d> vertices = sideFace.OuterLoop().Vertices();
            if (vertices.empty())
            {
                return PolyhedronBody();
            }

            vertices.insert(vertices.begin(), vertices.front());
            faces[2] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(vertices)));
            return PolyhedronBody(std::move(faces));
        }
        PolyhedronBody BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate all-vertices near-equal closed-prism
            // variant and inject another duplicate leading vertex on a second side
            // face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedPrismAllVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 5)
            {
                return PolyhedronBody();
            }
            const PolyhedronFace3d sideFace = faces[3];
            std::vector<SCPoint3d> loop = sideFace.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }
            loop.insert(loop.begin(), loop.front());
            faces[3] = PolyhedronFace3d(sideFace.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }
    }  // namespace

}  // namespace Geometry::Test::ConversionFixtures
