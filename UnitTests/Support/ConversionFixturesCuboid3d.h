#pragma once

#include "Support/ConversionFixturesBasic3d.h"

namespace Geometry::Test::ConversionFixtures
{
    namespace
    {
        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody()
        {
            // Tiny-scale 2闂?闂? rectangular box (pure-quad closed shell) where all
            // 8 logical shared vertices have near-equal variants across their 3
            // incident faces. All faces use a single mismatched support plane.
            // Tests that representative-average vertex placement works for a
            // pure-quad closed-shell topology with non-equal side lengths. Expected
            // after repair: FaceCount=6 / VertexCount=8 / EdgeCount=12 /
            // IsClosed=true.
            const double s = 1e-5;
            const double w = 2.0 * s;  // width in x-direction (twice the other dimensions)

            // v0 闂?(0,0,0): bottom + front + left
            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            // v1 闂?(w,0,0): bottom + front + right
            const SCPoint3d v1a{w + 2.0e-7, 0.0, 0.0};
            const SCPoint3d v1b{w - 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1c{w + 1.0e-7, 0.0, 0.0};

            // v2 闂?(w,s,0): bottom + back + right
            const SCPoint3d v2a{w + 1.0e-7, s, 0.0};
            const SCPoint3d v2b{w, s + 1.0e-7, 0.0};
            const SCPoint3d v2c{w, s, 0.0};

            // v3 闂?(0,s,0): bottom + back + left
            const SCPoint3d v3a{0.0, s + 2.0e-7, 0.0};
            const SCPoint3d v3b{0.0, s, 0.0};
            const SCPoint3d v3c{-1.0e-7, s, 0.0};

            // v4 闂?(0,0,s): top + front + left
            const SCPoint3d v4a{0.0, 0.0, s};
            const SCPoint3d v4b{1.0e-7, 0.0, s};
            const SCPoint3d v4c{0.0, 0.0, s + 1.0e-7};

            // v5 闂?(w,0,s): top + front + right
            const SCPoint3d v5a{w + 1.0e-7, 0.0, s};
            const SCPoint3d v5b{w, 0.0, s - 1.0e-7};
            const SCPoint3d v5c{w, 0.0, s};

            // v6 闂?(w,s,s): top + back + right
            const SCPoint3d v6a{w, s, s + 1.0e-7};
            const SCPoint3d v6b{w + 1.0e-7, s, s};
            const SCPoint3d v6c{w, s + 1.0e-7, s};

            // v7 闂?(0,s,s): top + back + left
            const SCPoint3d v7a{0.0, s, s};
            const SCPoint3d v7b{-1.0e-7, s, s};
            const SCPoint3d v7c{0.0, s, s + 1.0e-7};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({// Bottom (z闂?, normal -z): v0, v3, v2, v1
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3a, v2a, v1a})),
                                   // Top (z闂備焦鍓氶崰鏍ь熆? normal +z): v4, v5, v6, v7
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4a, v5a, v6a, v7a})),
                                   // Front (y闂?, normal -y): v0, v1, v5, v4
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5b, v4b})),
                                   // Back (y闂備焦鍓氶崰鏍ь熆? normal +y): v3, v7, v6, v2
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3b, v7b, v6b, v2b})),
                                   // Left (x闂?, normal -x): v0, v4, v7, v3
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4c, v7c, v3c})),
                                   // Right (x闂備焦鍓氶崰鏍ь熆? normal +x): v1, v2, v6, v5
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2c, v6c, v5c}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody()
        {
            // Reuse the all-vertices near-equal closed-cuboid and inject a
            // consecutive duplicate leading vertex into one face loop.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidAllVerticesBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d front = faces[2];
            std::vector<SCPoint3d> loop = front.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[2] = PolyhedronFace3d(front.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate variant and add another duplicate
            // leading vertex on a second face, to stress duplicate-loop
            // normalization composition in a closed all-vertices near-equal cuboid
            // input.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidAllVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d back = faces[3];
            std::vector<SCPoint3d> loop = back.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody()
        {
            // Tiny-scale 2x1x1 closed cuboid where two non-adjacent shared vertices
            // (v0 and v6) have near-equal variants across their 3 incident faces,
            // and one face also needs duplicate-loop normalization.
            const double s = 1e-5;
            const double w = 2.0 * s;

            // v0 variants: bottom/front/left
            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            // Stable vertices.
            const SCPoint3d v1{w, 0.0, 0.0};
            const SCPoint3d v2{w, s, 0.0};
            const SCPoint3d v3{0.0, s, 0.0};
            const SCPoint3d v4{0.0, 0.0, s};
            const SCPoint3d v5{w, 0.0, s};
            const SCPoint3d v7{0.0, s, s};

            // v6 variants: top/back/right
            const SCPoint3d v6a{w + 2.0e-7, s, s};
            const SCPoint3d v6b{w - 1.0e-7, s, s};
            const SCPoint3d v6c{w, s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            std::vector<SCPoint3d> leftLoop{v0c, v4, v7, v3};
            leftLoop.insert(leftLoop.begin(), leftLoop.front());

            return PolyhedronBody({// Bottom
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1})),
                                   // Top
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
                                   // Front
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v5, v4})),
                                   // Back
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
                                   // Left (with duplicate leading vertex)
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d(std::move(leftLoop))),
                                   // Right
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v6c, v5}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate dual-vertices variant and inject a
            // second duplicate leading vertex on another face.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidDualVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d back = faces[3];
            std::vector<SCPoint3d> loop = back.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidDualVerticesBody()
        {
            // Same as the dual-vertices+duplicate case but without duplicate loop
            // normalization input, to isolate representative-average behavior.
            const double s = 1e-5;
            const double w = 2.0 * s;

            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            const SCPoint3d v1{w, 0.0, 0.0};
            const SCPoint3d v2{w, s, 0.0};
            const SCPoint3d v3{0.0, s, 0.0};
            const SCPoint3d v4{0.0, 0.0, s};
            const SCPoint3d v5{w, 0.0, s};
            const SCPoint3d v7{0.0, s, s};

            const SCPoint3d v6a{w + 2.0e-7, s, s};
            const SCPoint3d v6b{w - 1.0e-7, s, s};
            const SCPoint3d v6c{w, s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1, v5, v4})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4, v7, v3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v2, v6c, v5}))});
        }

        // Three shared vertices with near-equal perturbations: v0, v1, v6.
        // Extends dual-vertices coverage to an intermediate triple-vertex case.
        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesBody()
        {
            const double s = 1e-5;
            const double w = 2.0 * s;

            // v0 cluster (3 faces share this corner)
            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            // v1 cluster (3 faces share this corner)
            const SCPoint3d v1a{w + 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1b{w - 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1c{w, 0.0, 0.0};

            // v6 cluster (3 faces share this corner, diagonal to v0)
            const SCPoint3d v6a{w + 2.0e-7, s, s};
            const SCPoint3d v6b{w - 1.0e-7, s, s};
            const SCPoint3d v6c{w, s, s};

            const SCPoint3d v2{w, s, 0.0};
            const SCPoint3d v3{0.0, s, 0.0};
            const SCPoint3d v4{0.0, 0.0, s};
            const SCPoint3d v5{w, 0.0, s};
            const SCPoint3d v7{0.0, s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1a})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5, v4})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v4, v7, v3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2, v6c, v5}))});
        }

        // Same triple-vertices near-equal perturbation, but the left face also has
        // a duplicate leading vertex injected for duplicate-loop-normalization
        // stress.
        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody()
        {
            const double s = 1e-5;
            const double w = 2.0 * s;

            const SCPoint3d v0a{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0b{-1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{0.0, 0.0, 0.0};

            const SCPoint3d v1a{w + 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1b{w - 1.0e-7, 0.0, 0.0};
            const SCPoint3d v1c{w, 0.0, 0.0};

            const SCPoint3d v6a{w + 2.0e-7, s, s};
            const SCPoint3d v6b{w - 1.0e-7, s, s};
            const SCPoint3d v6c{w, s, s};

            const SCPoint3d v2{w, s, 0.0};
            const SCPoint3d v3{0.0, s, 0.0};
            const SCPoint3d v4{0.0, 0.0, s};
            const SCPoint3d v5{w, 0.0, s};
            const SCPoint3d v7{0.0, s, s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            // Left face with duplicate leading vertex
            std::vector<SCPoint3d> leftLoop{v0c, v4, v7, v3};
            leftLoop.insert(leftLoop.begin(), leftLoop.front());

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v3, v2, v1a})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v4, v5, v6a, v7})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v1b, v5, v4})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v3, v7, v6b, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d(std::move(leftLoop))),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1c, v2, v6c, v5}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDualDuplicateLoopBody()
        {
            // Start from the single-duplicate triple-vertices variant and add a
            // second duplicate leading vertex on another face to stress composition
            // behavior.
            const PolyhedronBody base = BuildSupportMismatchNearEqualClosedCuboidTripleVerticesWithDuplicateLoopBody();
            std::vector<PolyhedronFace3d> faces = base.Faces();
            if (faces.size() != 6)
            {
                return PolyhedronBody();
            }

            const PolyhedronFace3d back = faces[3];
            std::vector<SCPoint3d> loop = back.OuterLoop().Vertices();
            if (loop.empty())
            {
                return PolyhedronBody();
            }

            loop.insert(loop.begin(), loop.front());
            faces[3] = PolyhedronFace3d(back.SupportPlane(), PolyhedronLoop3d(std::move(loop)));
            return PolyhedronBody(std::move(faces));
        }

        void AssertClosedCuboidAllVerticesRepresentativeTargets(const SCBrepBody& body)
        {
            const double s = 1e-5;
            const double w = 2.0 * s;
            const double third = 1.0e-7 / 3.0;
            const std::vector<SCPoint3d> expectedPoints{SCPoint3d{0.0, 0.0, 0.0},
                                                      SCPoint3d{w + 2.0 * third, 0.0, 0.0},
                                                      SCPoint3d{w + third, s + third, 0.0},
                                                      SCPoint3d{-third, s + 2.0 * third, 0.0},
                                                      SCPoint3d{third, 0.0, s + third},
                                                      SCPoint3d{w + third, 0.0, s - third},
                                                      SCPoint3d{w + third, s + third, s + third},
                                                      SCPoint3d{-third, s, s + third}};
            std::vector<std::size_t> expectedMatchCounts(expectedPoints.size(), 0);

            ASSERT_EQ(body.VertexCount(), expectedPoints.size());
            for (std::size_t i = 0; i < body.VertexCount(); ++i)
            {
                const SCPoint3d point = body.VertexAt(i).Point();
                bool matched = false;
                for (std::size_t expectedIndex = 0; expectedIndex < expectedPoints.size(); ++expectedIndex)
                {
                    const SCPoint3d& expected = expectedPoints[expectedIndex];
                    if (std::abs(point.x - expected.x) < 1e-10 && std::abs(point.y - expected.y) < 1e-10 &&
                        std::abs(point.z - expected.z) < 1e-10)
                    {
                        ++expectedMatchCounts[expectedIndex];
                        matched = true;
                        break;
                    }
                }

                ASSERT_TRUE(matched);
            }

            for (const std::size_t count : expectedMatchCounts)
            {
                ASSERT_EQ(count, 1);
            }
        }
    }  // namespace

}  // namespace Geometry::Test::ConversionFixtures
