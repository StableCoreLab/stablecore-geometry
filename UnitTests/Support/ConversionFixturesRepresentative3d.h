#pragma once

#include "Support/ConversionFixturesBasic3d.h"

namespace Geometry::Test::ConversionFixtures
{
    namespace
    {
        PolyhedronBody BuildNearEqualSharedEdgeQuadPairBody()
        {
            // Two planar quads share one edge geometrically, but one face stores
            // near-equal (<eps) perturbed coordinates for shared vertices.
            // Conversion should keep shared topology and place shared Brep vertices
            // at representative-id global average points instead of first-face
            // points.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 0.0};

            const SCPoint3d b2{s + dx, 0.0, 0.0};
            const SCPoint3d c2{s + dx, s, 0.0};
            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 0.0};

            const SCPlane plane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 0.0}, SCVector3d{0.0, 0.0, 1.0});
            return PolyhedronBody({PolyhedronFace3d(plane, PolyhedronLoop3d({a, b, c, d})),
                                   PolyhedronFace3d(plane, PolyhedronLoop3d({b2, e, f, c2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeQuadPairBody()
        {
            // Same near-equal shared-edge setup as
            // BuildNearEqualSharedEdgeQuadPairBody, but with intentionally
            // mismatched support planes to force repair path.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 0.0};

            const SCPoint3d b2{s + dx, 0.0, 0.0};
            const SCPoint3d c2{s + dx, s, 0.0};
            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 0.0};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});
            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, e, f, c2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualApexTriangularFanBody()
        {
            // Four triangles share a near-equal apex (<eps perturbations), with
            // mismatched support planes forcing repair before Brep conversion.
            // Representative-id averaging should converge the shared apex to the
            // cross-face average target point.
            const double s = 1e-5;
            const SCPoint3d apex0{0.0, 0.0, s};
            const SCPoint3d apex1{1.0e-7, 0.0, s};
            const SCPoint3d apex2{2.0e-7, 0.0, s};
            const SCPoint3d apex3{-1.0e-7, 0.0, s};

            const SCPoint3d v0{s, 0.0, 0.0};
            const SCPoint3d v1{0.0, s, 0.0};
            const SCPoint3d v2{-s, 0.0, 0.0};
            const SCPoint3d v3{0.0, -s, 0.0};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex0, v0, v1})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex1, v1, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex2, v2, v3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({apex3, v3, v0}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeChainBody()
        {
            // Three quads in a shared-edge chain with near-equal (<eps)
            // perturbations on middle-face shared vertices. Mismatched support
            // planes force repair.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 0.0};

            const SCPoint3d b2{s + dx, 0.0, 0.0};
            const SCPoint3d c2{s + dx, s, 0.0};
            const SCPoint3d e{2.0 * s + dx, 0.0, 0.0};
            const SCPoint3d f{2.0 * s + dx, s, 0.0};

            const SCPoint3d e2{2.0 * s, 0.0, 0.0};
            const SCPoint3d f2{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 0.0};
            const SCPoint3d h{3.0 * s, s, 0.0};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, e, f, c2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedCornerQuadFanBody()
        {
            // Three quads only share one near-equal corner (<eps), without shared
            // edges. Mismatched support planes force conversion repair.
            const double s = 1e-5;

            const SCPoint3d p0a{0.0, 0.0, 0.0};
            const SCPoint3d p0b{1.0e-7, 0.0, 0.0};
            const SCPoint3d p0c{-1.0e-7, 0.0, 0.0};

            const SCPoint3d a1{s, 0.0, 0.0};
            const SCPoint3d a2{s, s, 0.0};
            const SCPoint3d a3{0.0, s, 0.0};

            const SCPoint3d b1{-s, 0.0, 0.0};
            const SCPoint3d b2{-s, s, 0.0};
            const SCPoint3d b3{0.0, s, 0.0};

            const SCPoint3d c1{0.0, -s, 0.0};
            const SCPoint3d c2{s, -s, 0.0};
            const SCPoint3d c3{s, 0.0, 0.0};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0a, a1, a2, a3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0b, b1, b2, b3})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({p0c, c1, c2, c3}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualClosedTetrahedronBody()
        {
            // Closed tetrahedron where one logical vertex appears with near-equal
            // (<eps) perturbations across faces. Mismatched support planes force
            // repair before conversion. Representative averaging should merge the
            // perturbed variants into one shared Brep vertex.
            const double s = 1e-5;
            const SCPoint3d v0a{0.0, 0.0, 0.0};
            const SCPoint3d v0b{1.0e-7, 0.0, 0.0};
            const SCPoint3d v0c{-1.0e-7, 0.0, 0.0};

            const SCPoint3d v1{s, 0.0, 1.2e-6};
            const SCPoint3d v2{0.5 * s, 0.866 * s, 0.8e-6};
            const SCPoint3d v3{0.5 * s, 0.289 * s, 0.816 * s};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0a, v1, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0b, v3, v1})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v1, v3, v2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({v0c, v2, v3}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedEdgeChainWithDuplicateBody()
        {
            // Three-face shared-edge chain with near-equal perturbation on shared
            // vertices and duplicate leading vertex in middle face to force loop
            // normalization together with representative-average placement.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 0.0};

            const SCPoint3d b2{s + dx, 0.0, 0.0};
            const SCPoint3d c2{s + dx, s, 0.0};
            const SCPoint3d e{2.0 * s + dx, 0.0, 0.0};
            const SCPoint3d f{2.0 * s + dx, s, 0.0};

            const SCPoint3d e2{2.0 * s, 0.0, 0.0};
            const SCPoint3d f2{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 0.0};
            const SCPoint3d h{3.0 * s, s, 0.0};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({b2, b2, e, f, c2})),
                                   PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainMixedContentWithDuplicateHoleBody()
        {
            // Shared-edge chain mixed-content variant with near-equal perturbations
            // on shared vertices and duplicate hole vertices on middle face.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s + dx, 0.0, 1.2e-6};
            const SCPoint3d f{2.0 * s + dx, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.0e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            const SCPoint3d e2{2.0 * s, 0.0, 1.2e-6};
            const SCPoint3d f2{2.0 * s, s, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody(
                {PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
                 PolyhedronFace3d(mismatched,
                                  PolyhedronLoop3d({b + SCVector3d{dx, 0.0, 0.0}, e, f, c + SCVector3d{dx, 0.0, 0.0}}),
                                  {PolyhedronLoop3d(std::move(hole))}),
                 PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainFullCompositionBody()
        {
            // Shared-chain full composition: support-plane mismatch +
            // collinear-leading
            // + duplicate-hole normalization + near-equal shared-edge
            // perturbations.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s + dx, 0.0, 0.0};
            const SCPoint3d f{2.0 * s + dx, s, 1.2e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};
            const SCPoint3d m{1.5 * s + dx, 0.0, 0.0};

            const SCPoint3d e2{2.0 * s, 0.0, 0.0};
            const SCPoint3d f2{2.0 * s, s, 1.2e-6};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody(
                {PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, c, d})),
                 PolyhedronFace3d(mismatched,
                                  PolyhedronLoop3d({b + SCVector3d{dx, 0.0, 0.0}, m, e, f, c + SCVector3d{dx, 0.0, 0.0}}),
                                  {PolyhedronLoop3d(std::move(hole))}),
                 PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
        }

        PolyhedronBody BuildSupportMismatchNearEqualSharedChainDualDuplicateFullCompositionBody()
        {
            // Shared-chain full composition with both outer and hole duplicate
            // normalization + near-equal shared-edge perturbations.
            const double s = 1e-5;
            const double dx = 2e-7;

            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};

            const SCPoint3d e{2.0 * s + dx, 0.0, 0.0};
            const SCPoint3d f{2.0 * s + dx, s, 1.2e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.0e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};
            const SCPoint3d m{1.5 * s + dx, 0.0, 0.0};

            const SCPoint3d e2{2.0 * s, 0.0, 0.0};
            const SCPoint3d f2{2.0 * s, s, 1.2e-6};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const SCPlane mismatched = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody(
                {PolyhedronFace3d(mismatched, PolyhedronLoop3d({a, b, b, c, d})),
                 PolyhedronFace3d(mismatched,
                                  PolyhedronLoop3d({b + SCVector3d{dx, 0.0, 0.0}, m, e, f, c + SCVector3d{dx, 0.0, 0.0}}),
                                  {PolyhedronLoop3d(std::move(hole))}),
                 PolyhedronFace3d(mismatched, PolyhedronLoop3d({e2, g, h, f2}))});
        }

    }  // namespace

}  // namespace Geometry::Test::ConversionFixtures
