#pragma once

#include "Support/ConversionFixturesBasic3d.h"

namespace Geometry::Test::ConversionFixtures
{
    namespace
    {
        PolyhedronBody BuildTinyScaleNonPlanarRepairBody()
        {
            const double s = 1e-5;
            std::vector<SCPoint3d> outer{
                SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{s, 0.0, 0.0}, SCPoint3d{s, s, 0.0}, SCPoint3d{0.0, s, 0.0}};
            // Small-scale non-planar disturbance that still describes a recoverable
            // loop.
            outer[2].z += 2e-6;

            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 3e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d(std::move(outer)));
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildTinyScaleNonPlanarHoledRepairBody()
        {
            const double s = 1e-5;
            std::vector<SCPoint3d> outer{
                SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{s, 0.0, 0.0}, SCPoint3d{s, s, 0.0}, SCPoint3d{0.0, s, 0.0}};
            std::vector<SCPoint3d> hole{SCPoint3d{0.3 * s, 0.3 * s, 0.0},
                                      SCPoint3d{0.7 * s, 0.3 * s, 0.0},
                                      SCPoint3d{0.7 * s, 0.7 * s, 0.0},
                                      SCPoint3d{0.3 * s, 0.7 * s, 0.0}};

            outer[2].z += 2e-6;
            hole[1].z += 1.5e-6;

            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                        PolyhedronLoop3d(std::move(outer)),
                                        {PolyhedronLoop3d(std::move(hole))});
            return PolyhedronBody({face});
        }

        PolyhedronBody BuildTinyScaleNonPlanarMultiFaceRepairBody()
        {
            const double s = 1e-5;
            std::vector<SCPoint3d> faceA{
                SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{s, 0.0, 0.0}, SCPoint3d{s, s, 0.0}, SCPoint3d{0.0, s, 0.0}};
            std::vector<SCPoint3d> faceB{
                SCPoint3d{2.0 * s, 0.0, 0.0}, SCPoint3d{3.0 * s, 0.0, 0.0}, SCPoint3d{3.0 * s, s, 0.0}, SCPoint3d{2.0 * s, s, 0.0}};

            faceA[2].z += 2e-6;
            faceB[1].z += 1.4e-6;

            return PolyhedronBody(
                {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                  PolyhedronLoop3d(std::move(faceA))),
                 PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                  PolyhedronLoop3d(std::move(faceB)))});
        }

        PolyhedronBody BuildTinyScaleNonPlanarMixedContentBody()
        {
            const double s = 1e-5;
            std::vector<SCPoint3d> outer{
                SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{s, 0.0, 0.0}, SCPoint3d{s, s, 0.0}, SCPoint3d{0.0, s, 0.0}};
            std::vector<SCPoint3d> hole{SCPoint3d{0.3 * s, 0.3 * s, 0.0},
                                      SCPoint3d{0.7 * s, 0.3 * s, 0.0},
                                      SCPoint3d{0.7 * s, 0.7 * s, 0.0},
                                      SCPoint3d{0.3 * s, 0.7 * s, 0.0}};
            std::vector<SCPoint3d> plain{
                SCPoint3d{2.0 * s, 0.0, 0.0}, SCPoint3d{3.0 * s, 0.0, 0.0}, SCPoint3d{3.0 * s, s, 0.0}, SCPoint3d{2.0 * s, s, 0.0}};

            outer[2].z += 2e-6;
            hole[1].z += 1.3e-6;
            plain[0].z += 1.1e-6;

            return PolyhedronBody(
                {PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                  PolyhedronLoop3d(std::move(outer)),
                                  {PolyhedronLoop3d(std::move(hole))}),
                 PolyhedronFace3d(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                  PolyhedronLoop3d(std::move(plain)))});
        }

        PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.8e-6};
            const SCPoint3d e{2.0 * s, 0.0, 1.4e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};

            // Face A and Face B share edge (b, c), while each has tiny non-planar
            // drift.
            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}));
            return PolyhedronBody({faceA, faceB});
        }

        PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};
            const SCPoint3d e{2.0 * s, 0.0, 1.1e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.4e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            // Face A shares (b,c) with face B; face B shares (e,f) with face C.
            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}));
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSupportMismatchSharedEdgeChainBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};
            const SCPoint3d e{2.0 * s, 0.0, 1.1e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.4e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(mismatchedPlane, PolyhedronLoop3d({b, e, f, c}));
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSupportMismatchSharedEdgeChainWithDuplicateBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};
            const SCPoint3d e{2.0 * s, 0.0, 1.1e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.4e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0});

            const PolyhedronFace3d faceA(mismatchedPlane, PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(mismatchedPlane, PolyhedronLoop3d({b, b, e, f, c}));
            const PolyhedronFace3d faceC(mismatchedPlane, PolyhedronLoop3d({e, g, h, f}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainMixedContentBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};

            const SCPoint3d e{2.0 * s, 0.0, 1.1e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};

            const SCPoint3d g{3.0 * s, 0.0, 1.3e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 1.0e-6},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            // Face A shares (b,c) with face B; face B shares (e,f) with face C.
            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));
            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleNonPlanarSharedEdgeChainWithDuplicateLoopBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.4e-6};

            const SCPoint3d e{2.0 * s, 0.0, 1.2e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};

            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            // Duplicate vertex in middle face loop to exercise normalization +
            // refit.
            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, e, f, c}));
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedEdgeChainMixedContentWithDuplicateHoleBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s, 0.0, 1.2e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};

            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 1.0e-6},
                                      SCPoint3d{1.8 * s, 0.3 * s, 1.0e-6},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainMixedContentWithCollinearLeadingBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};

            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 1.3e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            // Leading points b -> m -> e are collinear to exercise robust normal
            // fallback while the loop remains mildly non-planar at tiny scale.
            const SCPoint3d m{1.5 * s, 0.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 1.0e-6},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, m, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainMixedContentSupportPlaneMismatchBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s, 0.0, 1.2e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.0e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            // Deliberately offset support planes to require refit in combination
            // with shared-chain mixed-content tiny-scale projection repair.
            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainMixedContentSupportMismatchWithDuplicateHoleBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s, 0.0, 1.2e-6};
            const SCPoint3d f{2.0 * s, s, 0.0};
            const SCPoint3d g{3.0 * s, 0.0, 1.0e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainSupportMismatchAndCollinearBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 1.2e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};
            const SCPoint3d m{1.5 * s, 0.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, m, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainFullCompositionBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.5e-6};

            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 1.2e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.1e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};
            const SCPoint3d m{1.5 * s, 0.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, m, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleSharedChainDualDuplicateFullCompositionBody()
        {
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 0.0};
            const SCPoint3d c{s, s, 0.0};
            const SCPoint3d d{0.0, s, 1.6e-6};

            const SCPoint3d e{2.0 * s, 0.0, 0.0};
            const SCPoint3d f{2.0 * s, s, 1.2e-6};
            const SCPoint3d g{3.0 * s, 0.0, 1.0e-6};
            const SCPoint3d h{3.0 * s, s, 0.0};
            const SCPoint3d m{1.5 * s, 0.0, 0.0};

            std::vector<SCPoint3d> hole{SCPoint3d{1.2 * s, 0.3 * s, 0.0},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.3 * s, 8e-7},
                                      SCPoint3d{1.8 * s, 0.7 * s, 0.0},
                                      SCPoint3d{1.2 * s, 0.7 * s, 0.0}};

            const PolyhedronFace3d faceA(SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({a, b, b, c, d}));
            const PolyhedronFace3d faceB(SCPlane::FromPointAndNormal(SCPoint3d{s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({b, m, e, f, c}),
                                         {PolyhedronLoop3d(std::move(hole))});
            const PolyhedronFace3d faceC(SCPlane::FromPointAndNormal(SCPoint3d{2.0 * s, 0.0, 4e-6}, SCVector3d{0.0, 0.0, 1.0}),
                                         PolyhedronLoop3d({e, g, h, f}));

            return PolyhedronBody({faceA, faceB, faceC});
        }

        PolyhedronBody BuildTinyScaleClosedTetrahedronBody()
        {
            // A tiny-scale tetrahedron (4 triangular faces, 6 edges, 4 vertices)
            // where every support plane is identical and mismatched to force
            // per-face refit. All 6 edges are each shared by exactly two faces, so
            // after repair the resulting SCBrepBody should form a deterministically
            // closed shell.
            const double s = 1e-5;
            const SCPoint3d v0{0.0, 0.0, 0.0};
            const SCPoint3d v1{s, 0.0, 1.2e-6};
            const SCPoint3d v2{0.5 * s, 0.866 * s, 0.8e-6};
            const SCPoint3d v3{0.5 * s, 0.289 * s, 0.816 * s};

            // Deliberately flat support plane (z=2e-6, normal +z) while actual face
            // normals vary 闂?the per-face refit must compute the correct
            // orientation.
            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0});

            const PolyhedronFace3d f0(mismatchedPlane, PolyhedronLoop3d({v0, v1, v2}));
            const PolyhedronFace3d f1(mismatchedPlane, PolyhedronLoop3d({v0, v3, v1}));
            const PolyhedronFace3d f2(mismatchedPlane, PolyhedronLoop3d({v1, v3, v2}));
            const PolyhedronFace3d f3(mismatchedPlane, PolyhedronLoop3d({v0, v2, v3}));

            return PolyhedronBody({f0, f1, f2, f3});
        }

        PolyhedronBody BuildTinyScaleTriangularFaceChainBody()
        {
            // Three triangular faces sharing edges in a strip:
            //   T1=(a,b,c), T2=(b,d,c), T3=(c,d,e)
            //   T1/T2 share edge b-c; T2/T3 share edge c-d (traversed reversed).
            // Key property: every shared vertex is one of the 3 defining vertices
            // of EVERY triangle that contains it, so after per-face refit its
            // distance to that face's refit-plane is exactly 0 闂?no projection
            // occurs and the original 3D position is preserved unchanged.
            // FindOrAddBrepVertex therefore merges all shared vertices exactly,
            // giving VertexCount=5, EdgeCount=7.
            const double s = 1e-5;
            const SCPoint3d a{0.0, 0.0, 0.0};
            const SCPoint3d b{s, 0.0, 1.3e-6};
            const SCPoint3d c{2.0 * s, 0.0, 0.9e-6};
            const SCPoint3d d{1.5 * s, s, 1.1e-6};
            const SCPoint3d e{3.0 * s, 0.5 * s, 0.7e-6};

            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({a, b, c})),
                                   PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({b, d, c})),
                                   PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({c, d, e}))});
        }

        PolyhedronBody BuildTinyScaleTriangularFanBody()
        {
            // Four triangular faces sharing a common apex vertex, arranged like a
            // square pyramid cap (open base). All support planes mismatched.
            //   T0=(apex,v0,v1), T1=(apex,v1,v2), T2=(apex,v2,v3), T3=(apex,v3,v0)
            // Each radial edge (apex-vk) is shared by exactly two triangles, the
            // four outer edges (vk-v(k+1 mod 4)) are each used once. Key: apex is
            // one of the 3 defining vertices of all 4 triangles 闂?after per-face
            // refit its distance to each refit-plane is exactly 0 闂?apex is never
            // projected 闂?it is preserved at its original position in all faces
            // 闂?FindOrAddBrepVertex merges it exactly once.
            // Expected: VertexCount=5, EdgeCount=8 (4 radial + 4 outer).
            const double s = 1e-5;
            const SCPoint3d apex{0.0, 0.0, s};
            const SCPoint3d v0{s, 0.0, 0.0};
            const SCPoint3d v1{0.0, s, 0.0};
            const SCPoint3d v2{-s, 0.0, 0.0};
            const SCPoint3d v3{0.0, -s, 0.0};

            const SCPlane mismatchedPlane = SCPlane::FromPointAndNormal(SCPoint3d{0.0, 0.0, 2e-6}, SCVector3d{0.0, 0.0, 1.0});

            return PolyhedronBody({PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v0, v1})),
                                   PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v1, v2})),
                                   PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v2, v3})),
                                   PolyhedronFace3d(mismatchedPlane, PolyhedronLoop3d({apex, v3, v0}))});
        }

    }  // namespace

}  // namespace Geometry::Test::ConversionFixtures
