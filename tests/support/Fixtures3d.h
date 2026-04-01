#pragma once

#include "sdk/Geometry.h"

namespace geometry::test
{
inline geometry::sdk::PolyhedronBody BuildUnitCubeBody()
{
    using geometry::sdk::Plane;
    using geometry::sdk::Point3d;
    using geometry::sdk::PolyhedronBody;
    using geometry::sdk::PolyhedronFace3d;
    using geometry::sdk::PolyhedronLoop3d;
    using geometry::sdk::Vector3d;

    return PolyhedronBody(
        {
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, 0.0, -1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{0.0, 1.0, 0.0},
                        Point3d{1.0, 1.0, 0.0},
                        Point3d{1.0, 0.0, 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 1.0}, Vector3d{0.0, 0.0, 1.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 1.0},
                        Point3d{1.0, 0.0, 1.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{0.0, 1.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{0.0, -1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{1.0, 0.0, 0.0},
                        Point3d{1.0, 0.0, 1.0},
                        Point3d{0.0, 0.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{1.0, 0.0, 0.0}, Vector3d{1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{1.0, 0.0, 0.0},
                        Point3d{1.0, 1.0, 0.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{1.0, 0.0, 1.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 1.0, 0.0}, Vector3d{0.0, 1.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 1.0, 0.0},
                        Point3d{0.0, 1.0, 1.0},
                        Point3d{1.0, 1.0, 1.0},
                        Point3d{1.0, 1.0, 0.0},
                    })),
            PolyhedronFace3d(
                Plane::FromPointAndNormal(Point3d{0.0, 0.0, 0.0}, Vector3d{-1.0, 0.0, 0.0}),
                PolyhedronLoop3d(
                    {
                        Point3d{0.0, 0.0, 0.0},
                        Point3d{0.0, 0.0, 1.0},
                        Point3d{0.0, 1.0, 1.0},
                        Point3d{0.0, 1.0, 0.0},
                    })),
        });
}
} // namespace geometry::test
