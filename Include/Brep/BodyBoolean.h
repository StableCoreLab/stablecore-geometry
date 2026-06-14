#pragma once

#include <string>
#include <vector>

#include "Brep/SCBrepBody.h"
#include "Brep/PolyhedronBody.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"

namespace Geometry
{
    enum class BodyBooleanIssue3d
    {
        None,
        InvalidInput,
        UnsupportedOperation
    };

    struct GEOMETRY_API BodyBooleanOptions3d
    {
        SCGeometryTolerance3d tolerance{};
        bool operateOnShells{false};
        bool preserveInputOrientation{true};
    };

    struct GEOMETRY_API BodyBooleanResult3d
    {
        BodyBooleanIssue3d issue{BodyBooleanIssue3d::None};
        SCBrepBody body{};
        std::vector<SCBrepBody> bodies{};
        bool producedEmptyResult{false};
        std::string message{};

        [[nodiscard]] bool IsSuccess() const
        {
            return issue == BodyBooleanIssue3d::None &&
                   (body.FaceCount() > 0 || !bodies.empty() || producedEmptyResult);
        }
    };

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d IntersectBodies(const SCBrepBody& first,
                                                                   const SCBrepBody& second,
                                                                   BodyBooleanOptions3d options = {});

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d UnionBodies(const SCBrepBody& first,
                                                               const SCBrepBody& second,
                                                               BodyBooleanOptions3d options = {});

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d DifferenceBodies(const SCBrepBody& first,
                                                                    const SCBrepBody& second,
                                                                    BodyBooleanOptions3d options = {});

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d IntersectBodies(const PolyhedronBody& first,
                                                                   const PolyhedronBody& second,
                                                                   BodyBooleanOptions3d options = {});

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d UnionBodies(const PolyhedronBody& first,
                                                               const PolyhedronBody& second,
                                                               BodyBooleanOptions3d options = {});

    [[nodiscard]] GEOMETRY_API BodyBooleanResult3d DifferenceBodies(const PolyhedronBody& first,
                                                                    const PolyhedronBody& second,
                                                                    BodyBooleanOptions3d options = {});
}  // namespace Geometry
