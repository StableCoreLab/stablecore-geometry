#pragma once
#include "Core/GeometryTypes.h"
#include "Brep/SCBrepBody.h"
#include "Brep/PolyhedronBody.h"
#include "Core/Section.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    enum class SCPolygonValidationIssue2d
    {
        None,
        TooFewPoints,
        NotClosed,
        InvalidOrientation,
        SelfIntersection
    };

    struct GEOMETRY_API SCPolygonValidation2d
    {
        bool valid{false};
        SCPolygonValidationIssue2d issue{SCPolygonValidationIssue2d::None};
    };

    enum class SCMeshValidationIssue3d
    {
        None,
        InvalidVertex,
        InvalidIndex,
        DuplicateIndex,
        DegenerateTriangle
    };

    struct GEOMETRY_API SCMeshValidation3d
    {
        bool valid{false};
        SCMeshValidationIssue3d issue{SCMeshValidationIssue3d::None};
        std::size_t elementIndex{0};
    };

    enum class SCPolyhedronValidationIssue3d
    {
        None,
        EmptyBody,
        InvalidFace
    };

    struct GEOMETRY_API SCPolyhedronValidation3d
    {
        bool valid{false};
        SCPolyhedronValidationIssue3d issue{SCPolyhedronValidationIssue3d::None};
        std::size_t faceIndex{0};
    };

    enum class SCBrepValidationIssue3d
    {
        None,
        EmptyBody,
        InvalidVertex,
        InvalidEdge,
        InvalidShell,
        InvalidFaceAdjacency
    };

    struct GEOMETRY_API SCBrepValidation3d
    {
        bool valid{false};
        SCBrepValidationIssue3d issue{SCBrepValidationIssue3d::None};
        std::size_t elementIndex{0};
    };

    enum class SCSectionValidationIssue3d
    {
        None,
        InvalidSectionState,
        InvalidBasis,
        InvalidSegment,
        InvalidContour,
        InvalidPolygon
    };

    struct GEOMETRY_API SCSectionValidation3d
    {
        bool valid{false};
        SCSectionValidationIssue3d issue{SCSectionValidationIssue3d::None};
        std::size_t elementIndex{0};
    };

    [[nodiscard]] GEOMETRY_API bool HasSelfIntersection(const SCPolyline2d& ring, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolygonValidation2d Validate(const SCPolyline2d& ring,
                                                            double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolygonValidation2d Validate(const SCPolygon2d& polygon,
                                                            double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCMeshValidation3d Validate(const TriangleMesh& mesh,
                                                         double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolyhedronValidation3d Validate(const PolyhedronBody& body,
                                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCBrepValidation3d Validate(const SCBrepBody& body,
                                                         const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCSectionValidation3d Validate(const SCPolyhedronSection3d& section,
                                                            double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry


