#pragma once

#include "export/GeometryExport.h"
#include "sdk/Polygon2d.h"
#include "sdk/PolyhedronBody.h"
#include "sdk/Polyline2d.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
{
enum class PolygonValidationIssue2d
{
    None,
    TooFewPoints,
    NotClosed,
    InvalidOrientation,
    SelfIntersection
};

struct GEOMETRY_API PolygonValidation2d
{
    bool valid{false};
    PolygonValidationIssue2d issue{PolygonValidationIssue2d::None};
};

enum class MeshValidationIssue3d
{
    None,
    InvalidVertex,
    InvalidIndex,
    DuplicateIndex,
    DegenerateTriangle
};

struct GEOMETRY_API MeshValidation3d
{
    bool valid{false};
    MeshValidationIssue3d issue{MeshValidationIssue3d::None};
    std::size_t elementIndex{0};
};

enum class PolyhedronValidationIssue3d
{
    None,
    EmptyBody,
    InvalidFace
};

struct GEOMETRY_API PolyhedronValidation3d
{
    bool valid{false};
    PolyhedronValidationIssue3d issue{PolyhedronValidationIssue3d::None};
    std::size_t faceIndex{0};
};

[[nodiscard]] GEOMETRY_API bool HasSelfIntersection(const Polyline2d& ring, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API PolygonValidation2d Validate(const Polyline2d& ring, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API PolygonValidation2d Validate(const Polygon2d& polygon, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API MeshValidation3d Validate(const TriangleMesh& mesh, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API PolyhedronValidation3d Validate(const PolyhedronBody& body, double eps = 1e-9);
} // namespace geometry::sdk
