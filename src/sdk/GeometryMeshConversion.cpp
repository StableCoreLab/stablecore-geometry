#include "sdk/GeometryMeshConversion.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace geometry::sdk
{
namespace
{
struct Basis2dOnPlane
{
    Vector3d u{};
    Vector3d v{};
};

[[nodiscard]] Basis2dOnPlane BuildPlaneBasis(const Plane& plane, double eps)
{
    const Vector3d normal = plane.UnitNormal(eps);
    const Vector3d axis = std::abs(normal.x) <= std::abs(normal.y) &&
                                  std::abs(normal.x) <= std::abs(normal.z)
                              ? Vector3d{1.0, 0.0, 0.0}
                              : (std::abs(normal.y) <= std::abs(normal.z)
                                     ? Vector3d{0.0, 1.0, 0.0}
                                     : Vector3d{0.0, 0.0, 1.0});
    const Vector3d u = Cross(normal, axis).Normalized(eps);
    const Vector3d v = Cross(normal, u).Normalized(eps);
    return {u, v};
}

[[nodiscard]] Point2d ProjectToPlane2d(const Point3d& point, const Plane& plane, const Basis2dOnPlane& basis)
{
    const Vector3d delta = point - plane.origin;
    return Point2d{Dot(delta, basis.u), Dot(delta, basis.v)};
}

[[nodiscard]] double SignedArea2d(const std::vector<Point2d>& points)
{
    double area = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const Point2d& a = points[i];
        const Point2d& b = points[(i + 1) % points.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return 0.5 * area;
}

[[nodiscard]] bool PointInTriangle2d(
    const Point2d& point,
    const Point2d& a,
    const Point2d& b,
    const Point2d& c,
    double eps)
{
    const double cross0 = Cross(b - a, point - a);
    const double cross1 = Cross(c - b, point - b);
    const double cross2 = Cross(a - c, point - c);
    const bool hasNegative = cross0 < -eps || cross1 < -eps || cross2 < -eps;
    const bool hasPositive = cross0 > eps || cross1 > eps || cross2 > eps;
    return !(hasNegative && hasPositive);
}

[[nodiscard]] bool IsEar(
    const std::vector<std::size_t>& polygon,
    std::size_t index,
    const std::vector<Point2d>& projected,
    double orientationSign,
    double eps)
{
    const std::size_t count = polygon.size();
    const std::size_t previous = polygon[(index + count - 1) % count];
    const std::size_t current = polygon[index];
    const std::size_t next = polygon[(index + 1) % count];
    const Point2d& a = projected[previous];
    const Point2d& b = projected[current];
    const Point2d& c = projected[next];

    const double corner = Cross(b - a, c - b);
    if (orientationSign > 0.0 ? (corner <= eps) : (corner >= -eps))
    {
        return false;
    }

    for (std::size_t candidate : polygon)
    {
        if (candidate == previous || candidate == current || candidate == next)
        {
            continue;
        }

        if (PointInTriangle2d(projected[candidate], a, b, c, eps))
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool TriangulateSimplePolygon(
    const std::vector<Point2d>& projected,
    std::vector<TriangleMesh::TriangleIndices>& triangles,
    double eps)
{
    if (projected.size() < 3)
    {
        return false;
    }

    const double area = SignedArea2d(projected);
    if (std::abs(area) <= eps)
    {
        return false;
    }

    std::vector<std::size_t> polygon(projected.size());
    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
        polygon[i] = i;
    }

    if (area < 0.0)
    {
        std::reverse(polygon.begin(), polygon.end());
    }

    const double orientationSign = area >= 0.0 ? 1.0 : -1.0;
    while (polygon.size() > 3)
    {
        bool clipped = false;
        for (std::size_t i = 0; i < polygon.size(); ++i)
        {
            if (!IsEar(polygon, i, projected, orientationSign, eps))
            {
                continue;
            }

            const std::size_t previous = polygon[(i + polygon.size() - 1) % polygon.size()];
            const std::size_t current = polygon[i];
            const std::size_t next = polygon[(i + 1) % polygon.size()];
            triangles.push_back(TriangleMesh::TriangleIndices{previous, current, next});
            polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }

        if (!clipped)
        {
            return false;
        }
    }

    triangles.push_back(TriangleMesh::TriangleIndices{polygon[0], polygon[1], polygon[2]});
    return true;
}
} // namespace

PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronFace3d& face, double eps)
{
    if (!face.IsValid(eps))
    {
        return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
    }

    if (face.HoleCount() > 0)
    {
        return {false, MeshConversionIssue3d::UnsupportedHoles, 0, {}};
    }

    const PolyhedronLoop3d outer = face.OuterLoop();
    const auto& vertices3d = outer.Vertices();
    std::vector<Point2d> projected;
    projected.reserve(vertices3d.size());
    const Basis2dOnPlane basis = BuildPlaneBasis(face.SupportPlane(), eps);
    for (const Point3d& vertex : vertices3d)
    {
        projected.push_back(ProjectToPlane2d(vertex, face.SupportPlane(), basis));
    }

    std::vector<TriangleMesh::TriangleIndices> triangles;
    triangles.reserve(vertices3d.size() >= 2 ? vertices3d.size() - 2 : 0);
    if (!TriangulateSimplePolygon(projected, triangles, eps))
    {
        return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
    }

    return {true, MeshConversionIssue3d::None, 0, TriangleMesh(vertices3d, std::move(triangles))};
}

PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronBody& body, double eps)
{
    if (!body.IsValid(eps))
    {
        return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
    }

    std::vector<Point3d> vertices;
    std::vector<TriangleMesh::TriangleIndices> triangles;
    for (std::size_t i = 0; i < body.FaceCount(); ++i)
    {
        const PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(body.FaceAt(i), eps);
        if (!converted.success)
        {
            return {false, converted.issue, i, {}};
        }

        const std::size_t offset = vertices.size();
        const auto& faceVertices = converted.mesh.Vertices();
        const auto& faceTriangles = converted.mesh.Triangles();
        vertices.insert(vertices.end(), faceVertices.begin(), faceVertices.end());
        for (const TriangleMesh::TriangleIndices& tri : faceTriangles)
        {
            triangles.push_back(TriangleMesh::TriangleIndices{
                tri[0] + offset,
                tri[1] + offset,
                tri[2] + offset});
        }
    }

    return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
}
} // namespace geometry::sdk
