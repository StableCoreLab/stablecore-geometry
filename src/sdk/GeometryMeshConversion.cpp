#include "sdk/GeometryMeshConversion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] bool AppendLoopVerticesFromBrepLoop(
    const BrepBody& body,
    const BrepLoop& loop,
    std::vector<Point3d>& vertices,
    double eps)
{
    vertices.clear();
    if (!loop.IsValid())
    {
        return false;
    }

    vertices.reserve(loop.CoedgeCount());
    for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
    {
        const BrepCoedge coedge = loop.CoedgeAt(i);
        if (coedge.EdgeIndex() >= body.EdgeCount())
        {
            return false;
        }

        const BrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
        const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
        if (vertexIndex >= body.VertexCount())
        {
            return false;
        }

        const Point3d point = body.VertexAt(vertexIndex).Point();
        if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
        {
            vertices.push_back(point);
        }
    }

    while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), eps))
    {
        vertices.pop_back();
    }

    return vertices.size() >= 3;
}

[[nodiscard]] bool BuildPolyhedronFaceFromBrepFace(
    const BrepBody& body,
    const BrepFace& face,
    PolyhedronFace3d& polyFace,
    double eps)
{
    if (!face.IsValid(GeometryTolerance3d{eps, eps, eps}))
    {
        return false;
    }

    const auto* planeSurface = dynamic_cast<const PlaneSurface*>(face.SupportSurface());
    if (planeSurface == nullptr)
    {
        return false;
    }

    std::vector<Point3d> outerVertices;
    if (!AppendLoopVerticesFromBrepLoop(body, face.OuterLoop(), outerVertices, eps))
    {
        return false;
    }

    std::vector<PolyhedronLoop3d> holes;
    holes.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        std::vector<Point3d> holeVertices;
        if (!AppendLoopVerticesFromBrepLoop(body, face.HoleAt(i), holeVertices, eps))
        {
            return false;
        }
        holes.emplace_back(std::move(holeVertices));
    }

    polyFace = PolyhedronFace3d(
        planeSurface->SupportPlane(),
        PolyhedronLoop3d(std::move(outerVertices)),
        std::move(holes));
    return polyFace.IsValid(eps);
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

    if (a.AlmostEquals(b, eps) || b.AlmostEquals(c, eps) || a.AlmostEquals(c, eps))
    {
        return false;
    }

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

        if (projected[candidate].AlmostEquals(a, eps) ||
            projected[candidate].AlmostEquals(b, eps) ||
            projected[candidate].AlmostEquals(c, eps))
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
            const std::size_t previous = polygon[(i + polygon.size() - 1) % polygon.size()];
            const std::size_t current = polygon[i];
            const std::size_t next = polygon[(i + 1) % polygon.size()];
            if (projected[previous].AlmostEquals(projected[current], eps) ||
                projected[current].AlmostEquals(projected[next], eps))
            {
                polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
                break;
            }

            if (!IsEar(polygon, i, projected, orientationSign, eps))
            {
                continue;
            }

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

[[nodiscard]] std::vector<Point2d> RingPoints(const Polyline2d& ring)
{
    std::vector<Point2d> points;
    points.reserve(ring.PointCount());
    for (std::size_t i = 0; i < ring.PointCount(); ++i)
    {
        points.push_back(ring.PointAt(i));
    }
    return points;
}

void EnsureOrientation(std::vector<Point2d>& points, bool ccw)
{
    if (points.size() < 3)
    {
        return;
    }

    const double area = SignedArea2d(points);
    if ((ccw && area < 0.0) || (!ccw && area > 0.0))
    {
        std::reverse(points.begin(), points.end());
    }
}

[[nodiscard]] std::vector<Point2d> RemoveAdjacentDuplicates(
    const std::vector<Point2d>& points,
    double eps)
{
    std::vector<Point2d> cleaned;
    cleaned.reserve(points.size());
    for (const Point2d& point : points)
    {
        if (cleaned.empty() || !cleaned.back().AlmostEquals(point, eps))
        {
            cleaned.push_back(point);
        }
    }

    while (cleaned.size() >= 2 && cleaned.front().AlmostEquals(cleaned.back(), eps))
    {
        cleaned.pop_back();
    }
    return cleaned;
}

[[nodiscard]] std::size_t RightmostVertexIndex(const std::vector<Point2d>& ring)
{
    std::size_t index = 0;
    for (std::size_t i = 1; i < ring.size(); ++i)
    {
        if (ring[i].x > ring[index].x ||
            (std::abs(ring[i].x - ring[index].x) <= geometry::kDefaultEpsilon && ring[i].y < ring[index].y))
        {
            index = i;
        }
    }
    return index;
}

[[nodiscard]] bool IsAllowedBridgeIntersection(
    const SegmentIntersection2d& intersection,
    const Point2d& bridgeStart,
    const Point2d& bridgeEnd,
    double eps)
{
    if (!intersection.HasIntersection())
    {
        return true;
    }

    if (intersection.kind == IntersectionKind2d::Overlap)
    {
        return false;
    }

    for (std::size_t i = 0; i < intersection.pointCount; ++i)
    {
        const Point2d& point = intersection.points[i].point;
        if (!point.AlmostEquals(bridgeStart, eps) && !point.AlmostEquals(bridgeEnd, eps))
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool IsVisibleBridge(
    const Point2d& holeVertex,
    const Point2d& polygonVertex,
    const std::vector<Point2d>& currentPolygon,
    const Polygon2d& polygon,
    double eps)
{
    const LineSegment2d bridge(holeVertex, polygonVertex);
    if (!bridge.IsValid())
    {
        return false;
    }

    const Point2d midpoint = holeVertex + (polygonVertex - holeVertex) * 0.5;
    if (LocatePoint(midpoint, polygon, eps) == PointContainment2d::Outside)
    {
        return false;
    }

    const Point2d sampleA = holeVertex + (polygonVertex - holeVertex) * 0.25;
    const Point2d sampleB = holeVertex + (polygonVertex - holeVertex) * 0.75;
    if (LocatePoint(sampleA, polygon, eps) == PointContainment2d::Outside ||
        LocatePoint(sampleB, polygon, eps) == PointContainment2d::Outside)
    {
        return false;
    }

    for (std::size_t i = 0; i < currentPolygon.size(); ++i)
    {
        const Point2d& a = currentPolygon[i];
        const Point2d& b = currentPolygon[(i + 1) % currentPolygon.size()];
        const SegmentIntersection2d intersection = Intersect(bridge, LineSegment2d(a, b), eps);
        if (!IsAllowedBridgeIntersection(intersection, holeVertex, polygonVertex, eps))
        {
            return false;
        }
    }

    const Polyline2d outer = polygon.OuterRing();
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        const Point2d a = outer.PointAt(i);
        const Point2d b = outer.PointAt((i + 1) % outer.PointCount());
        const SegmentIntersection2d intersection = Intersect(bridge, LineSegment2d(a, b), eps);
        if (!IsAllowedBridgeIntersection(intersection, holeVertex, polygonVertex, eps))
        {
            return false;
        }
    }

    for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
    {
        const Polyline2d hole = polygon.HoleAt(h);
        for (std::size_t i = 0; i < hole.PointCount(); ++i)
        {
            const Point2d a = hole.PointAt(i);
            const Point2d b = hole.PointAt((i + 1) % hole.PointCount());
            const SegmentIntersection2d intersection = Intersect(bridge, LineSegment2d(a, b), eps);
            if (!IsAllowedBridgeIntersection(intersection, holeVertex, polygonVertex, eps))
            {
                return false;
            }
        }
    }

    return true;
}

[[nodiscard]] bool MergeHoleIntoPolygon(
    const Polygon2d& polygon,
    const std::vector<Point2d>& hole,
    std::vector<Point2d>& currentPolygon,
    double eps)
{
    if (hole.size() < 3 || currentPolygon.size() < 3)
    {
        return false;
    }

    const std::size_t holeIndex = RightmostVertexIndex(hole);
    const Point2d& holeVertex = hole[holeIndex];

    std::size_t outerIndex = currentPolygon.size();
    double bestDistanceSquared = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < currentPolygon.size(); ++i)
    {
        const Point2d& candidate = currentPolygon[i];
        const double distanceSquared = (candidate - holeVertex).LengthSquared();
        if (distanceSquared <= eps * eps)
        {
            continue;
        }

        if (!IsVisibleBridge(holeVertex, candidate, currentPolygon, polygon, eps))
        {
            continue;
        }

        if (distanceSquared < bestDistanceSquared)
        {
            bestDistanceSquared = distanceSquared;
            outerIndex = i;
        }
    }

    if (outerIndex == currentPolygon.size())
    {
        return false;
    }

    std::vector<Point2d> merged;
    merged.reserve(currentPolygon.size() + hole.size() + 2);
    for (std::size_t i = 0; i <= outerIndex; ++i)
    {
        merged.push_back(currentPolygon[i]);
    }

    merged.push_back(holeVertex);
    for (std::size_t step = 1; step < hole.size(); ++step)
    {
        merged.push_back(hole[(holeIndex + step) % hole.size()]);
    }
    merged.push_back(holeVertex);
    merged.push_back(currentPolygon[outerIndex]);

    for (std::size_t i = outerIndex + 1; i < currentPolygon.size(); ++i)
    {
        merged.push_back(currentPolygon[i]);
    }

    currentPolygon = RemoveAdjacentDuplicates(merged, eps);
    return currentPolygon.size() >= 3;
}

[[nodiscard]] bool BuildTriangulationContour(
    const Polygon2d& polygon,
    std::vector<Point2d>& contour,
    double eps)
{
    contour = RemoveAdjacentDuplicates(RingPoints(polygon.OuterRing()), eps);
    EnsureOrientation(contour, true);
    if (contour.size() < 3)
    {
        return false;
    }

    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        std::vector<Point2d> hole = RemoveAdjacentDuplicates(RingPoints(polygon.HoleAt(i)), eps);
        EnsureOrientation(hole, false);
        if (!MergeHoleIntoPolygon(polygon, hole, contour, eps))
        {
            return false;
        }
    }

    return contour.size() >= 3;
}

[[nodiscard]] Point3d LiftToPlane(
    const Point2d& point,
    const FaceProjection3d& projectedFace)
{
    return projectedFace.origin + projectedFace.uAxis * point.x + projectedFace.vAxis * point.y;
}
} // namespace

PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronFace3d& face, double eps)
{
    if (!face.IsValid(eps))
    {
        return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
    }

    const FaceProjection3d projectedFace = ProjectFaceToPolygon2d(
        face,
        GeometryTolerance3d{eps, eps, eps});
    if (!projectedFace.success || !projectedFace.polygon.IsValid())
    {
        return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
    }

    std::vector<Point2d> contour;
    if (!BuildTriangulationContour(projectedFace.polygon, contour, eps))
    {
        return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
    }

    std::vector<Point3d> vertices3d;
    vertices3d.reserve(contour.size());
    for (const Point2d& point : contour)
    {
        vertices3d.push_back(LiftToPlane(point, projectedFace));
    }

    std::vector<TriangleMesh::TriangleIndices> triangles;
    triangles.reserve(vertices3d.size() >= 2 ? vertices3d.size() - 2 : 0);
    if (!TriangulateSimplePolygon(contour, triangles, eps))
    {
        return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
    }

    return {true, MeshConversionIssue3d::None, 0, TriangleMesh(vertices3d, std::move(triangles))};
}

PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronBody& body, double eps)
{
    if (!body.IsValid(eps))
    {
        return {false, MeshConversionIssue3d::InvalidBody, 0, {}};
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

PolyhedronMeshConversion3d ConvertToTriangleMesh(const BrepBody& body, double eps)
{
    const GeometryTolerance3d tolerance{eps, eps, eps};
    if (!body.IsValid(tolerance))
    {
        return {false, MeshConversionIssue3d::InvalidBody, 0, {}};
    }

    std::vector<Point3d> vertices;
    std::vector<TriangleMesh::TriangleIndices> triangles;
    std::size_t faceIndex = 0;
    for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
    {
        const BrepShell shell = body.ShellAt(shellIndex);
        for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
        {
            PolyhedronFace3d polyFace{};
            if (!BuildPolyhedronFaceFromBrepFace(body, shell.FaceAt(localFaceIndex), polyFace, eps))
            {
                return {false, MeshConversionIssue3d::UnsupportedSurface, faceIndex, {}};
            }

            const PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(polyFace, eps);
            if (!converted.success)
            {
                return {false, converted.issue, faceIndex, {}};
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
    }

    return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
}
} // namespace geometry::sdk
