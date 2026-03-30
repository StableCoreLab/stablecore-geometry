#include "sdk/GeometrySection.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <vector>

#include "sdk/GeometryMeshConversion.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"

namespace geometry::sdk
{
namespace
{
struct PlaneProjectionBasis
{
    Vector3d u{};
    Vector3d v{};
};

struct IndexedSegment2d
{
    std::size_t first{0};
    std::size_t second{0};
};

[[nodiscard]] PlaneProjectionBasis BuildPlaneProjectionBasis(const Plane& plane, double eps)
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

[[nodiscard]] Point2d ProjectToLocalPlaneCoordinates(
    const Point3d& point,
    const Plane& plane,
    const PlaneProjectionBasis& basis)
{
    const Vector3d delta = point - plane.origin;
    return Point2d{Dot(delta, basis.u), Dot(delta, basis.v)};
}

[[nodiscard]] std::size_t FindOrAddPoint2d(
    const Point2d& point,
    std::vector<Point2d>& uniquePoints,
    double eps)
{
    for (std::size_t i = 0; i < uniquePoints.size(); ++i)
    {
        if (uniquePoints[i].AlmostEquals(point, eps))
        {
            return i;
        }
    }

    uniquePoints.push_back(point);
    return uniquePoints.size() - 1;
}

[[nodiscard]] bool ContainsUndirectedSegment(
    const std::vector<LineSegment3d>& segments,
    const Point3d& first,
    const Point3d& second,
    double eps)
{
    for (const LineSegment3d& segment : segments)
    {
        if ((segment.startPoint.AlmostEquals(first, eps) && segment.endPoint.AlmostEquals(second, eps)) ||
            (segment.startPoint.AlmostEquals(second, eps) && segment.endPoint.AlmostEquals(first, eps)))
        {
            return true;
        }
    }
    return false;
}

void AddUniqueIntersectionPoint(
    const Point3d& point,
    std::vector<Point3d>& points,
    double eps)
{
    for (const Point3d& candidate : points)
    {
        if (candidate.AlmostEquals(point, eps))
        {
            return;
        }
    }

    points.push_back(point);
}

[[nodiscard]] Point3d InterpolateToPlane(
    const Point3d& first,
    const Point3d& second,
    double firstDistance,
    double secondDistance)
{
    const double denominator = firstDistance - secondDistance;
    if (std::abs(denominator) <= geometry::kDefaultEpsilon)
    {
        return first;
    }

    const double parameter = firstDistance / denominator;
    return first + (second - first) * parameter;
}

[[nodiscard]] bool SliceTriangle(
    const Triangle3d& triangle,
    const Plane& plane,
    double eps,
    LineSegment3d& segment,
    bool& coplanarGeometry)
{
    std::array<Point3d, 3> vertices{triangle.a, triangle.b, triangle.c};
    std::array<double, 3> distances{
        plane.SignedDistanceTo(vertices[0], eps),
        plane.SignedDistanceTo(vertices[1], eps),
        plane.SignedDistanceTo(vertices[2], eps)};

    const bool on0 = std::abs(distances[0]) <= eps;
    const bool on1 = std::abs(distances[1]) <= eps;
    const bool on2 = std::abs(distances[2]) <= eps;
    if (on0 && on1 && on2)
    {
        coplanarGeometry = true;
        return false;
    }

    std::vector<Point3d> points;
    points.reserve(3);
    const std::array<std::array<std::size_t, 2>, 3> edges{{{{0, 1}}, {{1, 2}}, {{2, 0}}}};
    for (const auto& edge : edges)
    {
        const std::size_t i = edge[0];
        const std::size_t j = edge[1];
        const bool onI = std::abs(distances[i]) <= eps;
        const bool onJ = std::abs(distances[j]) <= eps;
        if (onI && onJ)
        {
            coplanarGeometry = true;
            return false;
        }

        if (onI)
        {
            AddUniqueIntersectionPoint(vertices[i], points, eps);
            continue;
        }

        if (onJ)
        {
            AddUniqueIntersectionPoint(vertices[j], points, eps);
            continue;
        }

        if ((distances[i] < -eps && distances[j] > eps) || (distances[i] > eps && distances[j] < -eps))
        {
            AddUniqueIntersectionPoint(
                InterpolateToPlane(vertices[i], vertices[j], distances[i], distances[j]),
                points,
                eps);
        }
    }

    if (points.size() != 2 || points[0].AlmostEquals(points[1], eps))
    {
        return false;
    }

    segment = LineSegment3d::FromStartEnd(points[0], points[1]);
    return segment.IsValid(eps);
}

void SimplifyLoop(
    std::vector<Point3d>& contour3d,
    std::vector<Point2d>& contour2d,
    double eps)
{
    if (contour2d.size() < 3 || contour3d.size() != contour2d.size())
    {
        return;
    }

    bool removed = true;
    while (removed && contour2d.size() >= 3)
    {
        removed = false;
        for (std::size_t i = 0; i < contour2d.size(); ++i)
        {
            const std::size_t previous = (i + contour2d.size() - 1) % contour2d.size();
            const std::size_t next = (i + 1) % contour2d.size();
            const LineSegment2d chord(contour2d[previous], contour2d[next]);
            if (!chord.IsValid())
            {
                continue;
            }

            if (LocatePoint(contour2d[i], chord, eps) != PointContainment2d::OnBoundary)
            {
                continue;
            }

            contour2d.erase(contour2d.begin() + static_cast<std::ptrdiff_t>(i));
            contour3d.erase(contour3d.begin() + static_cast<std::ptrdiff_t>(i));
            removed = true;
            break;
        }
    }
}

} // namespace

PolyhedronSection3d Section(
    const PolyhedronBody& body,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    PolyhedronSection3d result{};
    if (!plane.IsValid(tolerance.distanceEpsilon))
    {
        result.issue = SectionIssue3d::InvalidPlane;
        return result;
    }

    if (!body.IsValid(tolerance.distanceEpsilon))
    {
        result.issue = SectionIssue3d::InvalidBody;
        return result;
    }

    result.origin = plane.origin;
    const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
    result.uAxis = basis.u;
    result.vAxis = basis.v;

    const auto meshConversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
    if (!meshConversion.success || !meshConversion.mesh.IsValid(tolerance.distanceEpsilon))
    {
        result.issue = SectionIssue3d::MeshConversionFailed;
        return result;
    }

    bool hasCoplanarGeometry = false;
    std::vector<LineSegment3d> rawSegments;
    rawSegments.reserve(meshConversion.mesh.TriangleCount());
    for (std::size_t i = 0; i < meshConversion.mesh.TriangleCount(); ++i)
    {
        LineSegment3d segment{};
        if (!SliceTriangle(
                meshConversion.mesh.TriangleAt(i),
                plane,
                tolerance.distanceEpsilon,
                segment,
                hasCoplanarGeometry))
        {
            continue;
        }

        if (!ContainsUndirectedSegment(rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
        {
            rawSegments.push_back(segment);
        }
    }

    if (hasCoplanarGeometry)
    {
        result.issue = SectionIssue3d::CoplanarGeometryUnsupported;
        return result;
    }

    result.segments = rawSegments;
    if (rawSegments.empty())
    {
        result.success = true;
        return result;
    }

    std::vector<Point2d> projectedNodes;
    std::vector<Point3d> nodePoints3d;
    std::vector<IndexedSegment2d> indexedSegments;
    indexedSegments.reserve(rawSegments.size());
    for (const LineSegment3d& segment : rawSegments)
    {
        const Point2d first2d = ProjectToLocalPlaneCoordinates(segment.startPoint, plane, basis);
        const Point2d second2d = ProjectToLocalPlaneCoordinates(segment.endPoint, plane, basis);
        const std::size_t firstIndex = FindOrAddPoint2d(first2d, projectedNodes, tolerance.distanceEpsilon);
        const std::size_t secondIndex = FindOrAddPoint2d(second2d, projectedNodes, tolerance.distanceEpsilon);
        if (firstIndex == secondIndex)
        {
            continue;
        }

        while (nodePoints3d.size() < projectedNodes.size())
        {
            nodePoints3d.push_back(Point3d{});
        }
        nodePoints3d[firstIndex] = segment.startPoint;
        nodePoints3d[secondIndex] = segment.endPoint;

        bool duplicate = false;
        for (const IndexedSegment2d& existing : indexedSegments)
        {
            if ((existing.first == firstIndex && existing.second == secondIndex) ||
                (existing.first == secondIndex && existing.second == firstIndex))
            {
                duplicate = true;
                break;
            }
        }

        if (!duplicate)
        {
            indexedSegments.push_back(IndexedSegment2d{firstIndex, secondIndex});
        }
    }

    std::vector<std::vector<std::size_t>> adjacency(projectedNodes.size());
    for (const IndexedSegment2d& segment : indexedSegments)
    {
        adjacency[segment.first].push_back(segment.second);
        adjacency[segment.second].push_back(segment.first);
    }

    for (const auto& neighbors : adjacency)
    {
        if (neighbors.empty())
        {
            continue;
        }

        if (neighbors.size() == 1)
        {
            result.issue = SectionIssue3d::OpenContour;
            return result;
        }

        if (neighbors.size() != 2)
        {
            result.issue = SectionIssue3d::NonManifoldContour;
            return result;
        }
    }

    std::vector<bool> edgeVisited(indexedSegments.size(), false);
    for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
    {
        if (edgeVisited[edgeIndex])
        {
            continue;
        }

        const IndexedSegment2d seed = indexedSegments[edgeIndex];
        std::vector<std::size_t> loopNodeIndices{seed.first};
        std::size_t previous = seed.first;
        std::size_t current = seed.second;
        edgeVisited[edgeIndex] = true;

        while (true)
        {
            loopNodeIndices.push_back(current);
            if (current == loopNodeIndices.front())
            {
                break;
            }

            const auto& neighbors = adjacency[current];
            const std::size_t next = neighbors[0] == previous ? neighbors[1] : neighbors[0];
            bool foundEdge = false;
            for (std::size_t candidateEdgeIndex = 0; candidateEdgeIndex < indexedSegments.size(); ++candidateEdgeIndex)
            {
                if (edgeVisited[candidateEdgeIndex])
                {
                    continue;
                }

                const IndexedSegment2d candidate = indexedSegments[candidateEdgeIndex];
                if ((candidate.first == current && candidate.second == next) ||
                    (candidate.first == next && candidate.second == current))
                {
                    edgeVisited[candidateEdgeIndex] = true;
                    foundEdge = true;
                    break;
                }
            }

            if (!foundEdge && next != loopNodeIndices.front())
            {
                result.issue = SectionIssue3d::OpenContour;
                return result;
            }

            previous = current;
            current = next;
        }

        loopNodeIndices.pop_back();
        std::vector<Point3d> contour3d;
        std::vector<Point2d> contour2d;
        contour3d.reserve(loopNodeIndices.size());
        contour2d.reserve(loopNodeIndices.size());
        for (std::size_t nodeIndex : loopNodeIndices)
        {
            contour3d.push_back(nodePoints3d[nodeIndex]);
            contour2d.push_back(projectedNodes[nodeIndex]);
        }

        SimplifyLoop(contour3d, contour2d, tolerance.distanceEpsilon);
        if (contour2d.size() < 3)
        {
            result.issue = SectionIssue3d::InvalidContour;
            return result;
        }

        Polygon2d polygon(Polyline2d(contour2d, PolylineClosure::Closed));
        if (!polygon.IsValid())
        {
            result.issue = SectionIssue3d::InvalidContour;
            return result;
        }

        result.contours.push_back(SectionPolyline3d{true, std::move(contour3d)});
        result.polygons.push_back(std::move(polygon));
    }

    result.success = true;
    return result;
}
} // namespace geometry::sdk
