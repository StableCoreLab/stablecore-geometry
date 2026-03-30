#include "sdk/GeometrySection.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <sstream>
#include <vector>

#include "sdk/GeometryMeshConversion.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryTopology.h"

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

struct FaceSectionData
{
    std::vector<Point3d> outer3d{};
    std::vector<Point2d> outer2d{};
    std::vector<std::vector<Point3d>> holeContours3d{};
    std::vector<std::vector<Point2d>> holeContours2d{};
    Polygon2d polygon{};
};

struct PolylineBuildResult
{
    bool success{false};
    bool closed{false};
    std::vector<std::size_t> nodeIndices{};
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

void SimplifyOpenPolyline(
    std::vector<Point3d>& contour3d,
    std::vector<Point2d>& contour2d,
    double eps)
{
    if (contour2d.size() < 2 || contour3d.size() != contour2d.size())
    {
        return;
    }

    bool removed = true;
    while (removed && contour2d.size() >= 2)
    {
        removed = false;
        for (std::size_t i = 1; i + 1 < contour2d.size(); ++i)
        {
            const LineSegment2d chord(contour2d[i - 1], contour2d[i + 1]);
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

[[nodiscard]] bool IsCoplanarWithSectionPlane(
    const PolyhedronFace3d& face,
    const Plane& plane,
    double eps)
{
    const Plane supportPlane = face.SupportPlane();
    if (!supportPlane.IsValid(eps) || !plane.IsValid(eps))
    {
        return false;
    }

    const Vector3d firstNormal = supportPlane.UnitNormal(eps);
    const Vector3d secondNormal = plane.UnitNormal(eps);
    if (Cross(firstNormal, secondNormal).Length() > eps)
    {
        return false;
    }

    return std::abs(plane.SignedDistanceTo(supportPlane.origin, eps)) <= eps;
}

[[nodiscard]] FaceSectionData BuildCoplanarFaceSectionData(
    const PolyhedronFace3d& face,
    const Plane& plane,
    const PlaneProjectionBasis& basis,
    double eps)
{
    FaceSectionData data{};

    auto projectLoop = [&](const PolyhedronLoop3d& loop, std::vector<Point3d>& points3d, std::vector<Point2d>& points2d) {
        points3d.assign(loop.Vertices().begin(), loop.Vertices().end());
        points2d.reserve(points3d.size());
        for (const Point3d& vertex : points3d)
        {
            points2d.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
        }
        SimplifyLoop(points3d, points2d, eps);
    };

    projectLoop(face.OuterLoop(), data.outer3d, data.outer2d);

    std::vector<Polyline2d> holeRings;
    holeRings.reserve(face.HoleCount());
    data.holeContours3d.reserve(face.HoleCount());
    data.holeContours2d.reserve(face.HoleCount());
    for (std::size_t i = 0; i < face.HoleCount(); ++i)
    {
        data.holeContours3d.emplace_back();
        data.holeContours2d.emplace_back();
        projectLoop(face.HoleAt(i), data.holeContours3d.back(), data.holeContours2d.back());
        holeRings.emplace_back(data.holeContours2d.back(), PolylineClosure::Closed);
    }

    if (data.outer2d.size() >= 3)
    {
        data.polygon = Polygon2d(Polyline2d(data.outer2d, PolylineClosure::Closed), std::move(holeRings));
    }

    return data;
}

[[nodiscard]] Point3d LiftFromSectionPlane(
    const Point2d& point,
    const Point3d& origin,
    const Vector3d& uAxis,
    const Vector3d& vAxis)
{
    return origin + uAxis * point.x + vAxis * point.y;
}

[[nodiscard]] std::size_t PolygonDepth(
    const PolygonTopology2d& topology,
    std::size_t index)
{
    std::size_t depth = 0;
    std::size_t current = topology.ParentOf(index);
    while (current != static_cast<std::size_t>(-1))
    {
        ++depth;
        current = topology.ParentOf(current);
    }
    return depth;
}

void AddUniquePlaneEdgeSegments(
    const PolyhedronLoop3d& loop,
    const Plane& plane,
    std::vector<LineSegment3d>& segments,
    double eps)
{
    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
    {
        const Point3d first = loop.VertexAt(i);
        const Point3d second = loop.VertexAt((i + 1) % loop.VertexCount());
        if (std::abs(plane.SignedDistanceTo(first, eps)) > eps || std::abs(plane.SignedDistanceTo(second, eps)) > eps)
        {
            continue;
        }

        if (first.AlmostEquals(second, eps) || ContainsUndirectedSegment(segments, first, second, eps))
        {
            continue;
        }

        segments.push_back(LineSegment3d::FromStartEnd(first, second));
    }
}

[[nodiscard]] bool MarkVisitedEdge(
    std::size_t first,
    std::size_t second,
    const std::vector<IndexedSegment2d>& indexedSegments,
    std::vector<bool>& edgeVisited)
{
    for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
    {
        if (edgeVisited[edgeIndex])
        {
            continue;
        }

        const IndexedSegment2d candidate = indexedSegments[edgeIndex];
        if ((candidate.first == first && candidate.second == second) ||
            (candidate.first == second && candidate.second == first))
        {
            edgeVisited[edgeIndex] = true;
            return true;
        }
    }

    return false;
}

[[nodiscard]] bool HasUnvisitedIncidentEdge(
    std::size_t nodeIndex,
    const std::vector<std::vector<std::size_t>>& adjacency,
    const std::vector<IndexedSegment2d>& indexedSegments,
    const std::vector<bool>& edgeVisited)
{
    for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
    {
        if (edgeVisited[edgeIndex])
        {
            continue;
        }

        const IndexedSegment2d& edge = indexedSegments[edgeIndex];
        if (edge.first == nodeIndex || edge.second == nodeIndex)
        {
            return true;
        }
    }

    return false;
}

[[nodiscard]] PolylineBuildResult BuildPolylineFromNode(
    std::size_t startNode,
    bool closed,
    const std::vector<std::vector<std::size_t>>& adjacency,
    const std::vector<IndexedSegment2d>& indexedSegments,
    std::vector<bool>& edgeVisited)
{
    PolylineBuildResult result{};
    result.closed = closed;
    result.nodeIndices.push_back(startNode);

    const auto& startNeighbors = adjacency[startNode];
    if (startNeighbors.empty())
    {
        return result;
    }

    std::size_t previous = startNode;
    std::size_t current = startNeighbors.front();
    if (!MarkVisitedEdge(startNode, current, indexedSegments, edgeVisited))
    {
        return result;
    }

    while (true)
    {
        result.nodeIndices.push_back(current);
        const auto& neighbors = adjacency[current];
        if (!closed && neighbors.size() == 1)
        {
            result.success = true;
            return result;
        }

        if (closed && current == startNode)
        {
            result.nodeIndices.pop_back();
            result.success = true;
            return result;
        }

        std::size_t next = neighbors[0];
        if (next == previous && neighbors.size() > 1)
        {
            next = neighbors[1];
        }

        if (next == previous)
        {
            return result;
        }

        if (!closed && next == startNode)
        {
            return result;
        }

        if (!MarkVisitedEdge(current, next, indexedSegments, edgeVisited))
        {
            if (closed && next == startNode)
            {
                result.nodeIndices.push_back(startNode);
                result.nodeIndices.pop_back();
                result.success = true;
            }
            return result;
        }

        previous = current;
        current = next;
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

    bool hasCoplanarFace = false;
    for (const PolyhedronFace3d& face : body.Faces())
    {
        if (!IsCoplanarWithSectionPlane(face, plane, tolerance.distanceEpsilon))
        {
            AddUniquePlaneEdgeSegments(face.OuterLoop(), plane, result.segments, tolerance.distanceEpsilon);
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                AddUniquePlaneEdgeSegments(face.HoleAt(i), plane, result.segments, tolerance.distanceEpsilon);
            }
            continue;
        }

        hasCoplanarFace = true;
        const FaceSectionData faceData = BuildCoplanarFaceSectionData(face, plane, basis, tolerance.distanceEpsilon);
        if (!faceData.polygon.IsValid())
        {
            result.issue = SectionIssue3d::InvalidContour;
            return result;
        }

        result.contours.push_back(SectionPolyline3d{true, faceData.outer3d});
        result.polygons.push_back(faceData.polygon);
        for (std::size_t i = 0; i < faceData.outer3d.size(); ++i)
        {
            const std::size_t next = (i + 1) % faceData.outer3d.size();
            if (!ContainsUndirectedSegment(
                    result.segments,
                    faceData.outer3d[i],
                    faceData.outer3d[next],
                    tolerance.distanceEpsilon))
            {
                result.segments.push_back(LineSegment3d::FromStartEnd(faceData.outer3d[i], faceData.outer3d[next]));
            }
        }

        for (const auto& holeContour : faceData.holeContours3d)
        {
            result.contours.push_back(SectionPolyline3d{true, holeContour});
            for (std::size_t i = 0; i < holeContour.size(); ++i)
            {
                const std::size_t next = (i + 1) % holeContour.size();
                if (!ContainsUndirectedSegment(
                        result.segments,
                        holeContour[i],
                        holeContour[next],
                        tolerance.distanceEpsilon))
                {
                    result.segments.push_back(LineSegment3d::FromStartEnd(holeContour[i], holeContour[next]));
                }
            }
        }
    }

    if (hasCoplanarFace)
    {
        result.success = true;
        return result;
    }

    const auto meshConversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
    if (!meshConversion.success || !meshConversion.mesh.IsValid(tolerance.distanceEpsilon))
    {
        result.issue = SectionIssue3d::MeshConversionFailed;
        return result;
    }

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
                hasCoplanarFace))
        {
            continue;
        }

        if (!ContainsUndirectedSegment(rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
        {
            rawSegments.push_back(segment);
        }
    }

    for (const LineSegment3d& segment : result.segments)
    {
        if (!ContainsUndirectedSegment(rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
        {
            rawSegments.push_back(segment);
        }
    }

    result.segments = rawSegments;
    if (result.segments.empty())
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

        if (neighbors.size() > 2)
        {
            result.issue = SectionIssue3d::NonManifoldContour;
            return result;
        }
    }

    std::vector<bool> edgeVisited(indexedSegments.size(), false);
    for (std::size_t nodeIndex = 0; nodeIndex < adjacency.size(); ++nodeIndex)
    {
        if (adjacency[nodeIndex].size() != 1 || !HasUnvisitedIncidentEdge(nodeIndex, adjacency, indexedSegments, edgeVisited))
        {
            continue;
        }

        const PolylineBuildResult polyline = BuildPolylineFromNode(
            nodeIndex,
            false,
            adjacency,
            indexedSegments,
            edgeVisited);
        if (!polyline.success)
        {
            result.issue = SectionIssue3d::OpenContour;
            return result;
        }

        std::vector<Point3d> contour3d;
        std::vector<Point2d> contour2d;
        contour3d.reserve(polyline.nodeIndices.size());
        contour2d.reserve(polyline.nodeIndices.size());
        for (std::size_t contourNodeIndex : polyline.nodeIndices)
        {
            contour3d.push_back(nodePoints3d[contourNodeIndex]);
            contour2d.push_back(projectedNodes[contourNodeIndex]);
        }

        SimplifyOpenPolyline(contour3d, contour2d, tolerance.distanceEpsilon);
        if (contour2d.size() < 2)
        {
            result.issue = SectionIssue3d::InvalidContour;
            return result;
        }

        result.contours.push_back(SectionPolyline3d{false, std::move(contour3d)});
    }

    for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
    {
        if (edgeVisited[edgeIndex])
        {
            continue;
        }

        const PolylineBuildResult polyline = BuildPolylineFromNode(
            indexedSegments[edgeIndex].first,
            true,
            adjacency,
            indexedSegments,
            edgeVisited);
        if (!polyline.success)
        {
            result.issue = SectionIssue3d::InvalidContour;
            return result;
        }

        std::vector<Point3d> contour3d;
        std::vector<Point2d> contour2d;
        contour3d.reserve(polyline.nodeIndices.size());
        contour2d.reserve(polyline.nodeIndices.size());
        for (std::size_t contourNodeIndex : polyline.nodeIndices)
        {
            contour3d.push_back(nodePoints3d[contourNodeIndex]);
            contour2d.push_back(projectedNodes[contourNodeIndex]);
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

SectionFaceRebuild3d RebuildSectionFaces(const PolyhedronSection3d& section, double eps)
{
    SectionFaceRebuild3d result{};
    if (!section.success || !section.IsValid(eps))
    {
        result.issue = SectionFaceRebuildIssue3d::InvalidSection;
        return result;
    }

    const Vector3d normal = Cross(section.uAxis, section.vAxis);
    const Plane supportPlane = Plane::FromPointAndNormal(section.origin, normal);
    if (!supportPlane.IsValid(eps))
    {
        result.issue = SectionFaceRebuildIssue3d::InvalidSection;
        return result;
    }

    MultiPolygon2d polygons;
    for (const Polygon2d& polygon : section.polygons)
    {
        if (!polygon.IsValid())
        {
            result.issue = SectionFaceRebuildIssue3d::InvalidPolygon;
            return result;
        }
        polygons.Add(polygon);
    }

    const PolygonTopology2d topology = BuildPolygonTopology(polygons, eps);
    result.faces.reserve(section.polygons.size());
    for (std::size_t polygonIndex = 0; polygonIndex < polygons.Count(); ++polygonIndex)
    {
        if ((PolygonDepth(topology, polygonIndex) % 2) != 0)
        {
            continue;
        }

        const Polygon2d& polygon = polygons.PolygonAt(polygonIndex);

        std::vector<Point3d> outerVertices;
        const Polyline2d outerRing = polygon.OuterRing();
        outerVertices.reserve(outerRing.PointCount());
        for (std::size_t i = 0; i < outerRing.PointCount(); ++i)
        {
            outerVertices.push_back(LiftFromSectionPlane(
                outerRing.PointAt(i),
                section.origin,
                section.uAxis,
                section.vAxis));
        }

        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(polygon.HoleCount() + topology.ChildrenOf(polygonIndex).size());
        for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
        {
            const Polyline2d holeRing = polygon.HoleAt(holeIndex);
            std::vector<Point3d> holeVertices;
            holeVertices.reserve(holeRing.PointCount());
            for (std::size_t i = 0; i < holeRing.PointCount(); ++i)
            {
                holeVertices.push_back(LiftFromSectionPlane(
                    holeRing.PointAt(i),
                    section.origin,
                    section.uAxis,
                    section.vAxis));
            }
            holes.emplace_back(std::move(holeVertices));
        }

        for (std::size_t childIndex : topology.ChildrenOf(polygonIndex))
        {
            if ((PolygonDepth(topology, childIndex) % 2) == 0)
            {
                continue;
            }

            const Polyline2d holeRing = polygons.PolygonAt(childIndex).OuterRing();
            std::vector<Point3d> holeVertices;
            holeVertices.reserve(holeRing.PointCount());
            for (std::size_t i = 0; i < holeRing.PointCount(); ++i)
            {
                holeVertices.push_back(LiftFromSectionPlane(
                    holeRing.PointAt(i),
                    section.origin,
                    section.uAxis,
                    section.vAxis));
            }
            holes.emplace_back(std::move(holeVertices));
        }

        result.faces.emplace_back(
            supportPlane,
            PolyhedronLoop3d(std::move(outerVertices)),
            std::move(holes));
    }

    result.success = true;
    return result;
}

SectionBodyRebuild3d RebuildSectionBody(const PolyhedronSection3d& section, double eps)
{
    SectionBodyRebuild3d result{};
    if (!section.success || !section.IsValid(eps))
    {
        result.issue = SectionBodyRebuildIssue3d::InvalidSection;
        return result;
    }

    const SectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
    if (!rebuiltFaces.success)
    {
        result.issue = SectionBodyRebuildIssue3d::FaceRebuildFailed;
        return result;
    }

    result.body = PolyhedronBody(rebuiltFaces.faces);
    result.success = true;
    return result;
}

SectionTopology3d BuildSectionTopology(const PolyhedronSection3d& section, double eps)
{
    SectionTopology3d result{};
    if (!section.success || !section.IsValid(eps))
    {
        return result;
    }

    MultiPolygon2d polygons;
    for (const Polygon2d& polygon : section.polygons)
    {
        if (!polygon.IsValid())
        {
            return result;
        }
        polygons.Add(polygon);
    }

    const PolygonTopology2d topology = BuildPolygonTopology(polygons, eps);
    if (!topology.IsValid())
    {
        return result;
    }

    result.valid_ = true;
    result.nodes_.resize(topology.Count());
    result.roots_ = topology.Roots();
    for (std::size_t i = 0; i < topology.Count(); ++i)
    {
        result.nodes_[i].polygonIndex = i;
        result.nodes_[i].parentIndex = topology.ParentOf(i);
        result.nodes_[i].children = topology.ChildrenOf(i);
        result.nodes_[i].depth = PolygonDepth(topology, i);
    }

    return result;
}

std::string SectionTopology3d::DebugString() const
{
    std::ostringstream stream;
    stream << "SectionTopology3d{polygonCount=" << Count()
           << ", rootCount=" << roots_.size()
           << ", valid=" << (valid_ ? "true" : "false") << "}";
    return stream.str();
}

SectionMeshConversion3d ConvertSectionToTriangleMesh(const PolyhedronSection3d& section, double eps)
{
    SectionMeshConversion3d result{};
    const SectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
    if (!rebuiltFaces.success)
    {
        result.issue = MeshConversionIssue3d::InvalidFace;
        return result;
    }

    std::vector<Point3d> vertices;
    std::vector<TriangleMesh::TriangleIndices> triangles;
    for (const PolyhedronFace3d& face : rebuiltFaces.faces)
    {
        const PolyhedronMeshConversion3d faceMesh = ConvertToTriangleMesh(face, eps);
        if (!faceMesh.success)
        {
            result.issue = faceMesh.issue;
            return result;
        }

        const std::size_t vertexOffset = vertices.size();
        vertices.insert(vertices.end(), faceMesh.mesh.Vertices().begin(), faceMesh.mesh.Vertices().end());
        for (const auto& triangle : faceMesh.mesh.Triangles())
        {
            triangles.push_back(TriangleMesh::TriangleIndices{
                triangle[0] + vertexOffset,
                triangle[1] + vertexOffset,
                triangle[2] + vertexOffset});
        }
    }

    result.mesh = TriangleMesh(std::move(vertices), std::move(triangles));
    result.success = true;
    return result;
}
} // namespace geometry::sdk
