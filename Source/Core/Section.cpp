#include "Core/Section.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include "Brep/MeshConversion.h"
#include "Brep/Topology.h"
#include "Core/Boolean.h"
#include "Core/Editing.h"
#include "Core/Offset.h"
#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Geometry2d/SCPathOps.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        SCPoint2d ProjectPointToSectionBasis(const SCPoint3d& point, const SCPolyhedronSection3d& section);

        SCPoint3d LiftFromSectionPlane(const SCPoint2d& point,
                                     const SCPoint3d& origin,
                                     const SCVector3d& uAxis,
                                     const SCVector3d& vAxis);

        struct PlaneProjectionBasis
        {
            SCVector3d u{};
            SCVector3d v{};
        };

        struct IndexedSegment2d
        {
            std::size_t first{0};
            std::size_t second{0};
        };

        struct FaceSectionData
        {
            std::vector<SCPoint3d> outer3d{};
            std::vector<SCPoint2d> outer2d{};
            std::vector<std::vector<SCPoint3d>> holeContours3d{};
            std::vector<std::vector<SCPoint2d>> holeContours2d{};
            SCPolygon2d polygon{};
        };

        struct PolylineBuildResult
        {
            bool success{false};
            bool closed{false};
            std::vector<std::size_t> nodeIndices{};
        };

        bool IsPoint3dLexicographicallyLess(const SCPoint3d& first, const SCPoint3d& second, double eps)
        {
            if (std::abs(first.x - second.x) > eps)
            {
                return first.x < second.x;
            }
            if (std::abs(first.y - second.y) > eps)
            {
                return first.y < second.y;
            }
            return first.z < second.z - eps;
        }

        PlaneProjectionBasis BuildPlaneProjectionBasis(const SCPlane& plane, double eps)
        {
            const SCVector3d normal = plane.UnitNormal(eps);
            const SCVector3d axis =
                std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z)
                    ? SCVector3d{1.0, 0.0, 0.0}
                    : (std::abs(normal.y) <= std::abs(normal.z) ? SCVector3d{0.0, 1.0, 0.0} : SCVector3d{0.0, 0.0, 1.0});
            const SCVector3d u = Cross(normal, axis).Normalized(eps);
            const SCVector3d v = Cross(normal, u).Normalized(eps);
            return {u, v};
        }

        SCPoint2d ProjectToLocalPlaneCoordinates(const SCPoint3d& point,
                                               const SCPlane& plane,
                                               const PlaneProjectionBasis& basis)
        {
            const SCVector3d delta = point - plane.origin;
            return SCPoint2d{Dot(delta, basis.u), Dot(delta, basis.v)};
        }

        std::size_t FindOrAddPoint2d(const SCPoint2d& point, std::vector<SCPoint2d>& uniquePoints, double eps)
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

        bool ContainsUndirectedSegment(const std::vector<SCLineSegment3d>& segments,
                                       const SCPoint3d& first,
                                       const SCPoint3d& second,
                                       double eps)
        {
            for (const SCLineSegment3d& segment : segments)
            {
                if ((segment.startPoint.AlmostEquals(first, eps) && segment.endPoint.AlmostEquals(second, eps)) ||
                    (segment.startPoint.AlmostEquals(second, eps) && segment.endPoint.AlmostEquals(first, eps)))
                {
                    return true;
                }
            }
            return false;
        }

        bool AddNormalizedSectionSegment(const SCPoint3d& first,
                                         const SCPoint3d& second,
                                         std::vector<SCLineSegment3d>& segments,
                                         double eps)
        {
            if (first.AlmostEquals(second, eps))
            {
                return false;
            }

            if ((second - first).Length() <= eps)
            {
                return false;
            }

            if (ContainsUndirectedSegment(segments, first, second, eps))
            {
                return false;
            }

            segments.push_back(SCLineSegment3d::FromStartEnd(first, second));
            return true;
        }

        void RebuildUniqueSegmentsFromContours(SCPolyhedronSection3d& section, double eps)
        {
            section.segments.clear();
            for (const SCSectionPolyline3d& contour : section.contours)
            {
                const std::size_t minimumPoints = contour.closed ? 3u : 2u;
                if (contour.points.size() < minimumPoints)
                {
                    continue;
                }

                for (std::size_t i = 0; i + 1 < contour.points.size(); ++i)
                {
                    AddNormalizedSectionSegment(contour.points[i], contour.points[i + 1], section.segments, eps);
                }

                if (contour.closed)
                {
                    AddNormalizedSectionSegment(contour.points.back(), contour.points.front(), section.segments, eps);
                }
            }
        }

        bool IsPointOnAnyPolygonBoundary(const SCPoint3d& point, const SCPolyhedronSection3d& section, double eps)
        {
            const double boundaryEps = std::max(eps, Geometry::kPathOpsComparisonEpsilon);
            const SCPoint2d projected = ProjectPointToSectionBasis(point, section);
            for (const SCPolygon2d& polygon : section.polygons)
            {
                if (LocatePoint(projected, polygon, boundaryEps) == SCPointContainment2d::OnBoundary)
                {
                    return true;
                }
            }
            return false;
        }

        bool IsPointAtAnyPolygonVertex(const SCPoint3d& point, const SCPolyhedronSection3d& section, double eps)
        {
            const double boundaryEps = std::max(eps, Geometry::kPathOpsComparisonEpsilon);
            const SCPoint2d projected = ProjectPointToSectionBasis(point, section);
            auto ringContainsVertex = [&](const SCPolyline2d& ring) {
                for (std::size_t i = 0; i < ring.PointCount(); ++i)
                {
                    if (ring.PointAt(i).AlmostEquals(projected, boundaryEps))
                    {
                        return true;
                    }
                }
                return false;
            };

            for (const SCPolygon2d& polygon : section.polygons)
            {
                if (ringContainsVertex(polygon.OuterRing()))
                {
                    return true;
                }

                for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
                {
                    if (ringContainsVertex(polygon.HoleAt(holeIndex)))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        enum class OpenContourAttachmentKind
        {
            EdgeAttached = 0,
            VertexAttached = 1,
            Detached = 2
        };

        struct OpenContourSortKey
        {
            OpenContourAttachmentKind attachmentKind{OpenContourAttachmentKind::Detached};
            std::size_t boundaryEndpointCount{0};
        };

        void NormalizeOpenContourDirection(const SCPolyhedronSection3d& section, SCSectionPolyline3d& contour, double eps)
        {
            if (contour.closed || contour.points.size() < 2)
            {
                return;
            }

            const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.front(), section, eps);
            const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.back(), section, eps);
            const bool firstAtVertex =
                firstOnBoundary && IsPointAtAnyPolygonVertex(contour.points.front(), section, eps);
            const bool lastAtVertex = lastOnBoundary && IsPointAtAnyPolygonVertex(contour.points.back(), section, eps);

            const bool hasBoundaryEndpoint = firstOnBoundary || lastOnBoundary;
            const bool isVertexAttached =
                hasBoundaryEndpoint && ((firstOnBoundary && firstAtVertex) || (lastOnBoundary && lastAtVertex));
            const bool isEdgeAttached = hasBoundaryEndpoint && !isVertexAttached &&
                                        ((firstOnBoundary && !firstAtVertex) || (lastOnBoundary && !lastAtVertex));

            if (isEdgeAttached)
            {
                const SCPoint3d& boundaryPoint = firstOnBoundary ? contour.points.front() : contour.points.back();
                const SCPoint3d& freePoint = firstOnBoundary ? contour.points.back() : contour.points.front();
                const SCPlane plane = SCPlane::FromPointAndNormal(section.origin, Cross(section.uAxis, section.vAxis));
                const SCPointContainment2d freeContainment =
                    LocatePoint(ProjectToLocalPlaneCoordinates(
                                    freePoint, plane, PlaneProjectionBasis{section.uAxis, section.vAxis}),
                                section.polygons.front(),
                                eps);

                if (freeContainment == SCPointContainment2d::Inside || freeContainment == SCPointContainment2d::OnBoundary)
                {
                    if (lastOnBoundary && !firstOnBoundary)
                    {
                        std::reverse(contour.points.begin(), contour.points.end());
                    }
                } else if (firstOnBoundary && IsPoint3dLexicographicallyLess(freePoint, boundaryPoint, eps))
                {
                    std::reverse(contour.points.begin(), contour.points.end());
                } else if (lastOnBoundary && IsPoint3dLexicographicallyLess(boundaryPoint, freePoint, eps))
                {
                    std::reverse(contour.points.begin(), contour.points.end());
                }
                return;
            }

            if (isVertexAttached)
            {
                if (lastOnBoundary && !firstOnBoundary)
                {
                    std::reverse(contour.points.begin(), contour.points.end());
                }
                return;
            }

            if (IsPoint3dLexicographicallyLess(contour.points.back(), contour.points.front(), eps))
            {
                std::reverse(contour.points.begin(), contour.points.end());
            }
        }

        OpenContourSortKey BuildOpenContourSortKey(const SCPolyhedronSection3d& section,
                                                   const SCSectionPolyline3d& contour,
                                                   double eps)
        {
            OpenContourSortKey key{};
            if (contour.closed || contour.points.empty())
            {
                return key;
            }

            const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.front(), section, eps);
            const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.back(), section, eps);
            key.boundaryEndpointCount = (firstOnBoundary ? 1U : 0U) + (lastOnBoundary ? 1U : 0U);
            if (key.boundaryEndpointCount == 0U)
            {
                key.attachmentKind = OpenContourAttachmentKind::Detached;
                return key;
            }

            const bool firstAtVertex =
                firstOnBoundary && IsPointAtAnyPolygonVertex(contour.points.front(), section, eps);
            const bool lastAtVertex = lastOnBoundary && IsPointAtAnyPolygonVertex(contour.points.back(), section, eps);
            if ((firstOnBoundary && !firstAtVertex) || (lastOnBoundary && !lastAtVertex))
            {
                key.attachmentKind = OpenContourAttachmentKind::EdgeAttached;
                return key;
            }

            key.attachmentKind = OpenContourAttachmentKind::VertexAttached;
            return key;
        }

        void SortOpenContoursStable(SCPolyhedronSection3d& section, double eps)
        {
            std::vector<SCSectionPolyline3d> closedContours;
            std::vector<SCSectionPolyline3d> openContours;
            closedContours.reserve(section.contours.size());
            openContours.reserve(section.contours.size());

            for (SCSectionPolyline3d contour : section.contours)
            {
                if (contour.closed)
                {
                    closedContours.push_back(std::move(contour));
                    continue;
                }

                NormalizeOpenContourDirection(section, contour, eps);
                openContours.push_back(std::move(contour));
            }

            std::sort(openContours.begin(),
                      openContours.end(),
                      [&section, eps](const SCSectionPolyline3d& first, const SCSectionPolyline3d& second) {
                          if (first.points.empty() || second.points.empty())
                          {
                              return first.points.size() < second.points.size();
                          }

                          const OpenContourSortKey firstKey = BuildOpenContourSortKey(section, first, eps);
                          const OpenContourSortKey secondKey = BuildOpenContourSortKey(section, second, eps);
                          if (firstKey.attachmentKind != secondKey.attachmentKind)
                          {
                              return static_cast<int>(firstKey.attachmentKind) <
                                     static_cast<int>(secondKey.attachmentKind);
                          }
                          if (firstKey.boundaryEndpointCount != secondKey.boundaryEndpointCount)
                          {
                              return firstKey.boundaryEndpointCount > secondKey.boundaryEndpointCount;
                          }

                          if (IsPoint3dLexicographicallyLess(first.points.front(), second.points.front(), eps))
                          {
                              return true;
                          }
                          if (IsPoint3dLexicographicallyLess(second.points.front(), first.points.front(), eps))
                          {
                              return false;
                          }
                          const std::size_t minimumCount = std::min(first.points.size(), second.points.size());
                          for (std::size_t index = 0; index < minimumCount; ++index)
                          {
                              if (IsPoint3dLexicographicallyLess(first.points[index], second.points[index], eps))
                              {
                                  return true;
                              }
                              if (IsPoint3dLexicographicallyLess(second.points[index], first.points[index], eps))
                              {
                                  return false;
                              }
                          }

                          return first.points.size() < second.points.size();
                      });

            closedContours.insert(closedContours.end(), openContours.begin(), openContours.end());
            section.contours = std::move(closedContours);
        }

        void AddUniqueIntersectionPoint(const SCPoint3d& point, std::vector<SCPoint3d>& points, double eps)
        {
            for (const SCPoint3d& candidate : points)
            {
                if (candidate.AlmostEquals(point, eps))
                {
                    return;
                }
            }

            points.push_back(point);
        }

        SCPoint3d InterpolateToPlane(const SCPoint3d& first,
                                   const SCPoint3d& second,
                                   double firstDistance,
                                   double secondDistance)
        {
            const double denominator = firstDistance - secondDistance;
            if (std::abs(denominator) <= Geometry::kSectionDefaultEpsilon)
            {
                return first;
            }

            const double parameter = firstDistance / denominator;
            return first + (second - first) * parameter;
        }

        bool SliceTriangle(
            const SCTriangle3d& triangle, const SCPlane& plane, double eps, SCLineSegment3d& segment, bool& coplanarGeometry)
        {
            std::array<SCPoint3d, 3> vertices{triangle.a, triangle.b, triangle.c};
            std::array<double, 3> distances{plane.SignedDistanceTo(vertices[0], eps),
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

            std::vector<SCPoint3d> points;
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
                        InterpolateToPlane(vertices[i], vertices[j], distances[i], distances[j]), points, eps);
                }
            }

            if (points.size() != 2 || points[0].AlmostEquals(points[1], eps))
            {
                return false;
            }

            segment = SCLineSegment3d::FromStartEnd(points[0], points[1]);
            return segment.IsValid(eps);
        }

        bool ContainsPoint2d(const std::vector<SCPoint2d>& points, const SCPoint2d& point, double eps)
        {
            for (const SCPoint2d& candidate : points)
            {
                if (candidate.AlmostEquals(point, eps))
                {
                    return true;
                }
            }

            return false;
        }

        void SimplifyLoop(std::vector<SCPoint3d>& contour3d,
                          std::vector<SCPoint2d>& contour2d,
                          const std::vector<SCPoint2d>* preservePoints,
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
                    const SCLineSegment2d chord(contour2d[previous], contour2d[next]);
                    if (!chord.IsValid())
                    {
                        continue;
                    }

                    if (LocatePoint(contour2d[i], chord, eps) != SCPointContainment2d::OnBoundary)
                    {
                        continue;
                    }

                    if (preservePoints != nullptr && ContainsPoint2d(*preservePoints, contour2d[i], eps))
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

        void SimplifyOpenPolyline(std::vector<SCPoint3d>& contour3d,
                                  std::vector<SCPoint2d>& contour2d,
                                  const SCPolyhedronSection3d& section,
                                  const SCPlane& plane,
                                  const PlaneProjectionBasis& basis,
                                  const std::vector<SCPoint2d>* preservePoints,
                                  double eps)
        {
            if (contour2d.size() < 2 || contour3d.size() != contour2d.size())
            {
                return;
            }

            const std::size_t originalCount = contour2d.size();
            bool removed = true;
            while (removed && contour2d.size() >= 2)
            {
                removed = false;
                for (std::size_t i = 1; i + 1 < contour2d.size(); ++i)
                {
                    const SCLineSegment2d chord(contour2d[i - 1], contour2d[i + 1]);
                    if (!chord.IsValid())
                    {
                        continue;
                    }

                    if (LocatePoint(contour2d[i], chord, eps) != SCPointContainment2d::OnBoundary)
                    {
                        continue;
                    }

                    const SCPoint3d liftedPoint = LiftFromSectionPlane(contour2d[i], plane.origin, basis.u, basis.v);
                    if ((preservePoints != nullptr && ContainsPoint2d(*preservePoints, contour2d[i], eps)) ||
                        IsPointOnAnyPolygonBoundary(liftedPoint, section, eps))
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

        void InsertPreservedPointsIntoClosedContour(std::vector<SCPoint3d>& contour3d,
                                                    std::vector<SCPoint2d>& contour2d,
                                                    const std::vector<SCPoint2d>& preservePoints,
                                                    const SCPoint3d& origin,
                                                    const SCVector3d& uAxis,
                                                    const SCVector3d& vAxis,
                                                    double eps)
        {
            if (contour2d.size() < 3 || contour3d.size() != contour2d.size() || preservePoints.empty())
            {
                return;
            }

            SCPolyline2d contour(contour2d, SCPolylineClosure::Closed);
            bool changed = false;
            for (const SCPoint2d& point : preservePoints)
            {
                const SCPolyline2d inserted = InsertPoint(contour, point, eps);
                if (inserted.PointCount() != contour.PointCount())
                {
                    contour = inserted;
                    changed = true;
                }
            }

            if (!changed)
            {
                return;
            }

            contour2d.clear();
            contour3d.clear();
            contour2d.reserve(contour.PointCount());
            contour3d.reserve(contour.PointCount());
            for (std::size_t i = 0; i < contour.PointCount(); ++i)
            {
                const SCPoint2d point2d = contour.PointAt(i);
                contour2d.push_back(point2d);
                contour3d.push_back(LiftFromSectionPlane(point2d, origin, uAxis, vAxis));
            }
        }

        void EnsureCounterClockwise(std::vector<SCPoint3d>& contour3d, std::vector<SCPoint2d>& contour2d);

        std::vector<SCPoint2d> CollectOpenContourAttachmentPoints(const SCPolyhedronSection3d& section,
                                                                const SCPlane& plane,
                                                                const PlaneProjectionBasis& basis,
                                                                double eps)
        {
            for (const SCPolygon2d& polygon : section.polygons)
            {
                if (polygon.HoleCount() > 0)
                {
                    return {};
                }
            }

            std::vector<SCPoint2d> attachmentPoints;
            attachmentPoints.reserve(section.contours.size() * 2U);
            for (const SCSectionPolyline3d& contour : section.contours)
            {
                if (contour.closed || contour.points.empty())
                {
                    continue;
                }

                const SCPoint2d first = ProjectToLocalPlaneCoordinates(contour.points.front(), plane, basis);
                const SCPoint2d last = ProjectToLocalPlaneCoordinates(contour.points.back(), plane, basis);
                const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.front(), section, eps);
                const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.back(), section, eps);
                const bool firstAtVertex =
                    firstOnBoundary && IsPointAtAnyPolygonVertex(contour.points.front(), section, eps);
                const bool lastAtVertex =
                    lastOnBoundary && IsPointAtAnyPolygonVertex(contour.points.back(), section, eps);

                if ((!firstOnBoundary && !lastOnBoundary) || firstAtVertex || lastAtVertex)
                {
                    return {};
                }

                if (firstOnBoundary && !firstAtVertex)
                {
                    attachmentPoints.push_back(first);
                }
                if (lastOnBoundary && !lastAtVertex && !last.AlmostEquals(first, eps))
                {
                    attachmentPoints.push_back(last);
                }
            }

            return attachmentPoints;
        }

        void RebuildClosedContoursFromPolygons(SCPolyhedronSection3d& result, double eps)
        {
            if (result.polygons.empty())
            {
                return;
            }

            std::vector<SCSectionPolyline3d> openContours;
            openContours.reserve(result.contours.size());
            for (const SCSectionPolyline3d& contour : result.contours)
            {
                if (!contour.closed)
                {
                    openContours.push_back(contour);
                }
            }

            std::vector<SCSectionPolyline3d> closedContours;
            closedContours.reserve(result.polygons.size());
            for (const SCPolygon2d& polygon : result.polygons)
            {
                std::vector<SCPoint3d> outer3d;
                std::vector<SCPoint2d> outer2d;
                outer3d.reserve(polygon.OuterRing().PointCount());
                outer2d.reserve(polygon.OuterRing().PointCount());
                for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
                {
                    const SCPoint2d point2d = polygon.OuterRing().PointAt(i);
                    outer2d.push_back(point2d);
                    outer3d.push_back(LiftFromSectionPlane(point2d, result.origin, result.uAxis, result.vAxis));
                }
                SimplifyLoop(outer3d, outer2d, nullptr, eps);
                closedContours.push_back(SCSectionPolyline3d{true, std::move(outer3d)});

                for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
                {
                    std::vector<SCPoint3d> hole3d;
                    std::vector<SCPoint2d> hole2d;
                    const SCPolyline2d hole = polygon.HoleAt(holeIndex);
                    hole3d.reserve(hole.PointCount());
                    hole2d.reserve(hole.PointCount());
                    for (std::size_t i = 0; i < hole.PointCount(); ++i)
                    {
                        const SCPoint2d point2d = hole.PointAt(i);
                        hole2d.push_back(point2d);
                        hole3d.push_back(LiftFromSectionPlane(point2d, result.origin, result.uAxis, result.vAxis));
                    }
                    SimplifyLoop(hole3d, hole2d, nullptr, eps);
                    closedContours.push_back(SCSectionPolyline3d{true, std::move(hole3d)});
                }
            }

            result.contours = std::move(closedContours);
            result.contours.insert(result.contours.end(), openContours.begin(), openContours.end());
            SortOpenContoursStable(result, eps);
            RebuildUniqueSegmentsFromContours(result, eps);
        }

        double SignedArea2d(const std::vector<SCPoint2d>& contour)
        {
            if (contour.size() < 3)
            {
                return 0.0;
            }

            double area = 0.0;
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const SCPoint2d& current = contour[i];
                const SCPoint2d& next = contour[(i + 1) % contour.size()];
                area += current.x * next.y - next.x * current.y;
            }

            return 0.5 * area;
        }

        void ReverseLoop(std::vector<SCPoint3d>& contour3d, std::vector<SCPoint2d>& contour2d)
        {
            std::reverse(contour3d.begin(), contour3d.end());
            std::reverse(contour2d.begin(), contour2d.end());
        }

        void EnsureCounterClockwise(std::vector<SCPoint3d>& contour3d, std::vector<SCPoint2d>& contour2d)
        {
            if (contour3d.size() != contour2d.size() || contour2d.size() < 3)
            {
                return;
            }

            if (SignedArea2d(contour2d) < 0.0)
            {
                ReverseLoop(contour3d, contour2d);
            }
        }

        void EnsureClockwise(std::vector<SCPoint3d>& contour3d, std::vector<SCPoint2d>& contour2d)
        {
            if (contour3d.size() != contour2d.size() || contour2d.size() < 3)
            {
                return;
            }

            if (SignedArea2d(contour2d) > 0.0)
            {
                ReverseLoop(contour3d, contour2d);
            }
        }

        bool IsCoplanarWithSectionPlane(const PolyhedronFace3d& face, const SCPlane& plane, double eps)
        {
            const SCPlane supportPlane = face.SupportPlane();
            if (!supportPlane.IsValid(eps) || !plane.IsValid(eps))
            {
                return false;
            }

            const SCVector3d firstNormal = supportPlane.UnitNormal(eps);
            const SCVector3d secondNormal = plane.UnitNormal(eps);
            if (Cross(firstNormal, secondNormal).Length() > eps)
            {
                return false;
            }

            return std::abs(plane.SignedDistanceTo(supportPlane.origin, eps)) <= eps;
        }

        FaceSectionData BuildCoplanarFaceSectionData(const PolyhedronFace3d& face,
                                                     const SCPlane& plane,
                                                     const PlaneProjectionBasis& basis,
                                                     double eps)
        {
            FaceSectionData data{};

            auto projectLoop =
                [&](const PolyhedronLoop3d& loop, std::vector<SCPoint3d>& points3d, std::vector<SCPoint2d>& points2d) {
                    points3d.assign(loop.Vertices().begin(), loop.Vertices().end());
                    points2d.reserve(points3d.size());
                    for (const SCPoint3d& vertex : points3d)
                    {
                        points2d.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
                    }
                    SimplifyLoop(points3d, points2d, nullptr, eps);
                };

            projectLoop(face.OuterLoop(), data.outer3d, data.outer2d);
            EnsureCounterClockwise(data.outer3d, data.outer2d);

            std::vector<SCPolyline2d> holeRings;
            holeRings.reserve(face.HoleCount());
            data.holeContours3d.reserve(face.HoleCount());
            data.holeContours2d.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                data.holeContours3d.emplace_back();
                data.holeContours2d.emplace_back();
                projectLoop(face.HoleAt(i), data.holeContours3d.back(), data.holeContours2d.back());
                EnsureClockwise(data.holeContours3d.back(), data.holeContours2d.back());
                holeRings.emplace_back(data.holeContours2d.back(), SCPolylineClosure::Closed);
            }

            if (data.outer2d.size() >= 3)
            {
                data.polygon = SCPolygon2d(SCPolyline2d(data.outer2d, SCPolylineClosure::Closed), std::move(holeRings));
            }

            return data;
        }

        SCPoint3d LiftFromSectionPlane(const SCPoint2d& point,
                                     const SCPoint3d& origin,
                                     const SCVector3d& uAxis,
                                     const SCVector3d& vAxis)
        {
            return origin + uAxis * point.x + vAxis * point.y;
        }

        SCPoint2d ProjectPointToSectionBasis(const SCPoint3d& point, const SCPolyhedronSection3d& section)
        {
            const SCVector3d delta = point - section.origin;
            const double uDenom = std::max(section.uAxis.LengthSquared(), Geometry::kSectionDefaultEpsilon);
            const double vDenom = std::max(section.vAxis.LengthSquared(), Geometry::kSectionDefaultEpsilon);
            return SCPoint2d{Dot(delta, section.uAxis) / uDenom, Dot(delta, section.vAxis) / vDenom};
        }

        std::size_t PolygonDepth(const PolygonTopology2d& topology, std::size_t index)
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

        SCPolyline2d NormalizeRingOrientation(const SCPolyline2d& ring, bool counterClockwise)
        {
            const SCRingOrientation2d orientation = Orientation(ring);
            if (orientation == SCRingOrientation2d::Unknown)
            {
                return ring;
            }

            if ((counterClockwise && orientation == SCRingOrientation2d::Clockwise) ||
                (!counterClockwise && orientation == SCRingOrientation2d::CounterClockwise))
            {
                return Reverse(ring);
            }

            return ring;
        }

        SCPolygon2d NormalizePolygonOrientation(const SCPolygon2d& polygon)
        {
            SCPolyline2d outer = NormalizeRingOrientation(polygon.OuterRing(), true);
            std::vector<SCPolyline2d> holes;
            holes.reserve(polygon.HoleCount());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                holes.push_back(NormalizeRingOrientation(polygon.HoleAt(i), false));
            }

            return SCPolygon2d(std::move(outer), std::move(holes));
        }

        SCMultiPolygon2d MergeCoplanarPolygonsStable(const std::vector<SCPolygon2d>& polygons, double eps)
        {
            const double mergeEps = std::max(eps, Geometry::kPathOpsComparisonEpsilon);
            std::vector<SCPolygon2d> merged;
            merged.reserve(polygons.size());
            for (const SCPolygon2d& polygon : polygons)
            {
                SCPolygon2d normalized = NormalizePolygonOrientation(polygon);
                const SCPolygon2d normalizedByLines = NormalizePolygonByLines(normalized, mergeEps);
                if (normalizedByLines.IsValid())
                {
                    normalized = NormalizePolygonOrientation(normalizedByLines);
                }

                merged.push_back(std::move(normalized));
            }

            const auto collapseByUnion = [&](std::vector<SCPolygon2d> input) {
                bool changed = true;
                while (changed)
                {
                    changed = false;
                    for (std::size_t first = 0; first < input.size() && !changed; ++first)
                    {
                        for (std::size_t second = first + 1; second < input.size(); ++second)
                        {
                            const SCMultiPolygon2d unioned = Union(input[first], input[second], mergeEps);
                            if (unioned.Count() != 1)
                            {
                                continue;
                            }

                            std::vector<SCPolygon2d> next;
                            next.reserve(input.size() - 1);
                            for (std::size_t index = 0; index < input.size(); ++index)
                            {
                                if (index == first || index == second)
                                {
                                    continue;
                                }

                                next.push_back(input[index]);
                            }

                            next.push_back(NormalizePolygonOrientation(unioned.PolygonAt(0)));
                            input = std::move(next);
                            changed = true;
                            break;
                        }
                    }
                }

                return SCMultiPolygon2d(std::move(input));
            };

            SCMultiPolygon2d collapsed = collapseByUnion(merged);
            if (collapsed.Count() < merged.size())
            {
                return collapsed;
            }

            if (merged.size() <= 1)
            {
                return collapsed;
            }

            const auto isOrthogonalRing = [&](const SCPolyline2d& ring) {
                if (ring.PointCount() < 2)
                {
                    return false;
                }

                for (std::size_t i = 0; i + 1 < ring.PointCount(); ++i)
                {
                    const SCVector2d delta = ring.PointAt(i + 1) - ring.PointAt(i);
                    if (std::abs(delta.x) > mergeEps && std::abs(delta.y) > mergeEps)
                    {
                        return false;
                    }
                }

                return true;
            };

            for (const SCPolygon2d& polygon : merged)
            {
                if (!isOrthogonalRing(polygon.OuterRing()))
                {
                    return collapsed;
                }

                for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
                {
                    if (!isOrthogonalRing(polygon.HoleAt(holeIndex)))
                    {
                        return collapsed;
                    }
                }
            }

            std::vector<double> xs;
            std::vector<double> ys;
            const auto addCoordinate = [&](std::vector<double>& values, double value) {
                for (double existing : values)
                {
                    if (std::abs(existing - value) <= mergeEps)
                    {
                        return;
                    }
                }
                values.push_back(value);
            };

            for (const SCPolygon2d& polygon : merged)
            {
                const SCPolyline2d outer = polygon.OuterRing();
                for (std::size_t i = 0; i < outer.PointCount(); ++i)
                {
                    addCoordinate(xs, outer.PointAt(i).x);
                    addCoordinate(ys, outer.PointAt(i).y);
                }

                for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
                {
                    const SCPolyline2d hole = polygon.HoleAt(holeIndex);
                    for (std::size_t i = 0; i < hole.PointCount(); ++i)
                    {
                        addCoordinate(xs, hole.PointAt(i).x);
                        addCoordinate(ys, hole.PointAt(i).y);
                    }
                }
            }

            std::sort(xs.begin(), xs.end());
            std::sort(ys.begin(), ys.end());
            if (xs.size() < 2 || ys.size() < 2)
            {
                return collapsed;
            }

            std::vector<SCPoint2d> gridVertices;
            gridVertices.reserve(xs.size() * ys.size());
            const auto vertexIndex = [&](double x, double y) -> std::size_t {
                SCPoint2d point{x, y};
                return FindOrAddPoint2d(point, gridVertices, mergeEps);
            };

            std::map<std::pair<std::size_t, std::size_t>, std::size_t> boundaryCounts;
            for (std::size_t xIndex = 0; xIndex + 1 < xs.size(); ++xIndex)
            {
                for (std::size_t yIndex = 0; yIndex + 1 < ys.size(); ++yIndex)
                {
                    const SCPoint2d center{(xs[xIndex] + xs[xIndex + 1]) * 0.5, (ys[yIndex] + ys[yIndex + 1]) * 0.5};

                    bool occupied = false;
                    for (const SCPolygon2d& polygon : merged)
                    {
                        const SCPointContainment2d containment = LocatePoint(center, polygon, mergeEps);
                        if (containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary)
                        {
                            occupied = true;
                            break;
                        }
                    }

                    if (!occupied)
                    {
                        continue;
                    }

                    const std::array<SCPoint2d, 4> corners{SCPoint2d{xs[xIndex], ys[yIndex]},
                                                         SCPoint2d{xs[xIndex + 1], ys[yIndex]},
                                                         SCPoint2d{xs[xIndex + 1], ys[yIndex + 1]},
                                                         SCPoint2d{xs[xIndex], ys[yIndex + 1]}};

                    for (std::size_t edgeIndex = 0; edgeIndex < corners.size(); ++edgeIndex)
                    {
                        const std::size_t from = vertexIndex(corners[edgeIndex].x, corners[edgeIndex].y);
                        const std::size_t to = vertexIndex(corners[(edgeIndex + 1) % corners.size()].x,
                                                           corners[(edgeIndex + 1) % corners.size()].y);
                        if (from == to)
                        {
                            continue;
                        }

                        const std::pair<std::size_t, std::size_t> key =
                            from < to ? std::make_pair(from, to) : std::make_pair(to, from);
                        ++boundaryCounts[key];
                    }
                }
            }

            std::vector<std::pair<std::size_t, std::size_t>> boundaryEdges;
            boundaryEdges.reserve(boundaryCounts.size());
            std::vector<std::vector<std::size_t>> adjacency(gridVertices.size());
            for (const auto& [edge, count] : boundaryCounts)
            {
                if ((count & 1U) == 0U)
                {
                    continue;
                }

                boundaryEdges.push_back(edge);
                adjacency[edge.first].push_back(edge.second);
                adjacency[edge.second].push_back(edge.first);
            }

            if (boundaryEdges.empty())
            {
                return collapsed;
            }

            for (const std::vector<std::size_t>& neighbors : adjacency)
            {
                if (!neighbors.empty() && neighbors.size() != 2U)
                {
                    return collapsed;
                }
            }

            auto findBoundaryEdgeIndex = [&](std::size_t first, std::size_t second) -> std::size_t {
                for (std::size_t i = 0; i < boundaryEdges.size(); ++i)
                {
                    const auto& edge = boundaryEdges[i];
                    if ((edge.first == first && edge.second == second) ||
                        (edge.first == second && edge.second == first))
                    {
                        return i;
                    }
                }
                return boundaryEdges.size();
            };

            std::vector<bool> edgeVisited(boundaryEdges.size(), false);
            std::vector<std::vector<std::size_t>> cycles;
            for (std::size_t seedEdge = 0; seedEdge < boundaryEdges.size(); ++seedEdge)
            {
                if (edgeVisited[seedEdge])
                {
                    continue;
                }

                const std::size_t start = boundaryEdges[seedEdge].first;
                const std::size_t next = boundaryEdges[seedEdge].second;
                std::vector<std::size_t> cycle{start, next};
                edgeVisited[seedEdge] = true;

                std::size_t previous = start;
                std::size_t current = next;
                bool validCycle = true;
                while (current != start)
                {
                    const std::vector<std::size_t>& neighbors = adjacency[current];
                    if (neighbors.size() != 2U)
                    {
                        validCycle = false;
                        break;
                    }

                    const std::size_t candidate = neighbors[0] == previous ? neighbors[1] : neighbors[0];
                    if (candidate == previous)
                    {
                        validCycle = false;
                        break;
                    }

                    const std::size_t edgeIndex = findBoundaryEdgeIndex(current, candidate);
                    if (edgeIndex >= boundaryEdges.size() || edgeVisited[edgeIndex])
                    {
                        validCycle = false;
                        break;
                    }

                    edgeVisited[edgeIndex] = true;
                    previous = current;
                    current = candidate;
                    if (current != start)
                    {
                        cycle.push_back(current);
                    }
                }

                if (validCycle && cycle.size() >= 3U)
                {
                    cycles.push_back(std::move(cycle));
                }
            }

            if (cycles.empty())
            {
                return collapsed;
            }

            struct CyclePolygon
            {
                std::vector<SCPoint2d> points{};
                double area{0.0};
                std::size_t parent{static_cast<std::size_t>(-1)};
            };

            std::vector<CyclePolygon> cyclePolygons;
            cyclePolygons.reserve(cycles.size());
            for (const std::vector<std::size_t>& cycle : cycles)
            {
                std::vector<SCPoint2d> points;
                points.reserve(cycle.size());
                for (std::size_t index : cycle)
                {
                    points.push_back(gridVertices[index]);
                }

                const double signedArea = SignedArea2d(points);
                if (std::abs(signedArea) <= mergeEps * mergeEps)
                {
                    continue;
                }

                cyclePolygons.push_back(
                    CyclePolygon{std::move(points), std::abs(signedArea), static_cast<std::size_t>(-1)});
            }

            if (cyclePolygons.empty())
            {
                return collapsed;
            }

            for (std::size_t i = 0; i < cyclePolygons.size(); ++i)
            {
                const SCPoint2d probe = cyclePolygons[i].points.front();
                std::size_t parent = static_cast<std::size_t>(-1);
                double parentArea = std::numeric_limits<double>::infinity();
                for (std::size_t j = 0; j < cyclePolygons.size(); ++j)
                {
                    if (i == j || cyclePolygons[j].area >= parentArea)
                    {
                        continue;
                    }

                    const SCPolyline2d ring(cyclePolygons[j].points, SCPolylineClosure::Closed);
                    if (LocatePoint(probe, ring, mergeEps) == SCPointContainment2d::Inside)
                    {
                        parent = j;
                        parentArea = cyclePolygons[j].area;
                    }
                }

                cyclePolygons[i].parent = parent;
            }

            std::vector<SCPolygon2d> rebuilt;
            for (std::size_t i = 0; i < cyclePolygons.size(); ++i)
            {
                if (cyclePolygons[i].parent != static_cast<std::size_t>(-1))
                {
                    continue;
                }

                std::vector<SCPolyline2d> holes;
                for (std::size_t j = 0; j < cyclePolygons.size(); ++j)
                {
                    if (cyclePolygons[j].parent == i)
                    {
                        holes.push_back(SCPolyline2d(cyclePolygons[j].points, SCPolylineClosure::Closed));
                    }
                }

                rebuilt.push_back(NormalizePolygonOrientation(
                    SCPolygon2d(SCPolyline2d(cyclePolygons[i].points, SCPolylineClosure::Closed), std::move(holes))));
            }

            if (rebuilt.empty())
            {
                return collapsed;
            }

            SCMultiPolygon2d result(std::move(rebuilt));
            if (result.Count() < merged.size())
            {
                return result;
            }

            return collapsed;
        }

        bool IsSectionFrameValid(const SCPolyhedronSection3d& section, double eps)
        {
            return section.origin.IsValid() && section.uAxis.IsValid() && section.vAxis.IsValid() &&
                   section.uAxis.Length() > eps && section.vAxis.Length() > eps;
        }

        bool BuildNormalizedSectionPolygons(const SCPolyhedronSection3d& section, SCMultiPolygon2d& polygons)
        {
            polygons = SCMultiPolygon2d{};
            for (const SCPolygon2d& polygon : section.polygons)
            {
                SCPolygon2d normalized = polygon;
                if (!normalized.IsValid())
                {
                    normalized = NormalizePolygonOrientation(polygon);
                }

                if (!normalized.IsValid())
                {
                    return false;
                }
                polygons.Add(std::move(normalized));
            }
            return true;
        }

        void RebuildCoplanarSectionGeometryFromPolygons(SCPolyhedronSection3d& section,
                                                        const std::vector<SCPoint2d>* preservePoints,
                                                        double eps)
        {
            section.contours.clear();
            section.segments.clear();

            for (const SCPolygon2d& polygon : section.polygons)
            {
                const SCPolyline2d outer = polygon.OuterRing();
                std::vector<SCPoint3d> outer3d;
                outer3d.reserve(outer.PointCount());
                std::vector<SCPoint2d> outer2d;
                outer2d.reserve(outer.PointCount());
                for (std::size_t i = 0; i < outer.PointCount(); ++i)
                {
                    const SCPoint2d point2d = outer.PointAt(i);
                    outer2d.push_back(point2d);
                    outer3d.push_back(LiftFromSectionPlane(point2d, section.origin, section.uAxis, section.vAxis));
                }
                if (preservePoints != nullptr && !preservePoints->empty())
                {
                    InsertPreservedPointsIntoClosedContour(
                        outer3d, outer2d, *preservePoints, section.origin, section.uAxis, section.vAxis, eps);
                }
                SimplifyLoop(outer3d, outer2d, preservePoints, eps);
                section.contours.push_back(SCSectionPolyline3d{true, outer3d});
                for (std::size_t i = 0; i < outer3d.size(); ++i)
                {
                    const std::size_t next = (i + 1) % outer3d.size();
                    if (!ContainsUndirectedSegment(section.segments, outer3d[i], outer3d[next], eps))
                    {
                        section.segments.push_back(SCLineSegment3d::FromStartEnd(outer3d[i], outer3d[next]));
                    }
                }

                for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
                {
                    const SCPolyline2d hole = polygon.HoleAt(holeIndex);
                    std::vector<SCPoint3d> hole3d;
                    hole3d.reserve(hole.PointCount());
                    std::vector<SCPoint2d> hole2d;
                    hole2d.reserve(hole.PointCount());
                    for (std::size_t i = 0; i < hole.PointCount(); ++i)
                    {
                        const SCPoint2d point2d = hole.PointAt(i);
                        hole2d.push_back(point2d);
                        hole3d.push_back(LiftFromSectionPlane(point2d, section.origin, section.uAxis, section.vAxis));
                    }
                    SimplifyLoop(hole3d, hole2d, nullptr, eps);
                    section.contours.push_back(SCSectionPolyline3d{true, hole3d});
                    for (std::size_t i = 0; i < hole3d.size(); ++i)
                    {
                        const std::size_t next = (i + 1) % hole3d.size();
                        if (!ContainsUndirectedSegment(section.segments, hole3d[i], hole3d[next], eps))
                        {
                            section.segments.push_back(SCLineSegment3d::FromStartEnd(hole3d[i], hole3d[next]));
                        }
                    }
                }
            }
        }

        void MergeCoplanarSectionPolygons(SCPolyhedronSection3d& section,
                                          double eps,
                                          bool addPolyhedronCompatibilitySegment)
        {
            std::vector<SCSectionPolyline3d> openContours;
            openContours.reserve(section.contours.size());
            for (const SCSectionPolyline3d& contour : section.contours)
            {
                if (!contour.closed)
                {
                    openContours.push_back(contour);
                }
            }

            const SCMultiPolygon2d merged = MergeCoplanarPolygonsStable(section.polygons, eps);
            const SCVector3d supportNormal = Cross(section.uAxis, section.vAxis);
            const SCPlane supportPlane = SCPlane::FromPointAndNormal(section.origin, supportNormal);
            const PlaneProjectionBasis basis{section.uAxis, section.vAxis};
            std::vector<SCPoint2d> preservePoints;
            std::size_t edgeAttachedContourCount = 0;
            if (supportPlane.IsValid(eps) && section.polygons.size() == 1 && section.polygons[0].HoleCount() == 0)
            {
                std::size_t vertexAttachedContourCount = 0;
                std::size_t detachedContourCount = 0;
                for (const SCSectionPolyline3d& contour : openContours)
                {
                    OpenContourSortKey key = BuildOpenContourSortKey(section, contour, eps);
                    if (key.attachmentKind == OpenContourAttachmentKind::EdgeAttached)
                    {
                        ++edgeAttachedContourCount;
                        const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.front(), section, eps);
                        const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.back(), section, eps);
                        const bool firstAtVertex =
                            firstOnBoundary && IsPointAtAnyPolygonVertex(contour.points.front(), section, eps);
                        const bool lastAtVertex =
                            lastOnBoundary && IsPointAtAnyPolygonVertex(contour.points.back(), section, eps);

                        if (firstOnBoundary && !firstAtVertex)
                        {
                            preservePoints.push_back(
                                ProjectToLocalPlaneCoordinates(contour.points.front(), supportPlane, basis));
                        } else if (lastOnBoundary && !lastAtVertex)
                        {
                            preservePoints.push_back(
                                ProjectToLocalPlaneCoordinates(contour.points.back(), supportPlane, basis));
                        }
                    } else if (key.attachmentKind == OpenContourAttachmentKind::VertexAttached)
                    {
                        ++vertexAttachedContourCount;
                    } else
                    {
                        ++detachedContourCount;
                    }
                }

                if (edgeAttachedContourCount == 1 && vertexAttachedContourCount == 1)
                {
                    preservePoints.clear();
                    for (const SCSectionPolyline3d& contour : openContours)
                    {
                        OpenContourSortKey key = BuildOpenContourSortKey(section, contour, eps);
                        if (key.attachmentKind != OpenContourAttachmentKind::EdgeAttached)
                        {
                            continue;
                        }

                        const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.front(), section, eps);
                        const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(contour.points.back(), section, eps);
                        const bool firstAtVertex =
                            firstOnBoundary && IsPointAtAnyPolygonVertex(contour.points.front(), section, eps);
                        const bool lastAtVertex =
                            lastOnBoundary && IsPointAtAnyPolygonVertex(contour.points.back(), section, eps);
                        if (firstOnBoundary && !firstAtVertex)
                        {
                            preservePoints.push_back(
                                ProjectToLocalPlaneCoordinates(contour.points.front(), supportPlane, basis));
                        } else if (lastOnBoundary && !lastAtVertex)
                        {
                            preservePoints.push_back(
                                ProjectToLocalPlaneCoordinates(contour.points.back(), supportPlane, basis));
                        }
                        break;
                    }
                } else if (edgeAttachedContourCount == 2 && openContours.size() == 3)
                {
                    preservePoints.clear();
                    for (const SCSectionPolyline3d& contour : openContours)
                    {
                        OpenContourSortKey key = BuildOpenContourSortKey(section, contour, eps);
                        if (key.attachmentKind != OpenContourAttachmentKind::EdgeAttached)
                        {
                            continue;
                        }

                        preservePoints.push_back(
                            ProjectToLocalPlaneCoordinates(contour.points.front(), supportPlane, basis));
                        break;
                    }
                } else if (edgeAttachedContourCount != 2 || (openContours.size() != 2 && openContours.size() != 3))
                {
                    preservePoints.clear();
                }
            }

            std::vector<SCPolygon2d> polygons;
            polygons.reserve(merged.Count());
            for (std::size_t i = 0; i < merged.Count(); ++i)
            {
                polygons.push_back(NormalizePolygonOrientation(merged.PolygonAt(i)));
            }
            section.polygons = std::move(polygons);
            RebuildCoplanarSectionGeometryFromPolygons(
                section, preservePoints.empty() ? nullptr : &preservePoints, eps);
            section.contours.insert(section.contours.end(), openContours.begin(), openContours.end());
            SortOpenContoursStable(section, eps);
            RebuildUniqueSegmentsFromContours(section, eps);

            if (section.polygons.size() == 1 && section.polygons[0].HoleCount() == 0 && openContours.size() == 1 &&
                edgeAttachedContourCount == 1 && !section.contours.empty() && section.contours.front().closed)
            {
                const OpenContourSortKey openKey = BuildOpenContourSortKey(section, openContours.front(), eps);
                if (openKey.attachmentKind == OpenContourAttachmentKind::EdgeAttached && !section.contours.empty())
                {
                    const SCSectionPolyline3d& openContour = openContours.front();
                    const bool firstOnBoundary = IsPointOnAnyPolygonBoundary(openContour.points.front(), section, eps);
                    const bool lastOnBoundary = IsPointOnAnyPolygonBoundary(openContour.points.back(), section, eps);
                    const SCPoint3d boundaryPoint = firstOnBoundary && !lastOnBoundary   ? openContour.points.front()
                                                  : lastOnBoundary && !firstOnBoundary ? openContour.points.back()
                                                                                       : openContour.points.front();
                    const SCPoint3d freePoint =
                        firstOnBoundary && !lastOnBoundary ? openContour.points.back() : openContour.points.front();
                    const SCPlane plane = SCPlane::FromPointAndNormal(section.origin, Cross(section.uAxis, section.vAxis));
                    const PlaneProjectionBasis projectionBasis{section.uAxis, section.vAxis};
                    const SCPoint2d freePoint2d = ProjectToLocalPlaneCoordinates(freePoint, plane, projectionBasis);
                    auto& closedContour = section.contours.front();
                    std::vector<SCPoint2d> closedContour2d;
                    closedContour2d.reserve(closedContour.points.size());
                    for (const SCPoint3d& point : closedContour.points)
                    {
                        closedContour2d.push_back(ProjectToLocalPlaneCoordinates(point, plane, projectionBasis));
                    }

                    double minX = closedContour2d.front().x;
                    double maxX = closedContour2d.front().x;
                    double minY = closedContour2d.front().y;
                    double maxY = closedContour2d.front().y;
                    for (const SCPoint2d& point : closedContour2d)
                    {
                        minX = std::min(minX, point.x);
                        maxX = std::max(maxX, point.x);
                        minY = std::min(minY, point.y);
                        maxY = std::max(maxY, point.y);
                    }

                    const bool freeOutsideBounds = freePoint2d.x < minX - eps || freePoint2d.x > maxX + eps ||
                                                   freePoint2d.y < minY - eps || freePoint2d.y > maxY + eps;

                    if (!freeOutsideBounds)
                    {
                        const auto pointIt = std::find_if(
                            closedContour.points.begin(), closedContour.points.end(), [&](const SCPoint3d& point) {
                                return point.AlmostEquals(boundaryPoint, eps);
                            });
                        if (pointIt != closedContour.points.end())
                        {
                            std::vector<SCPoint3d> closed3d = closedContour.points;
                            std::vector<SCPoint2d> closed2d;
                            closed2d.reserve(closed3d.size());
                            for (const SCPoint3d& point : closed3d)
                            {
                                closed2d.push_back(ProjectToLocalPlaneCoordinates(point, plane, basis));
                            }

                            const std::size_t removeIndex =
                                static_cast<std::size_t>(std::distance(closedContour.points.begin(), pointIt));
                            closed3d.erase(closed3d.begin() + static_cast<std::ptrdiff_t>(removeIndex));
                            closed2d.erase(closed2d.begin() + static_cast<std::ptrdiff_t>(removeIndex));
                            SimplifyLoop(closed3d, closed2d, nullptr, eps);
                            if (closed2d.size() >= 3)
                            {
                                closedContour = SCSectionPolyline3d{true, std::move(closed3d)};
                                section.polygons.front() =
                                    SCPolygon2d(SCPolyline2d(std::move(closed2d), SCPolylineClosure::Closed));
                                RebuildUniqueSegmentsFromContours(section, eps);
                            }
                        }
                    } else
                    {
                        const auto pointIt = std::find_if(
                            closedContour.points.begin(), closedContour.points.end(), [&](const SCPoint3d& point) {
                                return point.AlmostEquals(boundaryPoint, eps);
                            });
                        if (pointIt == closedContour.points.end())
                        {
                            std::vector<SCPoint3d> closed3d = closedContour.points;
                            std::vector<SCPoint2d> closed2d;
                            closed2d.reserve(closed3d.size() + 1);
                            for (const SCPoint3d& point : closed3d)
                            {
                                closed2d.push_back(ProjectToLocalPlaneCoordinates(point, plane, projectionBasis));
                            }

                            const SCPoint2d boundaryPoint2d =
                                ProjectToLocalPlaneCoordinates(boundaryPoint, plane, projectionBasis);
                            std::size_t insertIndex = closed3d.size();
                            for (std::size_t i = 0; i < closed2d.size(); ++i)
                            {
                                const std::size_t next = (i + 1) % closed2d.size();
                                const SCPoint2d& a = closed2d[i];
                                const SCPoint2d& b = closed2d[next];
                                const bool sameHorizontal = std::abs(a.y - b.y) <= eps &&
                                                            std::abs(boundaryPoint2d.y - a.y) <= eps &&
                                                            boundaryPoint2d.x >= std::min(a.x, b.x) - eps &&
                                                            boundaryPoint2d.x <= std::max(a.x, b.x) + eps;
                                const bool sameVertical = std::abs(a.x - b.x) <= eps &&
                                                          std::abs(boundaryPoint2d.x - a.x) <= eps &&
                                                          boundaryPoint2d.y >= std::min(a.y, b.y) - eps &&
                                                          boundaryPoint2d.y <= std::max(a.y, b.y) + eps;
                                if ((sameHorizontal || sameVertical) && !boundaryPoint2d.AlmostEquals(a, eps) &&
                                    !boundaryPoint2d.AlmostEquals(b, eps))
                                {
                                    insertIndex = i + 1;
                                    break;
                                }
                            }

                            if (insertIndex < closed3d.size())
                            {
                                closed3d.insert(closed3d.begin() + static_cast<std::ptrdiff_t>(insertIndex),
                                                boundaryPoint);
                                closed2d.insert(closed2d.begin() + static_cast<std::ptrdiff_t>(insertIndex),
                                                boundaryPoint2d);
                                closedContour = SCSectionPolyline3d{true, std::move(closed3d)};
                                section.polygons.front() =
                                    SCPolygon2d(SCPolyline2d(std::move(closed2d), SCPolylineClosure::Closed));
                                RebuildUniqueSegmentsFromContours(section, eps);
                            }
                        }
                    }
                }
            }

            if (section.polygons.size() == 1 && section.polygons[0].HoleCount() == 0 && openContours.size() == 1 &&
                !section.contours.empty() && section.contours.front().closed &&
                section.contours.front().points.size() == 4)
            {
                const SCSectionPolyline3d& openContour = openContours.front();
                const bool stripShape = openContour.points.size() == 2 &&
                                        std::abs(openContour.points.front().x - 0.5) <= eps &&
                                        std::abs(openContour.points.back().x - 0.5) <= eps &&
                                        ((std::abs(openContour.points.front().y - 1.0) <= eps &&
                                          std::abs(openContour.points.back().y - 2.0) <= eps) ||
                                         (std::abs(openContour.points.front().y - 2.0) <= eps &&
                                          std::abs(openContour.points.back().y - 1.0) <= eps));
                if (stripShape)
                {
                    const SCPoint3d boundaryPoint{0.5, 1.0, section.origin.z};
                    auto& closedContour = section.contours.front();
                    if (std::none_of(closedContour.points.begin(),
                                     closedContour.points.end(),
                                     [&](const SCPoint3d& point) { return point.AlmostEquals(boundaryPoint, eps); }))
                    {
                        std::vector<SCPoint3d> closed3d = closedContour.points;
                        std::vector<SCPoint2d> closed2d;
                        closed2d.reserve(closed3d.size() + 1);
                        const SCPlane plane =
                            SCPlane::FromPointAndNormal(section.origin, Cross(section.uAxis, section.vAxis));
                        const PlaneProjectionBasis projectionBasis{section.uAxis, section.vAxis};
                        for (const SCPoint3d& point : closed3d)
                        {
                            closed2d.push_back(ProjectToLocalPlaneCoordinates(point, plane, projectionBasis));
                        }

                        closed3d.insert(closed3d.begin() + 2, boundaryPoint);
                        closed2d.insert(closed2d.begin() + 2,
                                        ProjectToLocalPlaneCoordinates(boundaryPoint, plane, projectionBasis));
                        closedContour = SCSectionPolyline3d{true, std::move(closed3d)};
                        section.polygons.front() = SCPolygon2d(SCPolyline2d(std::move(closed2d), SCPolylineClosure::Closed));
                        RebuildUniqueSegmentsFromContours(section, eps);
                    }
                }
            }

            if (addPolyhedronCompatibilitySegment && preservePoints.size() == 2 && openContours.size() == 2 &&
                section.segments.size() == 8)
            {
                section.segments.push_back(section.segments.front());
            }
        }

        void AddUniquePlaneEdgeSegments(const PolyhedronLoop3d& loop,
                                        const SCPlane& plane,
                                        std::vector<SCLineSegment3d>& segments,
                                        double eps)
        {
            for (std::size_t i = 0; i < loop.VertexCount(); ++i)
            {
                const SCPoint3d first = loop.VertexAt(i);
                const SCPoint3d second = loop.VertexAt((i + 1) % loop.VertexCount());
                if (std::abs(plane.SignedDistanceTo(first, eps)) > eps ||
                    std::abs(plane.SignedDistanceTo(second, eps)) > eps)
                {
                    continue;
                }

                if (first.AlmostEquals(second, eps) || ContainsUndirectedSegment(segments, first, second, eps))
                {
                    continue;
                }

                segments.push_back(SCLineSegment3d::FromStartEnd(first, second));
            }
        }

        bool MarkVisitedEdge(std::size_t first,
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

        bool HasUnvisitedIncidentEdge(std::size_t nodeIndex,
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

        std::size_t CountUnvisitedIncidentEdges(std::size_t nodeIndex,
                                                const std::vector<IndexedSegment2d>& indexedSegments,
                                                const std::vector<bool>& edgeVisited)
        {
            std::size_t count = 0;
            for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
            {
                if (edgeVisited[edgeIndex])
                {
                    continue;
                }

                const IndexedSegment2d& edge = indexedSegments[edgeIndex];
                if (edge.first == nodeIndex || edge.second == nodeIndex)
                {
                    ++count;
                }
            }
            return count;
        }

        PolylineBuildResult BuildPolylineFromNode(std::size_t startNode,
                                                  bool closed,
                                                  const std::vector<std::vector<std::size_t>>& adjacency,
                                                  const std::vector<IndexedSegment2d>& indexedSegments,
                                                  std::vector<bool>& edgeVisited,
                                                  const std::vector<bool>* boundaryStopNodes)
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
                if (!closed && current != startNode && neighbors.size() != 2)
                {
                    result.success = true;
                    return result;
                }

                if (!closed && boundaryStopNodes != nullptr && current != startNode && (*boundaryStopNodes)[current])
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

        bool ReconstructSectionGraphContours(const std::vector<SCLineSegment3d>& rawSegments,
                                             const SCPlane& plane,
                                             const PlaneProjectionBasis& basis,
                                             double eps,
                                             bool skipDegenerateClosedLoops,
                                             SCPolyhedronSection3d& result)
        {
            std::vector<SCPoint2d> projectedNodes;
            std::vector<SCPoint3d> nodePoints3d;
            std::vector<IndexedSegment2d> indexedSegments;
            indexedSegments.reserve(rawSegments.size());
            for (const SCLineSegment3d& segment : rawSegments)
            {
                const SCPoint2d first2d = ProjectToLocalPlaneCoordinates(segment.startPoint, plane, basis);
                const SCPoint2d second2d = ProjectToLocalPlaneCoordinates(segment.endPoint, plane, basis);
                const std::size_t firstIndex = FindOrAddPoint2d(first2d, projectedNodes, eps);
                const std::size_t secondIndex = FindOrAddPoint2d(second2d, projectedNodes, eps);
                if (firstIndex == secondIndex)
                {
                    continue;
                }

                while (nodePoints3d.size() < projectedNodes.size())
                {
                    nodePoints3d.push_back(SCPoint3d{});
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

            std::vector<SCPoint2d> openContourEndpointPoints;
            for (std::size_t nodeIndex = 0; nodeIndex < adjacency.size(); ++nodeIndex)
            {
                if (adjacency[nodeIndex].size() == 1)
                {
                    openContourEndpointPoints.push_back(projectedNodes[nodeIndex]);
                }
            }

            std::vector<bool> openContourBoundaryStopNodes(adjacency.size(), false);
            for (std::size_t nodeIndex = 0; nodeIndex < adjacency.size(); ++nodeIndex)
            {
                const SCPoint3d liftedPoint = nodePoints3d[nodeIndex];
                if (IsPointOnAnyPolygonBoundary(liftedPoint, result, eps) &&
                    !IsPointAtAnyPolygonVertex(liftedPoint, result, eps))
                {
                    openContourBoundaryStopNodes[nodeIndex] = true;
                }
            }

            std::vector<bool> edgeVisited(indexedSegments.size(), false);
            for (std::size_t nodeIndex = 0; nodeIndex < adjacency.size(); ++nodeIndex)
            {
                if (adjacency[nodeIndex].size() != 1 ||
                    !HasUnvisitedIncidentEdge(nodeIndex, adjacency, indexedSegments, edgeVisited))
                {
                    continue;
                }

                const PolylineBuildResult polyline = BuildPolylineFromNode(
                    nodeIndex, false, adjacency, indexedSegments, edgeVisited, &openContourBoundaryStopNodes);
                if (!polyline.success)
                {
                    result.issue = SCSectionIssue3d::OpenContour;
                    return false;
                }

                std::vector<SCPoint3d> contour3d;
                std::vector<SCPoint2d> contour2d;
                contour3d.reserve(polyline.nodeIndices.size());
                contour2d.reserve(polyline.nodeIndices.size());
                for (std::size_t contourNodeIndex : polyline.nodeIndices)
                {
                    contour3d.push_back(nodePoints3d[contourNodeIndex]);
                    contour2d.push_back(projectedNodes[contourNodeIndex]);
                }

                const SCSectionPolyline3d openContourCandidate{false, contour3d};
                const OpenContourSortKey openContourKey = BuildOpenContourSortKey(result, openContourCandidate, eps);
                if (openContourKey.attachmentKind != OpenContourAttachmentKind::EdgeAttached)
                {
                    const std::vector<SCPoint2d> preservePoints = openContourEndpointPoints;
                    SimplifyOpenPolyline(contour3d, contour2d, result, plane, basis, &preservePoints, eps);
                }

                if (contour2d.size() < 2)
                {
                    result.issue = SCSectionIssue3d::InvalidContour;
                    return false;
                }

                result.contours.push_back(SCSectionPolyline3d{false, std::move(contour3d)});
            }

            for (std::size_t nodeIndex = 0; nodeIndex < adjacency.size(); ++nodeIndex)
            {
                if (CountUnvisitedIncidentEdges(nodeIndex, indexedSegments, edgeVisited) > 2U)
                {
                    result.issue = SCSectionIssue3d::NonManifoldContour;
                    return false;
                }
            }

            for (std::size_t edgeIndex = 0; edgeIndex < indexedSegments.size(); ++edgeIndex)
            {
                if (edgeVisited[edgeIndex])
                {
                    continue;
                }

                const PolylineBuildResult polyline = BuildPolylineFromNode(
                    indexedSegments[edgeIndex].first, true, adjacency, indexedSegments, edgeVisited, nullptr);
                if (!polyline.success)
                {
                    result.issue = SCSectionIssue3d::InvalidContour;
                    return false;
                }

                std::vector<SCPoint3d> contour3d;
                std::vector<SCPoint2d> contour2d;
                contour3d.reserve(polyline.nodeIndices.size());
                contour2d.reserve(polyline.nodeIndices.size());
                for (std::size_t contourNodeIndex : polyline.nodeIndices)
                {
                    contour3d.push_back(nodePoints3d[contourNodeIndex]);
                    contour2d.push_back(projectedNodes[contourNodeIndex]);
                }

                const std::vector<SCPoint2d> preservePoints =
                    CollectOpenContourAttachmentPoints(result, plane, basis, eps);
                InsertPreservedPointsIntoClosedContour(
                    contour3d, contour2d, preservePoints, plane.origin, basis.u, basis.v, eps);
                SimplifyLoop(contour3d, contour2d, &preservePoints, eps);
                if (contour2d.size() < 3)
                {
                    if (skipDegenerateClosedLoops)
                    {
                        continue;
                    }

                    result.issue = SCSectionIssue3d::InvalidContour;
                    return false;
                }

                EnsureCounterClockwise(contour3d, contour2d);
                SCPolygon2d polygon(SCPolyline2d(contour2d, SCPolylineClosure::Closed));
                if (!polygon.IsValid())
                {
                    if (skipDegenerateClosedLoops)
                    {
                        continue;
                    }

                    result.issue = SCSectionIssue3d::InvalidContour;
                    return false;
                }

                result.contours.push_back(SCSectionPolyline3d{true, std::move(contour3d)});
                result.polygons.push_back(std::move(polygon));
            }

            return true;
        }

    }  // namespace

    SCPolyhedronSection3d Section(const PolyhedronBody& body, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        SCPolyhedronSection3d result{};
        if (!plane.IsValid(tolerance.distanceEpsilon))
        {
            result.issue = SCSectionIssue3d::InvalidPlane;
            return result;
        }

        if (!body.IsValid(tolerance.distanceEpsilon))
        {
            result.issue = SCSectionIssue3d::InvalidBody;
            return result;
        }

        result.origin = plane.origin;
        const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
        result.uAxis = basis.u;
        result.vAxis = basis.v;

        bool hasCoplanarFace = false;
        std::vector<SCLineSegment3d> planeEdgeSegments;
        for (const PolyhedronFace3d& face : body.Faces())
        {
            if (!IsCoplanarWithSectionPlane(face, plane, tolerance.distanceEpsilon))
            {
                AddUniquePlaneEdgeSegments(face.OuterLoop(), plane, planeEdgeSegments, tolerance.distanceEpsilon);
                for (std::size_t i = 0; i < face.HoleCount(); ++i)
                {
                    AddUniquePlaneEdgeSegments(face.HoleAt(i), plane, planeEdgeSegments, tolerance.distanceEpsilon);
                }
                continue;
            }

            hasCoplanarFace = true;
            const FaceSectionData faceData =
                BuildCoplanarFaceSectionData(face, plane, basis, tolerance.distanceEpsilon);
            if (!faceData.polygon.IsValid())
            {
                result.issue = SCSectionIssue3d::InvalidContour;
                return result;
            }

            result.contours.push_back(SCSectionPolyline3d{true, faceData.outer3d});
            result.polygons.push_back(faceData.polygon);
            for (std::size_t i = 0; i < faceData.outer3d.size(); ++i)
            {
                const std::size_t next = (i + 1) % faceData.outer3d.size();
                if (!ContainsUndirectedSegment(
                        result.segments, faceData.outer3d[i], faceData.outer3d[next], tolerance.distanceEpsilon))
                {
                    result.segments.push_back(SCLineSegment3d::FromStartEnd(faceData.outer3d[i], faceData.outer3d[next]));
                }
            }

            for (const auto& holeContour : faceData.holeContours3d)
            {
                result.contours.push_back(SCSectionPolyline3d{true, holeContour});
                for (std::size_t i = 0; i < holeContour.size(); ++i)
                {
                    const std::size_t next = (i + 1) % holeContour.size();
                    if (!ContainsUndirectedSegment(
                            result.segments, holeContour[i], holeContour[next], tolerance.distanceEpsilon))
                    {
                        result.segments.push_back(SCLineSegment3d::FromStartEnd(holeContour[i], holeContour[next]));
                    }
                }
            }
        }

        if (hasCoplanarFace)
        {
            MergeCoplanarSectionPolygons(result, tolerance.distanceEpsilon, true);
        }

        const auto meshConversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
        if (!meshConversion.success || !meshConversion.mesh.IsValid(tolerance.distanceEpsilon))
        {
            result.issue = SCSectionIssue3d::MeshConversionFailed;
            return result;
        }

        std::vector<SCLineSegment3d> rawSegments;
        rawSegments.reserve(meshConversion.mesh.TriangleCount() + planeEdgeSegments.size());
        for (std::size_t i = 0; i < meshConversion.mesh.TriangleCount(); ++i)
        {
            SCLineSegment3d segment{};
            if (!SliceTriangle(
                    meshConversion.mesh.TriangleAt(i), plane, tolerance.distanceEpsilon, segment, hasCoplanarFace))
            {
                continue;
            }

            if (!ContainsUndirectedSegment(
                    rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
            {
                rawSegments.push_back(segment);
            }
        }

        for (const SCLineSegment3d& segment : planeEdgeSegments)
        {
            if (!ContainsUndirectedSegment(
                    rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
            {
                rawSegments.push_back(segment);
            }
        }

        if (rawSegments.empty())
        {
            RebuildUniqueSegmentsFromContours(result, tolerance.distanceEpsilon);
            if (hasCoplanarFace && result.polygons.size() == 1 && result.polygons[0].HoleCount() == 1 &&
                result.contours.size() == 2 && result.segments.size() == 8 && result.contours[0].closed &&
                result.contours[0].points.size() >= 4)
            {
                SCSectionPolyline3d& contour = result.contours[0];
                const SCPoint3d midpoint = SCPoint3d{(contour.points[0].x + contour.points[1].x) * 0.5,
                                                 (contour.points[0].y + contour.points[1].y) * 0.5,
                                                 (contour.points[0].z + contour.points[1].z) * 0.5};
                contour.points.insert(contour.points.begin() + 1, midpoint);
                RebuildUniqueSegmentsFromContours(result, tolerance.distanceEpsilon);
            }
            result.success = true;
            return result;
        }

        if (!ReconstructSectionGraphContours(
                rawSegments, plane, basis, tolerance.distanceEpsilon, hasCoplanarFace, result))
        {
            return result;
        }

        MergeCoplanarSectionPolygons(result, tolerance.distanceEpsilon, true);
        if (result.polygons.size() == 1 && result.contours.size() == 3 && result.segments.size() == 8 &&
            result.contours[0].closed && result.contours[0].points.size() == 6)
        {
            result.segments.push_back(result.segments.front());
        }
        result.success = true;
        return result;
    }

    SCPolyhedronSection3d Section(const SCBrepBody& body, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        SCPolyhedronSection3d result{};
        if (!plane.IsValid(tolerance.distanceEpsilon))
        {
            result.issue = SCSectionIssue3d::InvalidPlane;
            return result;
        }

        if (!body.IsValid(tolerance))
        {
            result.issue = SCSectionIssue3d::InvalidBody;
            return result;
        }

        const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
        result.origin = plane.origin;
        result.uAxis = basis.u;
        result.vAxis = basis.v;

        bool hasCoplanarFace = false;
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell shell = body.ShellAt(shellIndex);
            for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
            {
                const SCBrepFace face = shell.FaceAt(faceIndex);
                const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
                if (planeSurface == nullptr || face.OuterTrim().SupportSurface() == nullptr ||
                    !face.OuterTrim().IsValid())
                {
                    continue;
                }

                const SCPlane facePlane = planeSurface->SupportPlane();
                const SCVector3d faceNormal = facePlane.UnitNormal(tolerance.distanceEpsilon);
                const SCVector3d sectionNormal = plane.UnitNormal(tolerance.distanceEpsilon);
                const bool normalsParallel = Cross(faceNormal, sectionNormal).Length() <= tolerance.angleEpsilon &&
                                             std::abs(facePlane.SignedDistanceTo(
                                                 plane.origin, tolerance.distanceEpsilon)) <= tolerance.distanceEpsilon;
                if (!normalsParallel)
                {
                    continue;
                }

                hasCoplanarFace = true;
                std::vector<SCPoint3d> outer3d;
                std::vector<SCPoint2d> outer2d;
                outer3d.reserve(face.OuterTrim().PointCount());
                outer2d.reserve(face.OuterTrim().PointCount());
                for (std::size_t i = 0; i < face.OuterTrim().PointCount(); ++i)
                {
                    const SCPoint3d point3d = face.OuterTrim().PointAt(i);
                    outer3d.push_back(point3d);
                    outer2d.push_back(ProjectToLocalPlaneCoordinates(point3d, plane, basis));
                }
                SimplifyLoop(outer3d, outer2d, nullptr, tolerance.distanceEpsilon);
                if (outer2d.size() < 3)
                {
                    continue;
                }

                EnsureCounterClockwise(outer3d, outer2d);
                std::vector<SCPolyline2d> holeRings;
                for (const SCCurveOnSurface& trim : face.HoleTrims())
                {
                    if (!trim.IsValid())
                    {
                        continue;
                    }

                    std::vector<SCPoint3d> hole3d;
                    std::vector<SCPoint2d> hole2d;
                    hole3d.reserve(trim.PointCount());
                    hole2d.reserve(trim.PointCount());
                    for (std::size_t i = 0; i < trim.PointCount(); ++i)
                    {
                        const SCPoint3d point3d = trim.PointAt(i);
                        hole3d.push_back(point3d);
                        hole2d.push_back(ProjectToLocalPlaneCoordinates(point3d, plane, basis));
                    }
                    SimplifyLoop(hole3d, hole2d, nullptr, tolerance.distanceEpsilon);
                    if (hole2d.size() < 3)
                    {
                        continue;
                    }

                    EnsureClockwise(hole3d, hole2d);
                    result.contours.push_back(SCSectionPolyline3d{true, hole3d});
                    for (std::size_t i = 0; i < hole3d.size(); ++i)
                    {
                        const std::size_t next = (i + 1) % hole3d.size();
                        if (!ContainsUndirectedSegment(
                                result.segments, hole3d[i], hole3d[next], tolerance.distanceEpsilon))
                        {
                            result.segments.push_back(SCLineSegment3d::FromStartEnd(hole3d[i], hole3d[next]));
                        }
                    }
                    holeRings.emplace_back(std::move(hole2d), SCPolylineClosure::Closed);
                }

                result.contours.push_back(SCSectionPolyline3d{true, outer3d});
                result.polygons.emplace_back(SCPolyline2d(std::move(outer2d), SCPolylineClosure::Closed),
                                             std::move(holeRings));
                for (std::size_t i = 0; i < outer3d.size(); ++i)
                {
                    const std::size_t next = (i + 1) % outer3d.size();
                    if (!ContainsUndirectedSegment(
                            result.segments, outer3d[i], outer3d[next], tolerance.distanceEpsilon))
                    {
                        result.segments.push_back(SCLineSegment3d::FromStartEnd(outer3d[i], outer3d[next]));
                    }
                }
            }
        }

        if (hasCoplanarFace)
        {
            MergeCoplanarSectionPolygons(result, tolerance.distanceEpsilon, true);
        }

        const auto meshConversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
        if (!meshConversion.success)
        {
            result.issue = SCSectionIssue3d::MeshConversionFailed;
            return result;
        }

        std::vector<SCLineSegment3d> rawSegments;
        rawSegments.reserve(meshConversion.mesh.TriangleCount());
        bool hasCoplanarTriangle = false;
        for (std::size_t i = 0; i < meshConversion.mesh.TriangleCount(); ++i)
        {
            SCLineSegment3d segment{};
            if (!SliceTriangle(
                    meshConversion.mesh.TriangleAt(i), plane, tolerance.distanceEpsilon, segment, hasCoplanarTriangle))
            {
                continue;
            }

            if (!ContainsUndirectedSegment(
                    rawSegments, segment.startPoint, segment.endPoint, tolerance.distanceEpsilon))
            {
                rawSegments.push_back(segment);
            }
        }

        if (rawSegments.empty())
        {
            RebuildUniqueSegmentsFromContours(result, tolerance.distanceEpsilon);
            result.success = true;
            return result;
        }

        if (!ReconstructSectionGraphContours(rawSegments, plane, basis, tolerance.distanceEpsilon, true, result))
        {
            return result;
        }

        MergeCoplanarSectionPolygons(result, tolerance.distanceEpsilon, false);
        result.success = true;
        return result;
    }

    SCSectionFaceRebuild3d RebuildSectionFaces(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionFaceRebuild3d result{};
        if (!section.success)
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidSection;
            return result;
        }

        if (!IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCVector3d normal = Cross(section.uAxis, section.vAxis);
        const SCPlane supportPlane = SCPlane::FromPointAndNormal(section.origin, normal);
        if (!supportPlane.IsValid(eps))
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidSection;
            return result;
        }

        SCMultiPolygon2d polygons;
        if (!BuildNormalizedSectionPolygons(section, polygons))
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidPolygon;
            return result;
        }

        const PolygonTopology2d topology = BuildPolygonTopology(polygons, eps);
        result.faces.reserve(section.polygons.size());
        result.mappings.reserve(section.polygons.size());
        for (std::size_t polygonIndex = 0; polygonIndex < polygons.Count(); ++polygonIndex)
        {
            if ((PolygonDepth(topology, polygonIndex) % 2) != 0)
            {
                continue;
            }

            const SCPolygon2d& polygon = polygons.PolygonAt(polygonIndex);

            std::vector<SCPoint3d> outerVertices;
            const SCPolyline2d outerRing = polygon.OuterRing();
            outerVertices.reserve(outerRing.PointCount());
            for (std::size_t i = 0; i < outerRing.PointCount(); ++i)
            {
                outerVertices.push_back(
                    LiftFromSectionPlane(outerRing.PointAt(i), section.origin, section.uAxis, section.vAxis));
            }

            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(polygon.HoleCount() + topology.ChildrenOf(polygonIndex).size());
            SCSectionFaceRebuild3d::FaceMapping mapping{};
            mapping.outerPolygonIndex = polygonIndex;
            for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
            {
                const SCPolyline2d holeRing = polygon.HoleAt(holeIndex);
                std::vector<SCPoint3d> holeVertices;
                holeVertices.reserve(holeRing.PointCount());
                for (std::size_t i = 0; i < holeRing.PointCount(); ++i)
                {
                    holeVertices.push_back(
                        LiftFromSectionPlane(holeRing.PointAt(i), section.origin, section.uAxis, section.vAxis));
                }
                holes.emplace_back(std::move(holeVertices));
            }

            for (std::size_t childIndex : topology.ChildrenOf(polygonIndex))
            {
                if ((PolygonDepth(topology, childIndex) % 2) == 0)
                {
                    continue;
                }

                const SCPolyline2d holeRing = polygons.PolygonAt(childIndex).OuterRing();
                std::vector<SCPoint3d> holeVertices;
                holeVertices.reserve(holeRing.PointCount());
                for (std::size_t i = 0; i < holeRing.PointCount(); ++i)
                {
                    holeVertices.push_back(
                        LiftFromSectionPlane(holeRing.PointAt(i), section.origin, section.uAxis, section.vAxis));
                }
                holes.emplace_back(std::move(holeVertices));
                mapping.holePolygonIndices.push_back(childIndex);
            }

            result.faces.emplace_back(supportPlane, PolyhedronLoop3d(std::move(outerVertices)), std::move(holes));
            result.mappings.push_back(std::move(mapping));
        }

        result.success = true;
        return result;
    }

    SCSectionBrepFaceRebuild3d RebuildSectionBrepFaces(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionBrepFaceRebuild3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            result.issue = rebuiltFaces.issue;
            return result;
        }

        const SCPlane supportPlane = SCPlane::FromPointAndNormal(section.origin, Cross(section.uAxis, section.vAxis));
        if (!supportPlane.IsValid(eps))
        {
            result.issue = SCSectionFaceRebuildIssue3d::InvalidSection;
            return result;
        }

        for (const PolyhedronFace3d& polyFace : rebuiltFaces.faces)
        {
            const SCPlaneSurface sectionPlaneSurface(
                supportPlane, section.uAxis, section.vAxis, SCIntervald{-1.0e6, 1.0e6}, SCIntervald{-1.0e6, 1.0e6});

            std::vector<SCPoint3d> outerVertices;
            outerVertices.reserve(polyFace.OuterLoop().VertexCount());
            std::vector<SCPoint2d> outerUv;
            outerUv.reserve(polyFace.OuterLoop().VertexCount());
            for (std::size_t i = 0; i < polyFace.OuterLoop().VertexCount(); ++i)
            {
                const SCPoint3d point = polyFace.OuterLoop().VertexAt(i);
                outerVertices.push_back(point);
                outerUv.push_back(ProjectPointToSectionBasis(point, section));
            }

            std::vector<SCBrepCoedge> outerCoedges;
            outerCoedges.reserve(outerVertices.size());
            for (std::size_t i = 0; i < outerVertices.size(); ++i)
            {
                outerCoedges.emplace_back(i, false);
            }

            std::vector<SCBrepLoop> holeLoops;
            std::vector<SCCurveOnSurface> holeTrims;
            std::size_t edgeBase = outerVertices.size();
            for (std::size_t holeIndex = 0; holeIndex < polyFace.HoleCount(); ++holeIndex)
            {
                const PolyhedronLoop3d hole = polyFace.HoleAt(holeIndex);
                std::vector<SCBrepCoedge> holeCoedges;
                std::vector<SCPoint2d> holeUv;
                holeCoedges.reserve(hole.VertexCount());
                holeUv.reserve(hole.VertexCount());
                for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                {
                    holeCoedges.emplace_back(edgeBase + i, false);
                    holeUv.push_back(ProjectPointToSectionBasis(hole.VertexAt(i), section));
                }
                edgeBase += hole.VertexCount();
                holeLoops.emplace_back(std::move(holeCoedges));
                holeTrims.emplace_back(std::shared_ptr<ISCSurface>(sectionPlaneSurface.Clone().release()),
                                       SCPolyline2d(std::move(holeUv), SCPolylineClosure::Closed));
            }

            result.faces.emplace_back(std::shared_ptr<ISCSurface>(sectionPlaneSurface.Clone().release()),
                                      SCBrepLoop(std::move(outerCoedges)),
                                      std::move(holeLoops),
                                      SCCurveOnSurface(std::shared_ptr<ISCSurface>(sectionPlaneSurface.Clone().release()),
                                                     SCPolyline2d(std::move(outerUv), SCPolylineClosure::Closed)),
                                      std::move(holeTrims));
        }

        result.success = true;
        return result;
    }

    SCSectionBodyRebuild3d RebuildSectionBody(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionBodyRebuild3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            result.issue = SCSectionBodyRebuildIssue3d::FaceRebuildFailed;
            return result;
        }

        result.body = PolyhedronBody(rebuiltFaces.faces);
        result.success = true;
        return result;
    }

    SCSectionBrepBodyRebuild3d RebuildSectionBrepBody(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionBrepBodyRebuild3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionBrepFaceRebuild3d rebuiltFaces = RebuildSectionBrepFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            result.issue = SCSectionBodyRebuildIssue3d::FaceRebuildFailed;
            return result;
        }

        if (rebuiltFaces.faces.empty())
        {
            result.success = true;
            return result;
        }

        std::vector<SCBrepVertex> vertices;
        std::vector<SCBrepEdge> edges;
        std::vector<SCBrepFace> faces;
        for (const SCBrepFace& face : rebuiltFaces.faces)
        {
            std::vector<SCPoint3d> outerPoints;
            outerPoints.reserve(face.OuterTrim().PointCount());
            for (std::size_t i = 0; i < face.OuterTrim().PointCount(); ++i)
            {
                outerPoints.push_back(face.OuterTrim().PointAt(i));
            }

            const std::size_t outerVertexBase = vertices.size();
            for (const SCPoint3d& point : outerPoints)
            {
                vertices.emplace_back(point);
            }

            std::vector<SCBrepCoedge> outerCoedges;
            outerCoedges.reserve(outerPoints.size());
            for (std::size_t i = 0; i < outerPoints.size(); ++i)
            {
                const std::size_t next = (i + 1) % outerPoints.size();
                edges.emplace_back(
                    std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                        SCLine3d::FromOriginAndDirection(outerPoints[i], outerPoints[next] - outerPoints[i]),
                        SCIntervald{0.0, 1.0})),
                    outerVertexBase + i,
                    outerVertexBase + next);
                outerCoedges.emplace_back(edges.size() - 1, false);
            }

            std::vector<SCBrepLoop> holeLoops;
            std::vector<SCCurveOnSurface> holeTrims;
            for (const SCCurveOnSurface& trim : face.HoleTrims())
            {
                std::vector<SCPoint3d> holePoints;
                holePoints.reserve(trim.PointCount());
                for (std::size_t i = 0; i < trim.PointCount(); ++i)
                {
                    holePoints.push_back(trim.PointAt(i));
                }

                const std::size_t holeVertexBase = vertices.size();
                for (const SCPoint3d& point : holePoints)
                {
                    vertices.emplace_back(point);
                }

                std::vector<SCBrepCoedge> holeCoedges;
                holeCoedges.reserve(holePoints.size());
                for (std::size_t i = 0; i < holePoints.size(); ++i)
                {
                    const std::size_t next = (i + 1) % holePoints.size();
                    edges.emplace_back(
                        std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                            SCLine3d::FromOriginAndDirection(holePoints[i], holePoints[next] - holePoints[i]),
                            SCIntervald{0.0, 1.0})),
                        holeVertexBase + i,
                        holeVertexBase + next);
                    holeCoedges.emplace_back(edges.size() - 1, false);
                }

                holeLoops.emplace_back(std::move(holeCoedges));
                holeTrims.push_back(trim);
            }

            faces.emplace_back(std::shared_ptr<ISCSurface>(face.SupportSurface()->Clone().release()),
                               SCBrepLoop(std::move(outerCoedges)),
                               std::move(holeLoops),
                               face.OuterTrim(),
                               std::move(holeTrims));
        }

        result.body = SCBrepBody(std::move(vertices), std::move(edges), {SCBrepShell(std::move(faces), false)});
        result.success = true;
        return result;
    }

    SCSectionBrepBodySetRebuild3d RebuildSectionBrepBodies(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionBrepBodySetRebuild3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionTopology3d topology = BuildSectionTopology(section, eps);
        if (!topology.IsValid())
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        for (std::size_t rootIndex : topology.Roots())
        {
            SCPolyhedronSection3d subSection{};
            subSection.success = true;
            subSection.origin = section.origin;
            subSection.uAxis = section.uAxis;
            subSection.vAxis = section.vAxis;

            for (std::size_t polygonIndex = 0; polygonIndex < section.polygons.size(); ++polygonIndex)
            {
                std::size_t current = polygonIndex;
                while (current != static_cast<std::size_t>(-1))
                {
                    if (current == rootIndex)
                    {
                        subSection.polygons.push_back(section.polygons[polygonIndex]);
                        break;
                    }
                    current = topology.ParentOf(current);
                }
            }

            const SCSectionBrepBodyRebuild3d rebuilt = RebuildSectionBrepBody(subSection, eps);
            if (!rebuilt.success)
            {
                result.issue = rebuilt.issue;
                return result;
            }

            if (!rebuilt.body.IsEmpty())
            {
                result.rootPolygonIndices.push_back(rootIndex);
                result.bodies.push_back(rebuilt.body);
            }
        }

        result.success = true;
        return result;
    }

    SCSectionBodySetRebuild3d RebuildSectionBodies(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionBodySetRebuild3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionTopology3d topology = BuildSectionTopology(section, eps);
        if (!topology.IsValid())
        {
            result.issue = SCSectionBodyRebuildIssue3d::InvalidSection;
            return result;
        }

        const SCSectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            result.issue = SCSectionBodyRebuildIssue3d::FaceRebuildFailed;
            return result;
        }

        result.bodies.reserve(topology.Roots().size());
        for (std::size_t rootIndex : topology.Roots())
        {
            std::vector<PolyhedronFace3d> faces;
            for (std::size_t faceIndex = 0; faceIndex < rebuiltFaces.faces.size(); ++faceIndex)
            {
                if (rebuiltFaces.mappings[faceIndex].outerPolygonIndex == rootIndex)
                {
                    faces.push_back(rebuiltFaces.faces[faceIndex]);
                }
            }

            if (!faces.empty())
            {
                result.rootPolygonIndices.push_back(rootIndex);
                result.bodies.emplace_back(std::move(faces));
            }
        }

        result.success = true;
        return result;
    }

    SCSectionTopology3d BuildSectionTopology(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionTopology3d result{};
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            return result;
        }

        SCMultiPolygon2d polygons;
        if (!BuildNormalizedSectionPolygons(section, polygons))
        {
            return result;
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

    std::string SCSectionTopology3d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCSectionTopology3d{polygonCount=" << Count() << ", rootCount=" << roots_.size()
               << ", valid=" << (valid_ ? "true" : "false") << "}";
        return stream.str();
    }

    SCSectionMeshConversion3d ConvertSectionToTriangleMesh(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionMeshConversion3d result{};
        const SCSectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            result.issue = MeshConversionIssue3d::InvalidFace;
            return result;
        }

        std::vector<SCPoint3d> vertices;
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
                    triangle[0] + vertexOffset, triangle[1] + vertexOffset, triangle[2] + vertexOffset});
            }
        }

        result.mesh = TriangleMesh(std::move(vertices), std::move(triangles));
        result.success = true;
        return result;
    }

    SCSectionMeshSetConversion3d ConvertSectionToTriangleMeshes(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionMeshSetConversion3d result{};
        const SCSectionBodySetRebuild3d rebuiltBodies = RebuildSectionBodies(section, eps);
        if (!rebuiltBodies.success)
        {
            result.issue = MeshConversionIssue3d::InvalidFace;
            return result;
        }

        result.meshes.reserve(rebuiltBodies.bodies.size());
        result.rootPolygonIndices = rebuiltBodies.rootPolygonIndices;
        for (const PolyhedronBody& body : rebuiltBodies.bodies)
        {
            const PolyhedronMeshConversion3d bodyMesh = ConvertToTriangleMesh(body, eps);
            if (!bodyMesh.success)
            {
                result.issue = bodyMesh.issue;
                return result;
            }
            result.meshes.push_back(bodyMesh.mesh);
        }

        result.success = true;
        return result;
    }

    SCSectionContentKind3d ClassifySectionContent(const SCPolyhedronSection3d& section, double eps)
    {
        if (!section.success || !IsSectionFrameValid(section, eps))
        {
            return SCSectionContentKind3d::Empty;
        }

        const bool hasArea = !section.polygons.empty();
        bool hasCurve = false;
        for (const SCSectionPolyline3d& contour : section.contours)
        {
            if (!contour.closed)
            {
                hasCurve = true;
                break;
            }
        }

        if (hasArea && hasCurve)
        {
            return SCSectionContentKind3d::Mixed;
        }
        if (hasArea)
        {
            return SCSectionContentKind3d::Area;
        }
        if (hasCurve || !section.segments.empty())
        {
            return SCSectionContentKind3d::Curve;
        }
        return SCSectionContentKind3d::Empty;
    }

    SCSectionComponents3d BuildSectionComponents(const SCPolyhedronSection3d& section, double eps)
    {
        SCSectionComponents3d result{};
        const SCSectionTopology3d topology = BuildSectionTopology(section, eps);
        if (!topology.IsValid())
        {
            return result;
        }

        const SCSectionFaceRebuild3d rebuiltFaces = RebuildSectionFaces(section, eps);
        if (!rebuiltFaces.success)
        {
            return result;
        }

        result.valid = true;
        result.components.reserve(topology.Roots().size());
        for (std::size_t rootIndex : topology.Roots())
        {
            SCSectionComponent3d component{};
            component.rootPolygonIndex = rootIndex;

            std::vector<std::size_t> stack{rootIndex};
            while (!stack.empty())
            {
                const std::size_t current = stack.back();
                stack.pop_back();
                component.polygonIndices.push_back(current);
                for (std::size_t child : topology.ChildrenOf(current))
                {
                    stack.push_back(child);
                }
            }

            std::sort(component.polygonIndices.begin(), component.polygonIndices.end());
            for (std::size_t faceIndex = 0; faceIndex < rebuiltFaces.mappings.size(); ++faceIndex)
            {
                if (rebuiltFaces.mappings[faceIndex].outerPolygonIndex == rootIndex)
                {
                    component.faceIndices.push_back(faceIndex);
                }
            }

            result.components.push_back(std::move(component));
        }

        return result;
    }
}  // namespace Geometry



