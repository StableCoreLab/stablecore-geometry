#include "Brep/BodyBoolean.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "Brep/BrepConversion.h"
#include "Core/Measure.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] double ResolveTolerance(const BodyBooleanOptions3d& options)
        {
            return options.tolerance.distanceEpsilon;
        }

        [[nodiscard]] double ResolveAngularTolerance(const BodyBooleanOptions3d& options)
        {
            return options.tolerance.angleEpsilon;
        }

        [[nodiscard]] BodyBooleanResult3d MakeInvalidInputResult()
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::InvalidInput;
            result.message =
                "Body boolean input must satisfy the valid closed single-shell manifold contract, including "
                "consistent edge endpoints and planar face boundaries.";
            return result;
        }

        [[nodiscard]] BodyBooleanResult3d MakeUnsupportedResult()
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::UnsupportedOperation;
            result.message =
                "3D body boolean currently supports only deterministic straight-edge closed-body subsets: "
                "geometrically-equivalent/disjoint, axis-aligned contained, single-box, and face-connected orthogonal-box unions, "
                "plus positive-volume non-coplanar convex polyhedral intersections.";
            return result;
        }

        [[nodiscard]] bool BoundsLexicographicallyLess(const SCBox3d& first, const SCBox3d& second, double epsilon);
        [[nodiscard]] bool AppendLoopVerticesFromBody(const SCBrepBody& body,
                                                      const SCBrepLoop& loop,
                                                      std::vector<SCPoint3d>& vertices,
                                                      double epsilon);

        [[nodiscard]] bool IsClosedManifoldSingleShell(const SCBrepBody& body, const double epsilon)
        {
            const SCGeometryTolerance3d tolerance{epsilon, epsilon, epsilon};
            if (!body.IsValid(tolerance) || body.ShellCount() != 1)
            {
                return false;
            }

            const SCBrepShell shell = body.ShellAt(0);
            if (!shell.IsClosed())
            {
                return false;
            }

            std::vector<std::size_t> uses(body.EdgeCount(), 0U);
            std::vector<int> orientationBalance(body.EdgeCount(), 0);
            const auto validateLoop = [&](const SCBrepLoop& loop) {
                if (loop.CoedgeCount() < 3)
                {
                    return false;
                }

                std::size_t expectedStart = static_cast<std::size_t>(-1);
                std::size_t firstStart = static_cast<std::size_t>(-1);
                for (const SCBrepCoedge& coedge : loop.Coedges())
                {
                    if (coedge.EdgeIndex() >= body.EdgeCount())
                    {
                        return false;
                    }

                    const SCBrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
                    const std::size_t start = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
                    const std::size_t end = coedge.Reversed() ? edge.StartVertexIndex() : edge.EndVertexIndex();
                    if (start >= body.VertexCount() || end >= body.VertexCount() || start == end ||
                        (expectedStart != static_cast<std::size_t>(-1) && expectedStart != start))
                    {
                        return false;
                    }

                    if (firstStart == static_cast<std::size_t>(-1))
                    {
                        firstStart = start;
                    }
                    expectedStart = end;
                    ++uses[coedge.EdgeIndex()];
                    orientationBalance[coedge.EdgeIndex()] += coedge.Reversed() ? -1 : 1;
                }
                return expectedStart == firstStart;
            };
            for (const SCBrepFace& face : shell.Faces())
            {
                if (!validateLoop(face.OuterLoop()))
                {
                    return false;
                }
                for (const SCBrepLoop& hole : face.HoleLoops())
                {
                    if (!validateLoop(hole))
                    {
                        return false;
                    }
                }
            }

            for (std::size_t edgeIndex = 0; edgeIndex < body.EdgeCount(); ++edgeIndex)
            {
                if (uses[edgeIndex] != 2U || orientationBalance[edgeIndex] != 0)
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool HasConsistentEdgeEndpoints(const SCBrepBody& body, const double epsilon)
        {
            for (const SCBrepEdge& edge : body.Edges())
            {
                if (edge.Curve() == nullptr || edge.StartVertexIndex() >= body.VertexCount() ||
                    edge.EndVertexIndex() >= body.VertexCount())
                {
                    return false;
                }

                const SCPoint3d curveStart = edge.Curve()->PointAt(edge.Curve()->StartParameter());
                const SCPoint3d curveEnd = edge.Curve()->PointAt(edge.Curve()->EndParameter());
                if (!curveStart.IsValid() || !curveEnd.IsValid() ||
                    !curveStart.AlmostEquals(body.VertexAt(edge.StartVertexIndex()).Point(), epsilon) ||
                    !curveEnd.AlmostEquals(body.VertexAt(edge.EndVertexIndex()).Point(), epsilon))
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool HasPlanarFaceBoundariesConsistentWithSupport(const SCBrepBody& body, const double epsilon)
        {
            const auto loopLiesOnPlane = [&](const SCBrepLoop& loop, const SCPlane& plane) {
                std::vector<SCPoint3d> vertices;
                if (!AppendLoopVerticesFromBody(body, loop, vertices, epsilon))
                {
                    return false;
                }

                return std::all_of(vertices.begin(), vertices.end(), [&](const SCPoint3d& vertex) {
                    return std::abs(plane.SignedDistanceTo(vertex, epsilon)) <= epsilon;
                });
            };
            for (const SCBrepShell& shell : body.Shells())
            {
                for (const SCBrepFace& face : shell.Faces())
                {
                    const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
                    if (planeSurface == nullptr)
                    {
                        continue;
                    }

                    const SCPlane plane = planeSurface->SupportPlane();
                    if (!loopLiesOnPlane(face.OuterLoop(), plane))
                    {
                        return false;
                    }
                    for (const SCBrepLoop& hole : face.HoleLoops())
                    {
                        if (!loopLiesOnPlane(hole, plane))
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        [[nodiscard]] bool IsStraightBrepEdge(const SCBrepEdge& edge)
        {
            return dynamic_cast<const SCLineCurve3d*>(edge.Curve()) != nullptr;
        }

        [[nodiscard]] bool HasOnlyStraightEdges(const SCBrepBody& body)
        {
            return std::all_of(body.Edges().begin(), body.Edges().end(), [](const SCBrepEdge& edge) {
                return IsStraightBrepEdge(edge);
            });
        }

        [[nodiscard]] BodyBooleanResult3d MakeSingleBodyResult(SCBrepBody body, const char* message)
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::None;
            result.body = std::move(body);
            result.message = message;
            return result;
        }

        struct BooleanSolid3d
        {
            std::vector<SCPlane> planes{};
            std::vector<SCPoint3d> vertices{};
        };

        [[nodiscard]] bool HasPoint(const std::vector<SCPoint3d>& points, const SCPoint3d& point, const double epsilon)
        {
            return std::any_of(points.begin(), points.end(), [&](const SCPoint3d& candidate) {
                return candidate.AlmostEquals(point, epsilon);
            });
        }

        [[nodiscard]] bool TryBuildBooleanSolid(const SCBrepBody& body,
                                                const double epsilon,
                                                BooleanSolid3d& solid)
        {
            if (!IsClosedManifoldSingleShell(body, epsilon))
            {
                return false;
            }

            BooleanSolid3d candidate;
            const SCBrepShell shell = body.ShellAt(0);
            candidate.planes.reserve(shell.FaceCount());
            for (const SCBrepFace& face : shell.Faces())
            {
                const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
                std::vector<SCPoint3d> outerVertices;
                if (planeSurface == nullptr || face.HoleCount() != 0 ||
                    !AppendLoopVerticesFromBody(body, face.OuterLoop(), outerVertices, epsilon))
                {
                    return false;
                }
                for (const SCBrepCoedge& coedge : face.OuterLoop().Coedges())
                {
                    if (coedge.EdgeIndex() >= body.EdgeCount() || !IsStraightBrepEdge(body.EdgeAt(coedge.EdgeIndex())))
                    {
                        return false;
                    }
                }

                const SCVector3d normal = planeSurface->SupportPlane().UnitNormal(epsilon);
                if (!normal.IsValid())
                {
                    return false;
                }
                candidate.planes.push_back(SCPlane::FromPointAndNormal(planeSurface->SupportPlane().origin, normal));
                for (const SCPoint3d& vertex : outerVertices)
                {
                    if (!vertex.IsValid())
                    {
                        return false;
                    }
                    if (!HasPoint(candidate.vertices, vertex, epsilon))
                    {
                        candidate.vertices.push_back(vertex);
                    }
                }
            }

            if (candidate.vertices.size() < 4 || candidate.planes.size() < 4)
            {
                return false;
            }

            SCPoint3d center{};
            for (const SCPoint3d& vertex : candidate.vertices)
            {
                center.x += vertex.x;
                center.y += vertex.y;
                center.z += vertex.z;
            }
            const double inverseCount = 1.0 / static_cast<double>(candidate.vertices.size());
            center.x *= inverseCount;
            center.y *= inverseCount;
            center.z *= inverseCount;

            for (SCPlane& plane : candidate.planes)
            {
                if (plane.SignedDistanceTo(center, epsilon) > epsilon)
                {
                    plane.normal = -plane.normal;
                }
            }

            for (const SCPlane& plane : candidate.planes)
            {
                for (const SCPoint3d& vertex : candidate.vertices)
                {
                    if (plane.SignedDistanceTo(vertex, epsilon) > epsilon)
                    {
                        return false;
                    }
                }
            }

            solid = std::move(candidate);
            return true;
        }

        [[nodiscard]] bool AreCoplanar(const SCPlane& first,
                                       const SCPlane& second,
                                       const double distanceEpsilon,
                                       const double angleEpsilon)
        {
            const SCVector3d firstNormal = first.UnitNormal(distanceEpsilon);
            const SCVector3d secondNormal = second.UnitNormal(distanceEpsilon);
            return firstNormal.IsValid() && secondNormal.IsValid() &&
                   Cross(firstNormal, secondNormal).Length() <= angleEpsilon &&
                   std::abs(first.SignedDistanceTo(second.origin, distanceEpsilon)) <= distanceEpsilon;
        }

        [[nodiscard]] bool TryIntersectThreePlanes(const SCPlane& first,
                                                   const SCPlane& second,
                                                   const SCPlane& third,
                                                   const double distanceEpsilon,
                                                   const double angleEpsilon,
                                                   SCPoint3d& point)
        {
            const SCVector3d firstNormal = first.UnitNormal(distanceEpsilon);
            const SCVector3d secondNormal = second.UnitNormal(distanceEpsilon);
            const SCVector3d thirdNormal = third.UnitNormal(distanceEpsilon);
            const double denominator = Dot(firstNormal, Cross(secondNormal, thirdNormal));
            if (std::abs(denominator) <= angleEpsilon)
            {
                return false;
            }

            const double firstOffset = Dot(firstNormal, first.origin - SCPoint3d{});
            const double secondOffset = Dot(secondNormal, second.origin - SCPoint3d{});
            const double thirdOffset = Dot(thirdNormal, third.origin - SCPoint3d{});
            const SCVector3d numerator = firstOffset * Cross(secondNormal, thirdNormal) +
                                         secondOffset * Cross(thirdNormal, firstNormal) +
                                         thirdOffset * Cross(firstNormal, secondNormal);
            point = SCPoint3d{numerator.x / denominator, numerator.y / denominator, numerator.z / denominator};
            return point.IsValid();
        }

        [[nodiscard]] bool IsInsideAllHalfSpaces(const std::vector<SCPlane>& planes,
                                                 const SCPoint3d& point,
                                                 const double epsilon)
        {
            return std::all_of(planes.begin(), planes.end(), [&](const SCPlane& plane) {
                return plane.SignedDistanceTo(point, epsilon) <= epsilon;
            });
        }

        [[nodiscard]] bool TryBuildConvexIntersection(const SCBrepBody& first,
                                                       const SCBrepBody& second,
                                                       const double epsilon,
                                                       const double angleEpsilon,
                                                       SCBrepBody& result)
        {
            BooleanSolid3d firstSolid;
            BooleanSolid3d secondSolid;
            if (!TryBuildBooleanSolid(first, epsilon, firstSolid) || !TryBuildBooleanSolid(second, epsilon, secondSolid))
            {
                return false;
            }

            for (const SCPlane& firstPlane : firstSolid.planes)
            {
                if (std::any_of(secondSolid.planes.begin(), secondSolid.planes.end(), [&](const SCPlane& secondPlane) {
                        return AreCoplanar(firstPlane, secondPlane, epsilon, angleEpsilon);
                    }))
                {
                    return false;
                }
            }

            std::vector<SCPlane> planes = firstSolid.planes;
            planes.insert(planes.end(), secondSolid.planes.begin(), secondSolid.planes.end());
            std::vector<SCPoint3d> vertices;
            for (std::size_t firstIndex = 0; firstIndex < planes.size(); ++firstIndex)
            {
                for (std::size_t secondIndex = firstIndex + 1; secondIndex < planes.size(); ++secondIndex)
                {
                    for (std::size_t thirdIndex = secondIndex + 1; thirdIndex < planes.size(); ++thirdIndex)
                    {
                        SCPoint3d point;
                        if (TryIntersectThreePlanes(
                                planes[firstIndex], planes[secondIndex], planes[thirdIndex], epsilon, angleEpsilon, point) &&
                            IsInsideAllHalfSpaces(planes, point, epsilon) && !HasPoint(vertices, point, epsilon))
                        {
                            vertices.push_back(point);
                        }
                    }
                }
            }
            if (vertices.size() < 4)
            {
                return false;
            }

            std::vector<PolyhedronFace3d> faces;
            for (const SCPlane& plane : planes)
            {
                std::vector<SCPoint3d> faceVertices;
                for (const SCPoint3d& vertex : vertices)
                {
                    if (std::abs(plane.SignedDistanceTo(vertex, epsilon)) <= epsilon && !HasPoint(faceVertices, vertex, epsilon))
                    {
                        faceVertices.push_back(vertex);
                    }
                }
                if (faceVertices.size() < 3)
                {
                    continue;
                }

                SCPoint3d faceCenter{};
                for (const SCPoint3d& vertex : faceVertices)
                {
                    faceCenter.x += vertex.x;
                    faceCenter.y += vertex.y;
                    faceCenter.z += vertex.z;
                }
                const double inverseCount = 1.0 / static_cast<double>(faceVertices.size());
                faceCenter.x *= inverseCount;
                faceCenter.y *= inverseCount;
                faceCenter.z *= inverseCount;

                const SCVector3d normal = plane.UnitNormal(epsilon);
                const SCVector3d reference = std::abs(normal.x) < 0.8 ? SCVector3d{1.0, 0.0, 0.0} : SCVector3d{0.0, 1.0, 0.0};
                const SCVector3d uAxis = Cross(reference, normal).Normalized(epsilon);
                const SCVector3d vAxis = Cross(normal, uAxis).Normalized(epsilon);
                if (!uAxis.IsValid() || !vAxis.IsValid())
                {
                    return false;
                }
                std::sort(faceVertices.begin(), faceVertices.end(), [&](const SCPoint3d& left, const SCPoint3d& right) {
                    const SCVector3d leftOffset = left - faceCenter;
                    const SCVector3d rightOffset = right - faceCenter;
                    return std::atan2(Dot(leftOffset, vAxis), Dot(leftOffset, uAxis)) <
                           std::atan2(Dot(rightOffset, vAxis), Dot(rightOffset, uAxis));
                });
                faces.emplace_back(plane, PolyhedronLoop3d(std::move(faceVertices)));
            }

            const PolyhedronBrepBodyConversion3d converted = ConvertToBrepBody(PolyhedronBody(std::move(faces)), epsilon);
            if (!converted.success || !IsClosedManifoldSingleShell(converted.body, epsilon) ||
                Geometry::Volume(converted.body, epsilon) <= epsilon * epsilon * epsilon)
            {
                return false;
            }

            result = converted.body;
            return true;
        }

        [[nodiscard]] BodyBooleanResult3d MakeMultiBodyResult(std::vector<SCBrepBody> bodies, const char* message)
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::None;
            std::sort(bodies.begin(), bodies.end(), [](const SCBrepBody& first, const SCBrepBody& second) {
                return BoundsLexicographicallyLess(
                    first.Bounds(), second.Bounds(), Geometry::kBodyBooleanDefaultEpsilon);
            });
            result.bodies = std::move(bodies);
            result.message = message;
            return result;
        }

        [[nodiscard]] BodyBooleanResult3d MakeEmptyResult(const char* message)
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::None;
            result.producedEmptyResult = true;
            result.message = message;
            return result;
        }

        [[nodiscard]] bool HasFaces(const SCBrepBody& body)
        {
            return body.FaceCount() > 0;
        }

        [[nodiscard]] bool HasFaces(const PolyhedronBody& body)
        {
            return body.FaceCount() > 0;
        }

        [[nodiscard]] bool NearlyEqual(double left, double right, double epsilon)
        {
            return std::abs(left - right) <= epsilon;
        }

        [[nodiscard]] bool NearlyEqualScaled(double left, double right, double epsilon)
        {
            const double scale = std::max(std::abs(left), std::abs(right));
            const double absoluteTolerance = epsilon * epsilon * epsilon;
            const double relativeTolerance = epsilon * scale;
            return std::abs(left - right) <= std::max(absoluteTolerance, relativeTolerance);
        }

        [[nodiscard]] bool BoundsEqual(const SCBox3d& left, const SCBox3d& right, double epsilon)
        {
            return left.IsValid() && right.IsValid() && NearlyEqual(left.MinPoint().x, right.MinPoint().x, epsilon) &&
                   NearlyEqual(left.MinPoint().y, right.MinPoint().y, epsilon) &&
                   NearlyEqual(left.MinPoint().z, right.MinPoint().z, epsilon) &&
                   NearlyEqual(left.MaxPoint().x, right.MaxPoint().x, epsilon) &&
                   NearlyEqual(left.MaxPoint().y, right.MaxPoint().y, epsilon) &&
                   NearlyEqual(left.MaxPoint().z, right.MaxPoint().z, epsilon);
        }

        [[nodiscard]] bool BoundsLexicographicallyLess(const SCBox3d& first, const SCBox3d& second, double epsilon)
        {
            if (!NearlyEqual(first.MinPoint().x, second.MinPoint().x, epsilon))
            {
                return first.MinPoint().x < second.MinPoint().x;
            }
            if (!NearlyEqual(first.MinPoint().y, second.MinPoint().y, epsilon))
            {
                return first.MinPoint().y < second.MinPoint().y;
            }
            if (!NearlyEqual(first.MinPoint().z, second.MinPoint().z, epsilon))
            {
                return first.MinPoint().z < second.MinPoint().z;
            }
            if (!NearlyEqual(first.MaxPoint().x, second.MaxPoint().x, epsilon))
            {
                return first.MaxPoint().x < second.MaxPoint().x;
            }
            if (!NearlyEqual(first.MaxPoint().y, second.MaxPoint().y, epsilon))
            {
                return first.MaxPoint().y < second.MaxPoint().y;
            }
            if (!NearlyEqual(first.MaxPoint().z, second.MaxPoint().z, epsilon))
            {
                return first.MaxPoint().z < second.MaxPoint().z;
            }
            return false;
        }

        [[nodiscard]] bool BoundsDisjoint(const SCBox3d& left, const SCBox3d& right, double epsilon)
        {
            return left.IsValid() && right.IsValid() &&
                   (left.MaxPoint().x < right.MinPoint().x - epsilon ||
                    right.MaxPoint().x < left.MinPoint().x - epsilon ||
                    left.MaxPoint().y < right.MinPoint().y - epsilon ||
                    right.MaxPoint().y < left.MinPoint().y - epsilon ||
                    left.MaxPoint().z < right.MinPoint().z - epsilon ||
                    right.MaxPoint().z < left.MinPoint().z - epsilon);
        }

        [[nodiscard]] double BoxVolume(const SCBox3d& box)
        {
            if (!box.IsValid())
            {
                return 0.0;
            }

            return box.Width() * box.Height() * box.Depth();
        }

        [[nodiscard]] bool HasPositiveBoxVolume(const SCBox3d& box, double epsilon)
        {
            return box.IsValid() && box.Width() > epsilon && box.Height() > epsilon && box.Depth() > epsilon;
        }

        [[nodiscard]] double CoordinateAt(const SCPoint3d& point, int axis)
        {
            switch (axis)
            {
                case 0:
                    return point.x;
                case 1:
                    return point.y;
                default:
                    return point.z;
            }
        }

        [[nodiscard]] double BoxMinAt(const SCBox3d& box, int axis)
        {
            return CoordinateAt(box.MinPoint(), axis);
        }

        [[nodiscard]] double BoxMaxAt(const SCBox3d& box, int axis)
        {
            return CoordinateAt(box.MaxPoint(), axis);
        }

        [[nodiscard]] double NormalAt(const SCVector3d& vector, int axis)
        {
            switch (axis)
            {
                case 0:
                    return vector.x;
                case 1:
                    return vector.y;
                default:
                    return vector.z;
            }
        }

        [[nodiscard]] bool CoordinateMatchesEitherBoundary(double value,
                                                           double minValue,
                                                           double maxValue,
                                                           double epsilon)
        {
            return NearlyEqual(value, minValue, epsilon) || NearlyEqual(value, maxValue, epsilon);
        }

        [[nodiscard]] bool AppendLoopVerticesFromBody(const SCBrepBody& body,
                                                      const SCBrepLoop& loop,
                                                      std::vector<SCPoint3d>& vertices,
                                                      double epsilon)
        {
            vertices.clear();
            if (!loop.IsValid())
            {
                return false;
            }

            vertices.reserve(loop.CoedgeCount());
            for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
            {
                const SCBrepCoedge coedge = loop.CoedgeAt(i);
                if (coedge.EdgeIndex() >= body.EdgeCount())
                {
                    return false;
                }

                const SCBrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
                const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
                if (vertexIndex >= body.VertexCount())
                {
                    return false;
                }

                const SCPoint3d point = body.VertexAt(vertexIndex).Point();
                if (vertices.empty() || !vertices.back().AlmostEquals(point, epsilon))
                {
                    vertices.push_back(point);
                }
            }

            while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), epsilon))
            {
                vertices.pop_back();
            }

            return vertices.size() >= 3;
        }

        [[nodiscard]] bool PointCyclesAreEquivalent(const std::vector<SCPoint3d>& first,
                                                     const std::vector<SCPoint3d>& second,
                                                     const double epsilon)
        {
            if (first.size() != second.size() || first.empty())
            {
                return false;
            }

            for (std::size_t offset = 0; offset < second.size(); ++offset)
            {
                bool sameDirection = true;
                bool oppositeDirection = true;
                for (std::size_t index = 0; index < first.size() && (sameDirection || oppositeDirection); ++index)
                {
                    sameDirection = sameDirection &&
                                    first[index].AlmostEquals(second[(offset + index) % second.size()], epsilon);
                    oppositeDirection = oppositeDirection &&
                                        first[index].AlmostEquals(
                                            second[(offset + second.size() - index) % second.size()], epsilon);
                }
                if (sameDirection || oppositeDirection)
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool LoopsAreGeometricallyEquivalent(const SCBrepBody& first,
                                                            const SCBrepLoop& firstLoop,
                                                            const SCBrepBody& second,
                                                            const SCBrepLoop& secondLoop,
                                                            const double epsilon)
        {
            std::vector<SCPoint3d> firstVertices;
            std::vector<SCPoint3d> secondVertices;
            return AppendLoopVerticesFromBody(first, firstLoop, firstVertices, epsilon) &&
                   AppendLoopVerticesFromBody(second, secondLoop, secondVertices, epsilon) &&
                   PointCyclesAreEquivalent(firstVertices, secondVertices, epsilon);
        }

        [[nodiscard]] bool FaceSupportsAreGeometricallyEquivalent(const SCBrepFace& first,
                                                                   const SCBrepFace& second,
                                                                   const double distanceEpsilon,
                                                                   const double angleEpsilon)
        {
            if (first.SupportSurface() == second.SupportSurface())
            {
                return true;
            }

            const auto* firstPlaneSurface = dynamic_cast<const SCPlaneSurface*>(first.SupportSurface());
            const auto* secondPlaneSurface = dynamic_cast<const SCPlaneSurface*>(second.SupportSurface());
            if (firstPlaneSurface == nullptr || secondPlaneSurface == nullptr)
            {
                return false;
            }

            const SCPlane firstPlane = firstPlaneSurface->SupportPlane();
            const SCPlane secondPlane = secondPlaneSurface->SupportPlane();
            const SCVector3d firstNormal = firstPlane.UnitNormal(distanceEpsilon);
            const SCVector3d secondNormal = secondPlane.UnitNormal(distanceEpsilon);
            return firstNormal.IsValid() && secondNormal.IsValid() &&
                   Cross(firstNormal, secondNormal).Length() <= angleEpsilon &&
                   std::abs(firstPlane.SignedDistanceTo(secondPlane.origin, distanceEpsilon)) <= distanceEpsilon;
        }

        [[nodiscard]] bool FacesAreGeometricallyEquivalent(const SCBrepBody& first,
                                                            const SCBrepFace& firstFace,
                                                            const SCBrepBody& second,
                                                            const SCBrepFace& secondFace,
                                                            const double distanceEpsilon,
                                                            const double angleEpsilon)
        {
            if (!FaceSupportsAreGeometricallyEquivalent(firstFace, secondFace, distanceEpsilon, angleEpsilon) ||
                !LoopsAreGeometricallyEquivalent(
                    first, firstFace.OuterLoop(), second, secondFace.OuterLoop(), distanceEpsilon) ||
                firstFace.HoleCount() != secondFace.HoleCount())
            {
                return false;
            }

            std::vector<bool> matched(secondFace.HoleCount(), false);
            for (const SCBrepLoop& firstHole : firstFace.HoleLoops())
            {
                bool found = false;
                for (std::size_t index = 0; index < secondFace.HoleCount(); ++index)
                {
                    if (!matched[index] && LoopsAreGeometricallyEquivalent(
                                               first, firstHole, second, secondFace.HoleAt(index), distanceEpsilon))
                    {
                        matched[index] = true;
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool BodiesAreGeometricallyEquivalent(const SCBrepBody& first,
                                                             const SCBrepBody& second,
                                                             const double distanceEpsilon,
                                                             const double angleEpsilon)
        {
            if (&first == &second)
            {
                return true;
            }
            if (first.FaceCount() != second.FaceCount() || first.EdgeCount() != second.EdgeCount() ||
                first.VertexCount() != second.VertexCount() || first.ShellCount() != second.ShellCount())
            {
                return false;
            }

            std::vector<SCBrepFace> secondFaces;
            for (const SCBrepShell& shell : second.Shells())
            {
                secondFaces.insert(secondFaces.end(), shell.Faces().begin(), shell.Faces().end());
            }
            std::vector<bool> matched(secondFaces.size(), false);
            for (const SCBrepShell& shell : first.Shells())
            {
                for (const SCBrepFace& firstFace : shell.Faces())
                {
                    bool found = false;
                    for (std::size_t index = 0; index < secondFaces.size(); ++index)
                    {
                        if (!matched[index] &&
                            FacesAreGeometricallyEquivalent(
                                first, firstFace, second, secondFaces[index], distanceEpsilon, angleEpsilon))
                        {
                            matched[index] = true;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] bool FaceMatchesAxisAlignedBox(
            const SCBrepBody& body, const SCBrepFace& face, const SCBox3d& box, double epsilon, int& axis, bool& onMaxSide)
        {
            if (face.HoleCount() != 0 || face.OuterLoop().CoedgeCount() != 4)
            {
                return false;
            }

            const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
            if (planeSurface == nullptr)
            {
                return false;
            }

            std::vector<SCPoint3d> outerVertices;
            if (!AppendLoopVerticesFromBody(body, face.OuterLoop(), outerVertices, epsilon) ||
                outerVertices.size() != 4)
            {
                return false;
            }

            const SCVector3d unitNormal = planeSurface->SupportPlane().UnitNormal(epsilon);
            if (!unitNormal.IsValid())
            {
                return false;
            }

            axis = -1;
            if (std::abs(unitNormal.x) > epsilon && std::abs(unitNormal.y) <= epsilon &&
                std::abs(unitNormal.z) <= epsilon)
            {
                axis = 0;
            } else if (std::abs(unitNormal.y) > epsilon && std::abs(unitNormal.x) <= epsilon &&
                       std::abs(unitNormal.z) <= epsilon)
            {
                axis = 1;
            } else if (std::abs(unitNormal.z) > epsilon && std::abs(unitNormal.x) <= epsilon &&
                       std::abs(unitNormal.y) <= epsilon)
            {
                axis = 2;
            }

            if (axis < 0)
            {
                return false;
            }

            const double faceCoordinate = CoordinateAt(outerVertices[0], axis);
            const bool onMinSide = NearlyEqual(faceCoordinate, BoxMinAt(box, axis), epsilon);
            onMaxSide = NearlyEqual(faceCoordinate, BoxMaxAt(box, axis), epsilon);
            if (onMinSide == onMaxSide)
            {
                return false;
            }

            const double normalComponent = NormalAt(unitNormal, axis);
            if ((onMaxSide && normalComponent <= 0.0) || (onMinSide && normalComponent >= 0.0))
            {
                return false;
            }

            const int otherAxis0 = (axis + 1) % 3;
            const int otherAxis1 = (axis + 2) % 3;
            std::array<bool, 4> seenCorners{};
            for (const SCPoint3d& vertex : outerVertices)
            {
                if (!NearlyEqual(CoordinateAt(vertex, axis), faceCoordinate, epsilon))
                {
                    return false;
                }

                const double coordinate0 = CoordinateAt(vertex, otherAxis0);
                const double coordinate1 = CoordinateAt(vertex, otherAxis1);
                if (!CoordinateMatchesEitherBoundary(
                        coordinate0, BoxMinAt(box, otherAxis0), BoxMaxAt(box, otherAxis0), epsilon) ||
                    !CoordinateMatchesEitherBoundary(
                        coordinate1, BoxMinAt(box, otherAxis1), BoxMaxAt(box, otherAxis1), epsilon))
                {
                    return false;
                }

                const bool bit0 = NearlyEqual(coordinate0, BoxMaxAt(box, otherAxis0), epsilon);
                const bool bit1 = NearlyEqual(coordinate1, BoxMaxAt(box, otherAxis1), epsilon);
                const std::size_t cornerIndex =
                    static_cast<std::size_t>(bit0 ? 1 : 0) + static_cast<std::size_t>(bit1 ? 2 : 0);
                seenCorners[cornerIndex] = true;
            }

            return seenCorners[0] && seenCorners[1] && seenCorners[2] && seenCorners[3];
        }

        [[nodiscard]] bool TryExtractAxisAlignedBoxFromBrep(const SCBrepBody& body, double epsilon, SCBox3d& box)
        {
            if (body.ShellCount() != 1)
            {
                return false;
            }

            box = body.Bounds();
            if (!HasPositiveBoxVolume(box, epsilon))
            {
                return false;
            }

            std::array<bool, 6> seenFaces{};
            const SCBrepShell shell = body.ShellAt(0);
            for (std::size_t faceIndex = 0; faceIndex < shell.FaceCount(); ++faceIndex)
            {
                int axis = -1;
                bool onMaxSide = false;
                if (!FaceMatchesAxisAlignedBox(body, shell.FaceAt(faceIndex), box, epsilon, axis, onMaxSide))
                {
                    return false;
                }

                const std::size_t slot = static_cast<std::size_t>(axis * 2 + (onMaxSide ? 1 : 0));
                if (seenFaces[slot])
                {
                    return false;
                }
                seenFaces[slot] = true;
            }

            return seenFaces[0] && seenFaces[1] && seenFaces[2] && seenFaces[3] && seenFaces[4] && seenFaces[5];
        }

        [[nodiscard]] bool TryComputePositiveIntersectionBox(const SCBox3d& first,
                                                             const SCBox3d& second,
                                                             double epsilon,
                                                             SCBox3d& overlap)
        {
            if (!first.IsValid() || !second.IsValid())
            {
                return false;
            }

            overlap = SCBox3d::FromMinMax(SCPoint3d{std::max(first.MinPoint().x, second.MinPoint().x),
                                                std::max(first.MinPoint().y, second.MinPoint().y),
                                                std::max(first.MinPoint().z, second.MinPoint().z)},
                                        SCPoint3d{std::min(first.MaxPoint().x, second.MaxPoint().x),
                                                std::min(first.MaxPoint().y, second.MaxPoint().y),
                                                std::min(first.MaxPoint().z, second.MaxPoint().z)});
            return HasPositiveBoxVolume(overlap, epsilon);
        }

        [[nodiscard]] bool TryComputeIntersectionBox(const SCBox3d& first,
                                                     const SCBox3d& second,
                                                     double epsilon,
                                                     SCBox3d& overlap)
        {
            if (!first.IsValid() || !second.IsValid())
            {
                return false;
            }

            overlap = SCBox3d::FromMinMax(SCPoint3d{std::max(first.MinPoint().x, second.MinPoint().x),
                                                std::max(first.MinPoint().y, second.MinPoint().y),
                                                std::max(first.MinPoint().z, second.MinPoint().z)},
                                        SCPoint3d{std::min(first.MaxPoint().x, second.MaxPoint().x),
                                                std::min(first.MaxPoint().y, second.MaxPoint().y),
                                                std::min(first.MaxPoint().z, second.MaxPoint().z)});
            (void)epsilon;
            return overlap.IsValid();
        }

        [[nodiscard]] PolyhedronBody BuildAxisAlignedBoxPolyhedronBody(const SCBox3d& box)
        {
            const SCPoint3d minPoint = box.MinPoint();
            const SCPoint3d maxPoint = box.MaxPoint();

            return PolyhedronBody({
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{minPoint.x, minPoint.y, minPoint.z}, SCVector3d{0.0, 0.0, -1.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{minPoint.x, minPoint.y, minPoint.z},
                        SCPoint3d{minPoint.x, maxPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, minPoint.y, minPoint.z},
                    })),
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{minPoint.x, minPoint.y, maxPoint.z}, SCVector3d{0.0, 0.0, 1.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{minPoint.x, minPoint.y, maxPoint.z},
                        SCPoint3d{maxPoint.x, minPoint.y, maxPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, maxPoint.z},
                        SCPoint3d{minPoint.x, maxPoint.y, maxPoint.z},
                    })),
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{minPoint.x, minPoint.y, minPoint.z}, SCVector3d{0.0, -1.0, 0.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{minPoint.x, minPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, minPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, minPoint.y, maxPoint.z},
                        SCPoint3d{minPoint.x, minPoint.y, maxPoint.z},
                    })),
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{maxPoint.x, minPoint.y, minPoint.z}, SCVector3d{1.0, 0.0, 0.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{maxPoint.x, minPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, minPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, maxPoint.z},
                        SCPoint3d{maxPoint.x, minPoint.y, maxPoint.z},
                    })),
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{minPoint.x, maxPoint.y, minPoint.z}, SCVector3d{0.0, 1.0, 0.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{minPoint.x, maxPoint.y, minPoint.z},
                        SCPoint3d{minPoint.x, maxPoint.y, maxPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, maxPoint.z},
                        SCPoint3d{maxPoint.x, maxPoint.y, minPoint.z},
                    })),
                PolyhedronFace3d(
                    SCPlane::FromPointAndNormal(SCPoint3d{minPoint.x, minPoint.y, minPoint.z}, SCVector3d{-1.0, 0.0, 0.0}),
                    PolyhedronLoop3d({
                        SCPoint3d{minPoint.x, minPoint.y, minPoint.z},
                        SCPoint3d{minPoint.x, minPoint.y, maxPoint.z},
                        SCPoint3d{minPoint.x, maxPoint.y, maxPoint.z},
                        SCPoint3d{minPoint.x, maxPoint.y, minPoint.z},
                    })),
            });
        }

        [[nodiscard]] BodyBooleanResult3d MakeAxisAlignedBoxResult(const SCBox3d& box,
                                                                   double epsilon,
                                                                   const char* message)
        {
            const PolyhedronBrepBodyConversion3d converted =
                ConvertToBrepBody(BuildAxisAlignedBoxPolyhedronBody(box), epsilon);
            if (!converted.success)
            {
                return MakeUnsupportedResult();
            }

            return MakeSingleBodyResult(converted.body, message);
        }

        [[nodiscard]] bool FaceMatchesAxisAlignedBox(
            const PolyhedronFace3d& face, const SCBox3d& box, double epsilon, int& axis, bool& onMaxSide)
        {
            if (face.HoleCount() != 0 || face.OuterLoop().VertexCount() != 4)
            {
                return false;
            }

            const SCVector3d unitNormal = face.SupportPlane().UnitNormal(epsilon);
            if (!unitNormal.IsValid())
            {
                return false;
            }

            axis = -1;
            if (std::abs(unitNormal.x) > epsilon && std::abs(unitNormal.y) <= epsilon &&
                std::abs(unitNormal.z) <= epsilon)
            {
                axis = 0;
            } else if (std::abs(unitNormal.y) > epsilon && std::abs(unitNormal.x) <= epsilon &&
                       std::abs(unitNormal.z) <= epsilon)
            {
                axis = 1;
            } else if (std::abs(unitNormal.z) > epsilon && std::abs(unitNormal.x) <= epsilon &&
                       std::abs(unitNormal.y) <= epsilon)
            {
                axis = 2;
            }

            if (axis < 0)
            {
                return false;
            }

            const double faceCoordinate = CoordinateAt(face.OuterLoop().VertexAt(0), axis);
            const bool onMinSide = NearlyEqual(faceCoordinate, BoxMinAt(box, axis), epsilon);
            onMaxSide = NearlyEqual(faceCoordinate, BoxMaxAt(box, axis), epsilon);
            if (onMinSide == onMaxSide)
            {
                return false;
            }

            const double normalComponent = NormalAt(unitNormal, axis);
            if ((onMaxSide && normalComponent <= 0.0) || (onMinSide && normalComponent >= 0.0))
            {
                return false;
            }

            const int otherAxis0 = (axis + 1) % 3;
            const int otherAxis1 = (axis + 2) % 3;
            std::array<bool, 4> seenCorners{};
            for (const SCPoint3d& vertex : face.OuterLoop().Vertices())
            {
                if (!NearlyEqual(CoordinateAt(vertex, axis), faceCoordinate, epsilon))
                {
                    return false;
                }

                const double coordinate0 = CoordinateAt(vertex, otherAxis0);
                const double coordinate1 = CoordinateAt(vertex, otherAxis1);
                if (!CoordinateMatchesEitherBoundary(
                        coordinate0, BoxMinAt(box, otherAxis0), BoxMaxAt(box, otherAxis0), epsilon) ||
                    !CoordinateMatchesEitherBoundary(
                        coordinate1, BoxMinAt(box, otherAxis1), BoxMaxAt(box, otherAxis1), epsilon))
                {
                    return false;
                }

                const bool high0 = NearlyEqual(coordinate0, BoxMaxAt(box, otherAxis0), epsilon);
                const bool high1 = NearlyEqual(coordinate1, BoxMaxAt(box, otherAxis1), epsilon);
                const std::size_t cornerIndex = (high0 ? 2U : 0U) + (high1 ? 1U : 0U);
                if (seenCorners[cornerIndex])
                {
                    return false;
                }
                seenCorners[cornerIndex] = true;
            }

            return seenCorners[0] && seenCorners[1] && seenCorners[2] && seenCorners[3];
        }

        [[nodiscard]] bool TryExtractAxisAlignedBox(const PolyhedronBody& body, double epsilon, SCBox3d& box)
        {
            if (!body.IsValid(epsilon) || body.FaceCount() != 6)
            {
                return false;
            }

            box = body.Bounds();
            if (!HasPositiveBoxVolume(box, epsilon))
            {
                return false;
            }

            std::array<bool, 6> seenFaces{};
            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                int axis = -1;
                bool onMaxSide = false;
                if (!FaceMatchesAxisAlignedBox(body.FaceAt(faceIndex), box, epsilon, axis, onMaxSide))
                {
                    return false;
                }

                const std::size_t slot = static_cast<std::size_t>(axis * 2 + (onMaxSide ? 1 : 0));
                if (seenFaces[slot])
                {
                    return false;
                }
                seenFaces[slot] = true;
            }

            return seenFaces[0] && seenFaces[1] && seenFaces[2] && seenFaces[3] && seenFaces[4] && seenFaces[5];
        }

        [[nodiscard]] bool TryExtractAxisAlignedBox(const SCBrepBody& body, double epsilon, SCBox3d& box)
        {
            if (TryExtractAxisAlignedBoxFromBrep(body, epsilon, box))
            {
                return true;
            }

            const BrepBodyConversion3d converted = ConvertToPolyhedronBody(body, epsilon);
            if (!converted.success)
            {
                return false;
            }

            return TryExtractAxisAlignedBox(converted.body, epsilon, box);
        }

        [[nodiscard]] bool TryComputeSingleBoxUnion(
            const SCBox3d& first, const SCBox3d& second, const SCBox3d& overlap, double epsilon, SCBox3d& united)
        {
            united = SCBox3d::FromMinMax(SCPoint3d{std::min(first.MinPoint().x, second.MinPoint().x),
                                               std::min(first.MinPoint().y, second.MinPoint().y),
                                               std::min(first.MinPoint().z, second.MinPoint().z)},
                                       SCPoint3d{std::max(first.MaxPoint().x, second.MaxPoint().x),
                                               std::max(first.MaxPoint().y, second.MaxPoint().y),
                                               std::max(first.MaxPoint().z, second.MaxPoint().z)});
            if (!HasPositiveBoxVolume(united, epsilon))
            {
                return false;
            }

            const double unionVolume = BoxVolume(first) + BoxVolume(second) - BoxVolume(overlap);
            return NearlyEqualScaled(BoxVolume(united), unionVolume, epsilon);
        }

        [[nodiscard]] std::vector<double> CollectBoxCoordinates(const SCBox3d& first,
                                                                const SCBox3d& second,
                                                                const int axis,
                                                                const double epsilon)
        {
            std::vector<double> coordinates{
                BoxMinAt(first, axis), BoxMaxAt(first, axis), BoxMinAt(second, axis), BoxMaxAt(second, axis)};
            std::sort(coordinates.begin(), coordinates.end());

            std::vector<double> unique;
            unique.reserve(coordinates.size());
            for (const double coordinate : coordinates)
            {
                if (unique.empty() || !NearlyEqual(unique.back(), coordinate, epsilon))
                {
                    unique.push_back(coordinate);
                }
            }
            return unique;
        }

        [[nodiscard]] bool ContainsPoint(const SCBox3d& box, const SCPoint3d& point, const double epsilon)
        {
            return point.x >= box.MinPoint().x - epsilon && point.x <= box.MaxPoint().x + epsilon &&
                   point.y >= box.MinPoint().y - epsilon && point.y <= box.MaxPoint().y + epsilon &&
                   point.z >= box.MinPoint().z - epsilon && point.z <= box.MaxPoint().z + epsilon;
        }

        [[nodiscard]] PolyhedronFace3d MakeAxisAlignedCellFace(const SCPoint3d& minPoint,
                                                               const SCPoint3d& maxPoint,
                                                               const int axis,
                                                               const bool onMaxSide)
        {
            if (axis == 0)
            {
                const double x = onMaxSide ? maxPoint.x : minPoint.x;
                const SCVector3d normal{onMaxSide ? 1.0 : -1.0, 0.0, 0.0};
                const std::vector<SCPoint3d> vertices = onMaxSide ? std::vector<SCPoint3d>{{x, minPoint.y, minPoint.z},
                                                                                           {x, maxPoint.y, minPoint.z},
                                                                                           {x, maxPoint.y, maxPoint.z},
                                                                                           {x, minPoint.y, maxPoint.z}}
                                                                  : std::vector<SCPoint3d>{{x, minPoint.y, minPoint.z},
                                                                                           {x, minPoint.y, maxPoint.z},
                                                                                           {x, maxPoint.y, maxPoint.z},
                                                                                           {x, maxPoint.y, minPoint.z}};
                return PolyhedronFace3d(SCPlane::FromPointAndNormal(vertices.front(), normal),
                                        PolyhedronLoop3d(vertices));
            }

            if (axis == 1)
            {
                const double y = onMaxSide ? maxPoint.y : minPoint.y;
                const SCVector3d normal{0.0, onMaxSide ? 1.0 : -1.0, 0.0};
                const std::vector<SCPoint3d> vertices = onMaxSide ? std::vector<SCPoint3d>{{minPoint.x, y, minPoint.z},
                                                                                           {minPoint.x, y, maxPoint.z},
                                                                                           {maxPoint.x, y, maxPoint.z},
                                                                                           {maxPoint.x, y, minPoint.z}}
                                                                  : std::vector<SCPoint3d>{{minPoint.x, y, minPoint.z},
                                                                                           {maxPoint.x, y, minPoint.z},
                                                                                           {maxPoint.x, y, maxPoint.z},
                                                                                           {minPoint.x, y, maxPoint.z}};
                return PolyhedronFace3d(SCPlane::FromPointAndNormal(vertices.front(), normal),
                                        PolyhedronLoop3d(vertices));
            }

            const double z = onMaxSide ? maxPoint.z : minPoint.z;
            const SCVector3d normal{0.0, 0.0, onMaxSide ? 1.0 : -1.0};
            const std::vector<SCPoint3d> vertices = onMaxSide ? std::vector<SCPoint3d>{{minPoint.x, minPoint.y, z},
                                                                                       {maxPoint.x, minPoint.y, z},
                                                                                       {maxPoint.x, maxPoint.y, z},
                                                                                       {minPoint.x, maxPoint.y, z}}
                                                              : std::vector<SCPoint3d>{{minPoint.x, minPoint.y, z},
                                                                                       {minPoint.x, maxPoint.y, z},
                                                                                       {maxPoint.x, maxPoint.y, z},
                                                                                       {maxPoint.x, minPoint.y, z}};
            return PolyhedronFace3d(SCPlane::FromPointAndNormal(vertices.front(), normal), PolyhedronLoop3d(vertices));
        }

        [[nodiscard]] bool TryBuildFaceConnectedBoxUnion(const SCBox3d& first,
                                                         const SCBox3d& second,
                                                         const double epsilon,
                                                         SCBrepBody& result)
        {
            const std::vector<double> x = CollectBoxCoordinates(first, second, 0, epsilon);
            const std::vector<double> y = CollectBoxCoordinates(first, second, 1, epsilon);
            const std::vector<double> z = CollectBoxCoordinates(first, second, 2, epsilon);
            if (x.size() < 2 || y.size() < 2 || z.size() < 2)
            {
                return false;
            }

            const std::size_t xCount = x.size() - 1;
            const std::size_t yCount = y.size() - 1;
            const std::size_t zCount = z.size() - 1;
            const auto cellIndex = [yCount, zCount](const std::size_t ix, const std::size_t iy, const std::size_t iz) {
                return (ix * yCount + iy) * zCount + iz;
            };
            std::vector<bool> occupied(xCount * yCount * zCount, false);
            std::size_t occupiedCount = 0;
            for (std::size_t ix = 0; ix < xCount; ++ix)
            {
                for (std::size_t iy = 0; iy < yCount; ++iy)
                {
                    for (std::size_t iz = 0; iz < zCount; ++iz)
                    {
                        const SCPoint3d center{
                            0.5 * (x[ix] + x[ix + 1]), 0.5 * (y[iy] + y[iy + 1]), 0.5 * (z[iz] + z[iz + 1])};
                        const std::size_t index = cellIndex(ix, iy, iz);
                        occupied[index] =
                            ContainsPoint(first, center, epsilon) || ContainsPoint(second, center, epsilon);
                        occupiedCount += occupied[index] ? 1U : 0U;
                    }
                }
            }
            if (occupiedCount == 0)
            {
                return false;
            }

            std::vector<bool> visited(occupied.size(), false);
            std::vector<std::size_t> pending;
            const std::size_t seed =
                static_cast<std::size_t>(std::find(occupied.begin(), occupied.end(), true) - occupied.begin());
            pending.push_back(seed);
            visited[seed] = true;
            std::size_t connectedCount = 0;
            while (!pending.empty())
            {
                const std::size_t current = pending.back();
                pending.pop_back();
                ++connectedCount;
                const std::size_t iz = current % zCount;
                const std::size_t flattened = current / zCount;
                const std::size_t iy = flattened % yCount;
                const std::size_t ix = flattened / yCount;
                const std::array<std::array<std::size_t, 3>, 6> neighbors{{
                    {{ix > 0 ? ix - 1 : xCount, iy, iz}},
                    {{ix + 1 < xCount ? ix + 1 : xCount, iy, iz}},
                    {{ix, iy > 0 ? iy - 1 : yCount, iz}},
                    {{ix, iy + 1 < yCount ? iy + 1 : yCount, iz}},
                    {{ix, iy, iz > 0 ? iz - 1 : zCount}},
                    {{ix, iy, iz + 1 < zCount ? iz + 1 : zCount}},
                }};
                for (const auto& neighbor : neighbors)
                {
                    if (neighbor[0] >= xCount || neighbor[1] >= yCount || neighbor[2] >= zCount)
                    {
                        continue;
                    }
                    const std::size_t neighborIndex = cellIndex(neighbor[0], neighbor[1], neighbor[2]);
                    if (occupied[neighborIndex] && !visited[neighborIndex])
                    {
                        visited[neighborIndex] = true;
                        pending.push_back(neighborIndex);
                    }
                }
            }
            if (connectedCount != occupiedCount)
            {
                return false;
            }

            std::vector<PolyhedronFace3d> faces;
            for (std::size_t ix = 0; ix < xCount; ++ix)
            {
                for (std::size_t iy = 0; iy < yCount; ++iy)
                {
                    for (std::size_t iz = 0; iz < zCount; ++iz)
                    {
                        if (!occupied[cellIndex(ix, iy, iz)])
                        {
                            continue;
                        }

                        const SCPoint3d minPoint{x[ix], y[iy], z[iz]};
                        const SCPoint3d maxPoint{x[ix + 1], y[iy + 1], z[iz + 1]};
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            for (const bool onMaxSide : {false, true})
                            {
                                std::size_t neighborX = ix;
                                std::size_t neighborY = iy;
                                std::size_t neighborZ = iz;
                                bool hasNeighbor = false;
                                if (axis == 0)
                                {
                                    if (onMaxSide && ix + 1 < xCount)
                                    {
                                        neighborX = ix + 1;
                                        hasNeighbor = true;
                                    } else if (!onMaxSide && ix > 0)
                                    {
                                        neighborX = ix - 1;
                                        hasNeighbor = true;
                                    }
                                } else if (axis == 1)
                                {
                                    if (onMaxSide && iy + 1 < yCount)
                                    {
                                        neighborY = iy + 1;
                                        hasNeighbor = true;
                                    } else if (!onMaxSide && iy > 0)
                                    {
                                        neighborY = iy - 1;
                                        hasNeighbor = true;
                                    }
                                } else
                                {
                                    if (onMaxSide && iz + 1 < zCount)
                                    {
                                        neighborZ = iz + 1;
                                        hasNeighbor = true;
                                    } else if (!onMaxSide && iz > 0)
                                    {
                                        neighborZ = iz - 1;
                                        hasNeighbor = true;
                                    }
                                }

                                if (!hasNeighbor || !occupied[cellIndex(neighborX, neighborY, neighborZ)])
                                {
                                    faces.push_back(MakeAxisAlignedCellFace(minPoint, maxPoint, axis, onMaxSide));
                                }
                            }
                        }
                    }
                }
            }

            const PolyhedronBrepBodyConversion3d converted =
                ConvertToBrepBody(PolyhedronBody(std::move(faces)), epsilon);
            if (!converted.success || !converted.body.IsValid() || converted.body.ShellCount() != 1 ||
                !converted.body.ShellAt(0).IsClosed())
            {
                return false;
            }

            result = converted.body;
            return true;
        }

        [[nodiscard]] bool TryComputeSingleBoxDifference(const SCBox3d& first,
                                                         const SCBox3d& overlap,
                                                         double epsilon,
                                                         SCBox3d& difference)
        {
            std::vector<SCBox3d> candidates;
            candidates.reserve(6);

            auto addCandidate = [&](const SCPoint3d& minPoint, const SCPoint3d& maxPoint) {
                const SCBox3d candidate = SCBox3d::FromMinMax(minPoint, maxPoint);
                if (HasPositiveBoxVolume(candidate, epsilon))
                {
                    candidates.push_back(candidate);
                }
            };

            addCandidate(SCPoint3d{first.MinPoint().x, first.MinPoint().y, first.MinPoint().z},
                         SCPoint3d{overlap.MinPoint().x, first.MaxPoint().y, first.MaxPoint().z});
            addCandidate(SCPoint3d{overlap.MaxPoint().x, first.MinPoint().y, first.MinPoint().z},
                         SCPoint3d{first.MaxPoint().x, first.MaxPoint().y, first.MaxPoint().z});
            addCandidate(SCPoint3d{overlap.MinPoint().x, first.MinPoint().y, first.MinPoint().z},
                         SCPoint3d{overlap.MaxPoint().x, overlap.MinPoint().y, first.MaxPoint().z});
            addCandidate(SCPoint3d{overlap.MinPoint().x, overlap.MaxPoint().y, first.MinPoint().z},
                         SCPoint3d{overlap.MaxPoint().x, first.MaxPoint().y, first.MaxPoint().z});
            addCandidate(SCPoint3d{overlap.MinPoint().x, overlap.MinPoint().y, first.MinPoint().z},
                         SCPoint3d{overlap.MaxPoint().x, overlap.MaxPoint().y, overlap.MinPoint().z});
            addCandidate(SCPoint3d{overlap.MinPoint().x, overlap.MinPoint().y, overlap.MaxPoint().z},
                         SCPoint3d{overlap.MaxPoint().x, overlap.MaxPoint().y, first.MaxPoint().z});

            if (candidates.size() != 1)
            {
                return false;
            }

            difference = candidates.front();
            return NearlyEqualScaled(BoxVolume(difference), BoxVolume(first) - BoxVolume(overlap), epsilon);
        }

        [[nodiscard]] bool TryDetectFaceTouchingDifferenceIdentity(const SCBox3d& first,
                                                                   const SCBox3d& second,
                                                                   double epsilon)
        {
            if (!first.IsValid() || !second.IsValid())
            {
                return false;
            }

            std::size_t positiveOverlapAxes = 0;
            std::size_t touchingAxes = 0;
            for (int axis = 0; axis < 3; ++axis)
            {
                const double firstMin = BoxMinAt(first, axis);
                const double firstMax = BoxMaxAt(first, axis);
                const double secondMin = BoxMinAt(second, axis);
                const double secondMax = BoxMaxAt(second, axis);
                const double overlapMin = std::max(firstMin, secondMin);
                const double overlapMax = std::min(firstMax, secondMax);
                if (overlapMax > overlapMin + epsilon)
                {
                    ++positiveOverlapAxes;
                    continue;
                }

                const bool touchesOnMax = NearlyEqual(firstMax, secondMin, epsilon);
                const bool touchesOnMin = NearlyEqual(secondMax, firstMin, epsilon);
                if (!touchesOnMax && !touchesOnMin)
                {
                    return false;
                }

                ++touchingAxes;
            }

            return positiveOverlapAxes == 2U && touchingAxes == 1U;
        }

        [[nodiscard]] bool BoxContains(const SCBox3d& container, const SCBox3d& containee, double epsilon)
        {
            if (!container.IsValid() || !containee.IsValid())
            {
                return false;
            }

            return container.MinPoint().x <= containee.MinPoint().x + epsilon &&
                   container.MinPoint().y <= containee.MinPoint().y + epsilon &&
                   container.MinPoint().z <= containee.MinPoint().z + epsilon &&
                   container.MaxPoint().x >= containee.MaxPoint().x - epsilon &&
                   container.MaxPoint().y >= containee.MaxPoint().y - epsilon &&
                   container.MaxPoint().z >= containee.MaxPoint().z - epsilon;
        }

        [[nodiscard]] bool TryDetectNonVolumeTouchingIntersectionEmpty(const SCBox3d& first,
                                                                       const SCBox3d& second,
                                                                       double epsilon)
        {
            if (!first.IsValid() || !second.IsValid() || BoundsDisjoint(first, second, epsilon))
            {
                return false;
            }

            std::size_t positiveOverlapAxes = 0;
            std::size_t touchingAxes = 0;
            for (int axis = 0; axis < 3; ++axis)
            {
                const double firstMin = BoxMinAt(first, axis);
                const double firstMax = BoxMaxAt(first, axis);
                const double secondMin = BoxMinAt(second, axis);
                const double secondMax = BoxMaxAt(second, axis);
                const double overlapMin = std::max(firstMin, secondMin);
                const double overlapMax = std::min(firstMax, secondMax);
                if (overlapMax > overlapMin + epsilon)
                {
                    ++positiveOverlapAxes;
                    continue;
                }

                if (NearlyEqual(firstMax, secondMin, epsilon) || NearlyEqual(secondMax, firstMin, epsilon))
                {
                    ++touchingAxes;
                    continue;
                }

                return false;
            }

            return positiveOverlapAxes < 3U && touchingAxes > 0U;
        }

        [[nodiscard]] bool TryDetectEdgeOrVertexTouchingUnionAsMultiBody(const SCBox3d& first,
                                                                         const SCBox3d& second,
                                                                         double epsilon)
        {
            if (!TryDetectNonVolumeTouchingIntersectionEmpty(first, second, epsilon))
            {
                return false;
            }

            return !TryDetectFaceTouchingDifferenceIdentity(first, second, epsilon) &&
                   !TryDetectFaceTouchingDifferenceIdentity(second, first, epsilon);
        }

        [[nodiscard]] BodyBooleanResult3d IntersectClosedSubset(const SCBrepBody& first,
                                                                const SCBrepBody& second,
                                                                const BodyBooleanOptions3d& options)
        {
            const double epsilon = ResolveTolerance(options);
            if (BodiesAreGeometricallyEquivalent(first, second, epsilon, ResolveAngularTolerance(options)))
            {
                return MakeSingleBodyResult(first, "Deterministic identical-body intersection subset.");
            }
            if (BoundsDisjoint(first.Bounds(), second.Bounds(), epsilon))
            {
                return MakeEmptyResult("Deterministic disjoint-body empty intersection subset.");
            }

            SCBox3d firstBox;
            SCBox3d secondBox;
            SCBox3d overlapBox;
            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox))
            {
                if (BoxContains(firstBox, secondBox, epsilon))
                {
                    return MakeSingleBodyResult(second,
                                                "Deterministic axis-aligned contained-body "
                                                "intersection subset.");
                }

                if (BoxContains(secondBox, firstBox, epsilon))
                {
                    return MakeSingleBodyResult(first,
                                                "Deterministic axis-aligned contained-body "
                                                "intersection subset.");
                }
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryComputePositiveIntersectionBox(firstBox, secondBox, epsilon, overlapBox))
            {
                return MakeAxisAlignedBoxResult(overlapBox,
                                                epsilon,
                                                "Deterministic axis-aligned overlap-box intersection "
                                                "subset.");
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryDetectNonVolumeTouchingIntersectionEmpty(firstBox, secondBox, epsilon))
            {
                return MakeEmptyResult(
                    "Deterministic axis-aligned touching empty intersection "
                    "subset.");
            }

            SCBrepBody convexIntersection;
            if (TryBuildConvexIntersection(first, second, epsilon, ResolveAngularTolerance(options), convexIntersection))
            {
                return MakeSingleBodyResult(
                    std::move(convexIntersection), "Deterministic positive-volume convex polyhedral intersection subset.");
            }

            return MakeUnsupportedResult();
        }

        [[nodiscard]] BodyBooleanResult3d UnionClosedSubset(const SCBrepBody& first,
                                                            const SCBrepBody& second,
                                                            const BodyBooleanOptions3d& options)
        {
            const double epsilon = ResolveTolerance(options);
            if (BodiesAreGeometricallyEquivalent(first, second, epsilon, ResolveAngularTolerance(options)))
            {
                return MakeSingleBodyResult(first, "Deterministic identical-body union subset.");
            }
            if (BoundsDisjoint(first.Bounds(), second.Bounds(), epsilon))
            {
                return MakeMultiBodyResult({first, second}, "Deterministic disjoint-body union subset.");
            }

            SCBox3d firstBox;
            SCBox3d secondBox;
            SCBox3d overlapBox;
            SCBox3d unionBox;
            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox))
            {
                if (BoxContains(firstBox, secondBox, epsilon))
                {
                    return MakeSingleBodyResult(first,
                                                "Deterministic axis-aligned "
                                                "contained-body union subset.");
                }

                if (BoxContains(secondBox, firstBox, epsilon))
                {
                    return MakeSingleBodyResult(second,
                                                "Deterministic axis-aligned "
                                                "contained-body union subset.");
                }
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryComputeIntersectionBox(firstBox, secondBox, epsilon, overlapBox) &&
                TryComputeSingleBoxUnion(firstBox, secondBox, overlapBox, epsilon, unionBox))
            {
                return MakeAxisAlignedBoxResult(
                    unionBox, epsilon, "Deterministic axis-aligned overlap-box union subset.");
            }

            SCBrepBody orthogonalUnion;
            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryBuildFaceConnectedBoxUnion(firstBox, secondBox, epsilon, orthogonalUnion))
            {
                return MakeSingleBodyResult(std::move(orthogonalUnion),
                                            "Deterministic face-connected axis-aligned box union subset.");
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryDetectEdgeOrVertexTouchingUnionAsMultiBody(firstBox, secondBox, epsilon))
            {
                return MakeMultiBodyResult({first, second},
                                           "Deterministic axis-aligned edge/vertex-touching "
                                           "multi-body union subset.");
            }

            return MakeUnsupportedResult();
        }

        [[nodiscard]] BodyBooleanResult3d DifferenceClosedSubset(const SCBrepBody& first,
                                                                 const SCBrepBody& second,
                                                                 const BodyBooleanOptions3d& options)
        {
            const double epsilon = ResolveTolerance(options);
            if (BodiesAreGeometricallyEquivalent(first, second, epsilon, ResolveAngularTolerance(options)))
            {
                return MakeEmptyResult("Deterministic identical-body difference empty subset.");
            }

            if (BoundsDisjoint(first.Bounds(), second.Bounds(), epsilon))
            {
                return MakeSingleBodyResult(first, "Deterministic disjoint-body difference subset.");
            }

            SCBox3d firstBox;
            SCBox3d secondBox;
            SCBox3d overlapBox;
            SCBox3d differenceBox;
            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryComputePositiveIntersectionBox(firstBox, secondBox, epsilon, overlapBox) &&
                TryComputeSingleBoxDifference(firstBox, overlapBox, epsilon, differenceBox))
            {
                return MakeAxisAlignedBoxResult(differenceBox,
                                                epsilon,
                                                "Deterministic axis-aligned overlap-box difference "
                                                "subset.");
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) && BoxContains(secondBox, firstBox, epsilon))
            {
                return MakeEmptyResult(
                    "Deterministic axis-aligned contained difference empty "
                    "subset.");
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryDetectFaceTouchingDifferenceIdentity(firstBox, secondBox, epsilon))
            {
                return MakeSingleBodyResult(first, "Deterministic face-touching external difference subset.");
            }

            if (TryExtractAxisAlignedBox(first, epsilon, firstBox) &&
                TryExtractAxisAlignedBox(second, epsilon, secondBox) &&
                TryDetectEdgeOrVertexTouchingUnionAsMultiBody(firstBox, secondBox, epsilon))
            {
                return MakeSingleBodyResult(first,
                                            "Deterministic edge/vertex-touching external difference "
                                            "subset.");
            }

            return MakeUnsupportedResult();
        }

        [[nodiscard]] BodyBooleanResult3d MakeResultForBrepBodies(const SCBrepBody& first,
                                                                  const SCBrepBody& second,
                                                                  const BodyBooleanOptions3d& options,
                                                                  const char operation)
        {
            const double epsilon = ResolveTolerance(options);
            if (!options.tolerance.IsValid() || !HasFaces(first) || !HasFaces(second) ||
                !IsClosedManifoldSingleShell(first, epsilon) ||
                !IsClosedManifoldSingleShell(second, epsilon) || !HasConsistentEdgeEndpoints(first, epsilon) ||
                !HasConsistentEdgeEndpoints(second, epsilon) ||
                !HasPlanarFaceBoundariesConsistentWithSupport(first, epsilon) ||
                !HasPlanarFaceBoundariesConsistentWithSupport(second, epsilon))
            {
                return MakeInvalidInputResult();
            }
            if (!HasOnlyStraightEdges(first) || !HasOnlyStraightEdges(second))
            {
                return MakeUnsupportedResult();
            }

            switch (operation)
            {
                case 'i':
                    return IntersectClosedSubset(first, second, options);
                case 'u':
                    return UnionClosedSubset(first, second, options);
                case 'd':
                    return DifferenceClosedSubset(first, second, options);
                default:
                    return MakeUnsupportedResult();
            }
        }

        [[nodiscard]] BodyBooleanResult3d MakeResultForPolyhedronBodies(const PolyhedronBody& first,
                                                                        const PolyhedronBody& second,
                                                                        const BodyBooleanOptions3d& options,
                                                                        const char operation)
        {
            const double epsilon = ResolveTolerance(options);
            if (!options.tolerance.IsValid() || !HasFaces(first) || !HasFaces(second) || !first.IsValid(epsilon) ||
                !second.IsValid(epsilon))
            {
                return MakeInvalidInputResult();
            }

            const PolyhedronBrepBodyConversion3d firstConversion = ConvertToBrepBody(first, epsilon);
            const PolyhedronBrepBodyConversion3d secondConversion = ConvertToBrepBody(second, epsilon);
            if (!firstConversion.success || !secondConversion.success)
            {
                return MakeInvalidInputResult();
            }

            return MakeResultForBrepBodies(firstConversion.body, secondConversion.body, options, operation);
        }
    }  // namespace

    BodyBooleanResult3d IntersectBodies(const SCBrepBody& first, const SCBrepBody& second, BodyBooleanOptions3d options)
    {
        return MakeResultForBrepBodies(first, second, options, 'i');
    }

    BodyBooleanResult3d UnionBodies(const SCBrepBody& first, const SCBrepBody& second, BodyBooleanOptions3d options)
    {
        return MakeResultForBrepBodies(first, second, options, 'u');
    }

    BodyBooleanResult3d DifferenceBodies(const SCBrepBody& first, const SCBrepBody& second, BodyBooleanOptions3d options)
    {
        return MakeResultForBrepBodies(first, second, options, 'd');
    }

    BodyBooleanResult3d IntersectBodies(const PolyhedronBody& first,
                                        const PolyhedronBody& second,
                                        BodyBooleanOptions3d options)
    {
        return MakeResultForPolyhedronBodies(first, second, options, 'i');
    }

    BodyBooleanResult3d UnionBodies(const PolyhedronBody& first,
                                    const PolyhedronBody& second,
                                    BodyBooleanOptions3d options)
    {
        return MakeResultForPolyhedronBodies(first, second, options, 'u');
    }

    BodyBooleanResult3d DifferenceBodies(const PolyhedronBody& first,
                                         const PolyhedronBody& second,
                                         BodyBooleanOptions3d options)
    {
        return MakeResultForPolyhedronBodies(first, second, options, 'd');
    }
}  // namespace Geometry
