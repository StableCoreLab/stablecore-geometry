#include "Brep/BrepConversion.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Types/Geometry3d/SCLine3d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        struct FaceLoopRepresentativeIds
        {
            std::vector<std::size_t> outer;
            std::vector<std::vector<std::size_t>> holes;
        };

        struct RepresentativePointAccumulator
        {
            SCVector3d sum{};
            std::size_t count{0};
            bool hasPreferredPlane{false};
            SCPlane preferredPlane{};
        };

        struct NonPlanarRepairPassResult
        {
            PolyhedronBody body{};
            std::vector<FaceLoopRepresentativeIds> representativeIds;
            std::unordered_map<std::size_t, SCPoint3d> representativeTargetPoints;
        };

        [[nodiscard]] bool BuildFaceWithRefitSupportPlane(
            const PolyhedronFace3d& face,
            PolyhedronFace3d& repairedFace,
            double eps,
            const std::vector<bool>& preferredOuterVertices,
            const std::vector<std::size_t>* sourceOuterRepresentativeIds,
            const std::vector<std::vector<std::size_t>>* sourceHoleRepresentativeIds,
            FaceLoopRepresentativeIds* repairedRepresentativeIds);

        [[nodiscard]] std::size_t FindOrAddRepresentativePoint(const SCPoint3d& point,
                                                               std::vector<SCPoint3d>& representativePoints,
                                                               double eps)
        {
            for (std::size_t i = 0; i < representativePoints.size(); ++i)
            {
                if (representativePoints[i].AlmostEquals(point, eps))
                {
                    return i;
                }
            }

            representativePoints.push_back(point);
            return representativePoints.size() - 1;
        }

        [[nodiscard]] double ComputeBodyValidationEpsilon(const PolyhedronBody& body, double eps)
        {
            double maxDistanceSquared = 0.0;
            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const PolyhedronFace3d face = body.FaceAt(faceIndex);
                const auto accumulateLoop = [&](const PolyhedronLoop3d& loop) {
                    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                    {
                        for (std::size_t j = i + 1; j < loop.VertexCount(); ++j)
                        {
                            const double distanceSquared = (loop.VertexAt(i) - loop.VertexAt(j)).LengthSquared();
                            if (distanceSquared > maxDistanceSquared)
                            {
                                maxDistanceSquared = distanceSquared;
                            }
                        }
                    }
                };

                accumulateLoop(face.OuterLoop());
                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    accumulateLoop(face.HoleAt(holeIndex));
                }
            }

            if (maxDistanceSquared <= 0.0)
            {
                return eps;
            }

            return std::min(eps, 0.1 * maxDistanceSquared);
        }

        void BuildBodyLoopRepresentativeIds(const PolyhedronBody& body,
                                            std::vector<FaceLoopRepresentativeIds>& ids,
                                            double eps)
        {
            ids.clear();
            ids.resize(body.FaceCount());

            std::vector<SCPoint3d> representativePoints;
            const SCPlane referencePlane = body.FaceCount() > 0 ? body.FaceAt(0).SupportPlane() : SCPlane{};
            const SCVector3d referenceNormal = referencePlane.UnitNormal(eps);
            bool useCommonSupportPlaneGrouping =
                body.FaceCount() > 0 && body.FaceCount() <= 3 && referenceNormal.IsValid();
            if (useCommonSupportPlaneGrouping)
            {
                for (std::size_t faceIndex = 1; faceIndex < body.FaceCount(); ++faceIndex)
                {
                    const SCVector3d faceNormal = body.FaceAt(faceIndex).SupportPlane().UnitNormal(eps);
                    if (!faceNormal.IsValid() || (faceNormal - referenceNormal).Length() > eps)
                    {
                        useCommonSupportPlaneGrouping = false;
                        break;
                    }
                }
            }

            auto groupPoint = [&](const SCPoint3d& point) {
                if (!useCommonSupportPlaneGrouping)
                {
                    return point;
                }

                const double signedDistance = referencePlane.SignedDistanceTo(point, eps);
                return point - referenceNormal * signedDistance;
            };

            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const PolyhedronFace3d face = body.FaceAt(faceIndex);
                auto& faceIds = ids[faceIndex];

                faceIds.outer.reserve(face.OuterLoop().VertexCount());
                for (std::size_t i = 0; i < face.OuterLoop().VertexCount(); ++i)
                {
                    faceIds.outer.push_back(FindOrAddRepresentativePoint(groupPoint(face.OuterLoop().VertexAt(i)),
                                                                         representativePoints,
                                                                         Geometry::kRepresentativeGroupingEpsilon));
                }

                faceIds.holes.resize(face.HoleCount());
                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    const PolyhedronLoop3d hole = face.HoleAt(holeIndex);
                    auto& holeIds = faceIds.holes[holeIndex];
                    holeIds.reserve(hole.VertexCount());
                    for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                    {
                        holeIds.push_back(FindOrAddRepresentativePoint(groupPoint(hole.VertexAt(i)),
                                                                       representativePoints,
                                                                       Geometry::kRepresentativeGroupingEpsilon));
                    }
                }
            }
        }

        template <typename FaceAccessor>
        [[nodiscard]] bool ComputeRepresentativeTargetPointsForFaceRange(
            std::size_t faceCount,
            const FaceAccessor& faceAt,
            const std::vector<FaceLoopRepresentativeIds>& representativeIds,
            std::unordered_map<std::size_t, SCPoint3d>& representativeTargetPoints,
            double eps)
        {
            representativeTargetPoints.clear();
            if (representativeIds.size() != faceCount)
            {
                return false;
            }

            std::unordered_map<std::size_t, RepresentativePointAccumulator> accumulators;

            auto accumulateLoop = [&](const PolyhedronLoop3d& loop,
                                      const std::vector<std::size_t>& loopIds,
                                      const PolyhedronFace3d& face) {
                if (loop.VertexCount() != loopIds.size())
                {
                    return false;
                }

                bool hasPrevious = false;
                SCPoint3d previousPoint{};
                std::size_t previousId = 0;
                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    const SCPoint3d point = loop.VertexAt(i);
                    const std::size_t representativeId = loopIds[i];
                    if (hasPrevious && representativeId == previousId && point.AlmostEquals(previousPoint, eps))
                    {
                        continue;
                    }

                    auto& accumulator = accumulators[loopIds[i]];
                    accumulator.sum = accumulator.sum + (point - SCPoint3d{});
                    ++accumulator.count;
                    if (face.HoleCount() > 0 && !accumulator.hasPreferredPlane)
                    {
                        accumulator.hasPreferredPlane = true;
                        accumulator.preferredPlane = face.SupportPlane();
                    }

                    hasPrevious = true;
                    previousPoint = point;
                    previousId = representativeId;
                }
                return true;
            };

            for (std::size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex)
            {
                const PolyhedronFace3d face = faceAt(faceIndex);
                const FaceLoopRepresentativeIds& ids = representativeIds[faceIndex];

                if (!accumulateLoop(face.OuterLoop(), ids.outer, face))
                {
                    return false;
                }

                if (ids.holes.size() != face.HoleCount())
                {
                    return false;
                }

                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    if (!accumulateLoop(face.HoleAt(holeIndex), ids.holes[holeIndex], face))
                    {
                        return false;
                    }
                }
            }

            representativeTargetPoints.reserve(accumulators.size());
            for (const auto& [id, accumulator] : accumulators)
            {
                if (accumulator.count == 0)
                {
                    return false;
                }

                const SCVector3d average = accumulator.sum / static_cast<double>(accumulator.count);
                SCPoint3d target{average.x, average.y, average.z};
                if (accumulator.hasPreferredPlane)
                {
                    const SCVector3d unitNormal = accumulator.preferredPlane.UnitNormal(eps);
                    const double signedDistance = accumulator.preferredPlane.SignedDistanceTo(target, eps);
                    target = target - unitNormal * signedDistance;
                }
                representativeTargetPoints.emplace(id, target);
            }

            return true;
        }

        [[nodiscard]] bool ExecuteRepresentativeTargetAggregationPass(
            const std::vector<PolyhedronFace3d>& faces,
            const std::vector<FaceLoopRepresentativeIds>& representativeIds,
            std::unordered_map<std::size_t, SCPoint3d>& representativeTargetPoints)
        {
            return ComputeRepresentativeTargetPointsForFaceRange(
                faces.size(),
                [&faces](const std::size_t faceIndex) -> const PolyhedronFace3d& { return faces[faceIndex]; },
                representativeIds,
                representativeTargetPoints,
                Geometry::kRepresentativeMatchEpsilon);
        }

        [[nodiscard]] bool ExecuteRepresentativeTargetAggregationPass(
            const PolyhedronBody& body,
            const std::vector<FaceLoopRepresentativeIds>& representativeIds,
            std::unordered_map<std::size_t, SCPoint3d>& representativeTargetPoints)
        {
            return ComputeRepresentativeTargetPointsForFaceRange(
                body.FaceCount(),
                [&body](const std::size_t faceIndex) { return body.FaceAt(faceIndex); },
                representativeIds,
                representativeTargetPoints,
                Geometry::kRepresentativeMatchEpsilon);
        }

        [[nodiscard]] bool ExecuteCrossFaceSnappingPass(
            const std::vector<FaceLoopRepresentativeIds>& representativeIds,
            const std::unordered_map<std::size_t, SCPoint3d>& representativeTargetPoints,
            std::vector<PolyhedronFace3d>& faces,
            double eps)
        {
            if (representativeIds.size() != faces.size())
            {
                return false;
            }

            auto snapLoopToTargets = [&](const PolyhedronLoop3d& loop, const std::vector<std::size_t>& loopIds) {
                std::vector<SCPoint3d> snapped;
                snapped.reserve(loop.VertexCount());

                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    const auto targetIt = representativeTargetPoints.find(loopIds[i]);
                    const SCPoint3d target =
                        targetIt != representativeTargetPoints.end() ? targetIt->second : loop.VertexAt(i);
                    snapped.push_back(target);
                }

                return PolyhedronLoop3d(std::move(snapped));
            };

            for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
            {
                const PolyhedronFace3d& face = faces[faceIndex];
                const FaceLoopRepresentativeIds& ids = representativeIds[faceIndex];
                PolyhedronLoop3d outer = snapLoopToTargets(face.OuterLoop(), ids.outer);
                if (!outer.IsValid(eps))
                {
                    return false;
                }

                std::vector<PolyhedronLoop3d> holes;
                holes.reserve(face.HoleCount());
                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    PolyhedronLoop3d hole = snapLoopToTargets(face.HoleAt(holeIndex), ids.holes[holeIndex]);
                    if (!hole.IsValid(eps))
                    {
                        return false;
                    }
                    holes.push_back(std::move(hole));
                }

                PolyhedronFace3d snappedInput(face.SupportPlane(), std::move(outer), std::move(holes));
                if (snappedInput.IsValid(eps))
                {
                    faces[faceIndex] = std::move(snappedInput);
                    continue;
                }

                PolyhedronFace3d snappedFace{};
                const std::vector<bool> emptyPreferredOuterVertices;
                if (!BuildFaceWithRefitSupportPlane(
                        snappedInput, snappedFace, eps, emptyPreferredOuterVertices, &ids.outer, &ids.holes, nullptr))
                {
                    return false;
                }

                faces[faceIndex] = std::move(snappedFace);
            }

            return true;
        }

        [[nodiscard]] bool AppendLoopVerticesFromBody(const SCBrepBody& body,
                                                      const SCBrepLoop& loop,
                                                      std::vector<SCPoint3d>& vertices,
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
                if (vertices.empty() || !vertices.back().AlmostEquals(point, Geometry::kLoopCleanupEpsilon))
                {
                    vertices.push_back(point);
                }
            }

            while (vertices.size() >= 2 &&
                   vertices.front().AlmostEquals(vertices.back(), Geometry::kLoopCleanupEpsilon))
            {
                vertices.pop_back();
            }

            return vertices.size() >= 3;
        }

        [[nodiscard]] bool AppendLoopVerticesFromBrepTopology(const std::vector<SCBrepVertex>& vertices,
                                                              const std::vector<SCBrepEdge>& edges,
                                                              const SCBrepLoop& loop,
                                                              std::vector<SCPoint3d>& loopVertices)
        {
            loopVertices.clear();
            if (!loop.IsValid())
            {
                return false;
            }

            loopVertices.reserve(loop.CoedgeCount());
            for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
            {
                const SCBrepCoedge coedge = loop.CoedgeAt(i);
                if (coedge.EdgeIndex() >= edges.size())
                {
                    return false;
                }

                const SCBrepEdge& edge = edges[coedge.EdgeIndex()];
                const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
                if (vertexIndex >= vertices.size())
                {
                    return false;
                }

                loopVertices.push_back(vertices[vertexIndex].Point());
            }

            return loopVertices.size() >= 3;
        }

        [[nodiscard]] bool BuildLoopFromTrim(const SCCurveOnSurface& trim, std::vector<SCPoint3d>& vertices, double eps)
        {
            if (!trim.IsValid() || trim.PointCount() < 3)
            {
                return false;
            }

            vertices.clear();
            vertices.reserve(trim.PointCount());
            for (std::size_t i = 0; i < trim.PointCount(); ++i)
            {
                const SCPoint3d point = trim.PointAt(i);
                if (!point.IsValid())
                {
                    return false;
                }

                if (vertices.empty() || !vertices.back().AlmostEquals(point, Geometry::kLoopCleanupEpsilon))
                {
                    vertices.push_back(point);
                }
            }

            while (vertices.size() >= 2 &&
                   vertices.front().AlmostEquals(vertices.back(), Geometry::kLoopCleanupEpsilon))
            {
                vertices.pop_back();
            }

            return vertices.size() >= 3;
        }

        [[nodiscard]] bool BuildPolyhedronFaceFromBodyFace(const SCBrepBody& body,
                                                           const SCBrepFace& face,
                                                           PolyhedronFace3d& polyFace,
                                                           double eps)
        {
            const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
            if (planeSurface == nullptr)
            {
                return false;
            }

            std::vector<SCPoint3d> outerVertices;
            if (!AppendLoopVerticesFromBody(body, face.OuterLoop(), outerVertices, eps))
            {
                return false;
            }

            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                std::vector<SCPoint3d> holeVertices;
                if (!AppendLoopVerticesFromBody(body, face.HoleAt(i), holeVertices, eps))
                {
                    return false;
                }
                holes.emplace_back(std::move(holeVertices));
            }

            polyFace = PolyhedronFace3d(
                planeSurface->SupportPlane(), PolyhedronLoop3d(std::move(outerVertices)), std::move(holes));
            return polyFace.IsValid(eps);
        }

        [[nodiscard]] SCPoint2d ProjectPointToPlaneUv(const SCPoint3d& point, const SCPlaneSurface& planeSurface)
        {
            const SCPlane plane = planeSurface.SupportPlane();
            const SCVector3d delta = point - plane.origin;
            const SCVector3d uAxis = planeSurface.UAxis();
            const SCVector3d vAxis = planeSurface.VAxis();
            const double uDenom = std::max(uAxis.LengthSquared(), Geometry::kPlaneProjectionEpsilon);
            const double vDenom = std::max(vAxis.LengthSquared(), Geometry::kPlaneProjectionEpsilon);
            return SCPoint2d{Dot(delta, uAxis) / uDenom, Dot(delta, vAxis) / vDenom};
        }

        [[nodiscard]] bool AppendBrepLoopFromPolyLoop(const PolyhedronLoop3d& polyLoop,
                                                      std::vector<SCBrepVertex>& vertices,
                                                      std::vector<SCBrepEdge>& edges,
                                                      SCBrepLoop& loop,
                                                      std::vector<SCPoint2d>& uvPoints)
        {
            if (!polyLoop.IsValid())
            {
                return false;
            }

            const std::size_t vertexBase = vertices.size();
            const std::size_t vertexCount = polyLoop.VertexCount();
            vertices.reserve(vertexBase + vertexCount);
            for (std::size_t i = 0; i < vertexCount; ++i)
            {
                vertices.emplace_back(polyLoop.VertexAt(i));
            }

            std::vector<SCBrepCoedge> coedges;
            coedges.reserve(vertexCount);
            for (std::size_t i = 0; i < vertexCount; ++i)
            {
                const std::size_t next = (i + 1) % vertexCount;
                const SCPoint3d first = polyLoop.VertexAt(i);
                const SCPoint3d second = polyLoop.VertexAt(next);
                edges.emplace_back(std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                                       SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                                   vertexBase + i,
                                   vertexBase + next);
                coedges.emplace_back(edges.size() - 1, false);
            }

            loop = SCBrepLoop(std::move(coedges));
            uvPoints.clear();
            uvPoints.reserve(vertexCount);
            return loop.IsValid();
        }

        [[nodiscard]] std::size_t FindOrAddBrepVertex(const SCPoint3d& point,
                                                      std::vector<SCBrepVertex>& vertices,
                                                      double eps,
                                                      const SCPoint3d* preferredPoint = nullptr)
        {
            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                if (vertices[i].Point().AlmostEquals(point, eps))
                {
                    if (preferredPoint != nullptr && !vertices[i].Point().AlmostEquals(*preferredPoint, eps))
                    {
                        vertices[i] = SCBrepVertex(*preferredPoint);
                    }
                    return i;
                }
            }

            vertices.emplace_back(preferredPoint != nullptr ? *preferredPoint : point);
            return vertices.size() - 1;
        }

        [[nodiscard]] bool FindReusableBrepEdge(const std::vector<SCBrepEdge>& edges,
                                                std::size_t startVertexIndex,
                                                std::size_t endVertexIndex,
                                                std::size_t& edgeIndex,
                                                bool& reversed)
        {
            for (std::size_t i = 0; i < edges.size(); ++i)
            {
                const SCBrepEdge& edge = edges[i];
                if (edge.StartVertexIndex() == startVertexIndex && edge.EndVertexIndex() == endVertexIndex)
                {
                    edgeIndex = i;
                    reversed = false;
                    return true;
                }

                if (edge.StartVertexIndex() == endVertexIndex && edge.EndVertexIndex() == startVertexIndex)
                {
                    edgeIndex = i;
                    reversed = true;
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool AppendSharedBrepLoopFromPolyLoop(
            const PolyhedronLoop3d& polyLoop,
            std::vector<SCBrepVertex>& vertices,
            std::vector<SCBrepEdge>& edges,
            SCBrepLoop& loop,
            std::vector<SCPoint2d>& uvPoints,
            const std::vector<std::size_t>* representativeIds,
            const std::unordered_map<std::size_t, SCPoint3d>* representativeTargetPoints,
            const SCPlane* representativeTargetPlane,
            bool useAxisAlignedRepresentativeSnapping,
            std::unordered_map<std::size_t, std::size_t>* representativeToVertexIndex,
            double vertexDedupEps,
            double eps)
        {
            try
            {
                if (!polyLoop.IsValid(eps))
                {
                    return false;
                }

                auto snapToRepresentativePlaneAxis = [&](SCPoint3d point) {
                    if (representativeTargetPlane == nullptr || !useAxisAlignedRepresentativeSnapping)
                    {
                        return point;
                    }

                    const SCVector3d unitNormal = representativeTargetPlane->UnitNormal(eps);
                    if (!unitNormal.IsValid())
                    {
                        return point;
                    }

                    const SCVector3d absNormal{std::abs(unitNormal.x), std::abs(unitNormal.y), std::abs(unitNormal.z)};
                    if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z)
                    {
                        point.x = representativeTargetPlane->origin.x;
                    } else if (absNormal.y >= absNormal.z)
                    {
                        point.y = representativeTargetPlane->origin.y;
                    } else
                    {
                        point.z = representativeTargetPlane->origin.z;
                    }

                    return point;
                };

                const std::size_t vertexCount = polyLoop.VertexCount();
                std::vector<std::size_t> loopVertexIndices;
                loopVertexIndices.reserve(vertexCount);

                const bool hasRepresentativeIds = representativeIds != nullptr &&
                                                  representativeToVertexIndex != nullptr &&
                                                  representativeIds->size() == vertexCount;

                for (std::size_t i = 0; i < vertexCount; ++i)
                {
                    const SCPoint3d point = polyLoop.VertexAt(i);
                    if (hasRepresentativeIds)
                    {
                        if (i >= representativeIds->size())
                        {
                            return false;
                        }

                        const std::size_t representativeId = (*representativeIds)[i];
                        const auto found = representativeToVertexIndex->find(representativeId);
                        if (found != representativeToVertexIndex->end())
                        {
                            if (found->second >= vertices.size())
                            {
                                return false;
                            }

                            if (representativeTargetPoints != nullptr)
                            {
                                const auto representativePointIt = representativeTargetPoints->find(representativeId);
                                if (representativePointIt != representativeTargetPoints->end() &&
                                    !vertices[found->second].Point().AlmostEquals(representativePointIt->second,
                                                                                  vertexDedupEps))
                                {
                                    vertices[found->second] =
                                        SCBrepVertex(snapToRepresentativePlaneAxis(representativePointIt->second));
                                }
                            }

                            loopVertexIndices.push_back(found->second);
                        } else
                        {
                            SCPoint3d representativePoint = point;
                            if (representativeTargetPoints != nullptr)
                            {
                                const auto representativePointIt = representativeTargetPoints->find(representativeId);
                                if (representativePointIt != representativeTargetPoints->end())
                                {
                                    representativePoint = snapToRepresentativePlaneAxis(representativePointIt->second);
                                }
                            }

                            const std::size_t vertexIndex = FindOrAddBrepVertex(
                                representativePoint, vertices, vertexDedupEps, &representativePoint);
                            (*representativeToVertexIndex)[representativeId] = vertexIndex;
                            loopVertexIndices.push_back(vertexIndex);
                        }
                    } else
                    {
                        loopVertexIndices.push_back(FindOrAddBrepVertex(point, vertices, vertexDedupEps));
                    }
                }

                std::vector<SCBrepCoedge> coedges;
                coedges.reserve(vertexCount);

                if (loopVertexIndices.size() != vertexCount)
                {
                    return false;
                }

                for (std::size_t i = 0; i < vertexCount; ++i)
                {
                    const std::size_t next = (i + 1) % vertexCount;
                    if (i >= loopVertexIndices.size() || next >= loopVertexIndices.size())
                    {
                        return false;
                    }

                    const std::size_t startVertexIndex = loopVertexIndices[i];
                    const std::size_t endVertexIndex = loopVertexIndices[next];
                    if (startVertexIndex >= vertices.size() || endVertexIndex >= vertices.size())
                    {
                        return false;
                    }

                    std::size_t edgeIndex = static_cast<std::size_t>(-1);
                    bool reversed = false;
                    if (!FindReusableBrepEdge(edges, startVertexIndex, endVertexIndex, edgeIndex, reversed))
                    {
                        const SCPoint3d first = vertices[startVertexIndex].Point();
                        const SCPoint3d second = vertices[endVertexIndex].Point();
                        edges.emplace_back(
                            std::make_shared<SCLineCurve3d>(SCLineCurve3d::FromLine(
                                SCLine3d::FromOriginAndDirection(first, second - first), SCIntervald{0.0, 1.0})),
                            startVertexIndex,
                            endVertexIndex);
                        edgeIndex = edges.size() - 1;
                    }

                    coedges.emplace_back(edgeIndex, reversed);
                }

                loop = SCBrepLoop(std::move(coedges));
                uvPoints.clear();
                uvPoints.reserve(vertexCount);
                return loop.IsValid();
            } catch (const std::exception&)
            {
                return false;
            }
        }

        void BuildSharedOuterVertexPreferenceMask(const PolyhedronBody& body,
                                                  std::vector<std::vector<bool>>& facePreferredOuterVertices,
                                                  double eps)
        {
            facePreferredOuterVertices.clear();
            facePreferredOuterVertices.resize(body.FaceCount());

            std::vector<SCPoint3d> representatives;
            std::vector<std::size_t> representativeCounts;
            std::vector<std::vector<std::size_t>> faceRepresentativeIndices(body.FaceCount());
            const double matchEps = std::max(eps, Geometry::kSharedTopologyMatchEpsilon);

            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const PolyhedronLoop3d outer = body.FaceAt(faceIndex).OuterLoop();
                auto& indices = faceRepresentativeIndices[faceIndex];
                indices.reserve(outer.VertexCount());

                for (std::size_t vertexIndex = 0; vertexIndex < outer.VertexCount(); ++vertexIndex)
                {
                    const SCPoint3d point = outer.VertexAt(vertexIndex);

                    std::size_t representativeIndex = static_cast<std::size_t>(-1);
                    for (std::size_t i = 0; i < representatives.size(); ++i)
                    {
                        if (representatives[i].AlmostEquals(point, matchEps))
                        {
                            representativeIndex = i;
                            break;
                        }
                    }

                    if (representativeIndex == static_cast<std::size_t>(-1))
                    {
                        representatives.push_back(point);
                        representativeCounts.push_back(0);
                        representativeIndex = representatives.size() - 1;
                    }

                    ++representativeCounts[representativeIndex];
                    indices.push_back(representativeIndex);
                }
            }

            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const auto& indices = faceRepresentativeIndices[faceIndex];
                auto& preferred = facePreferredOuterVertices[faceIndex];
                preferred.resize(indices.size(), false);
                for (std::size_t i = 0; i < indices.size(); ++i)
                {
                    preferred[i] = representativeCounts[indices[i]] > 1;
                }
            }
        }

        [[nodiscard]] bool ComputeSharedShellClosed(const PolyhedronBody& body,
                                                    const std::vector<FaceLoopRepresentativeIds>& representativeIds)
        {
            if (representativeIds.size() != body.FaceCount())
            {
                return false;
            }

            std::map<std::pair<std::size_t, std::size_t>, std::size_t> edgeUseCounts;
            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const PolyhedronFace3d face = body.FaceAt(faceIndex);
                const FaceLoopRepresentativeIds& ids = representativeIds[faceIndex];

                auto accumulateLoop = [&](const PolyhedronLoop3d& loop, const std::vector<std::size_t>& loopIds) {
                    if (loop.VertexCount() != loopIds.size())
                    {
                        return false;
                    }

                    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                    {
                        const std::size_t next = (i + 1) % loop.VertexCount();
                        ++edgeUseCounts[std::minmax(loopIds[i], loopIds[next])];
                    }

                    return true;
                };

                if (!accumulateLoop(face.OuterLoop(), ids.outer))
                {
                    return false;
                }

                if (ids.holes.size() != face.HoleCount())
                {
                    return false;
                }

                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    if (!accumulateLoop(face.HoleAt(holeIndex), ids.holes[holeIndex]))
                    {
                        return false;
                    }
                }
            }

            if (edgeUseCounts.empty())
            {
                return false;
            }

            for (const auto& [edgeKey, count] : edgeUseCounts)
            {
                (void)edgeKey;
                if (count != 2)
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] bool BuildFaceWithRefitSupportPlane(
            const PolyhedronFace3d& face,
            PolyhedronFace3d& repairedFace,
            double eps,
            const std::vector<bool>& preferredOuterVertices,
            const std::vector<std::size_t>* sourceOuterRepresentativeIds,
            const std::vector<std::vector<std::size_t>>* sourceHoleRepresentativeIds,
            FaceLoopRepresentativeIds* repairedRepresentativeIds)
        {
            const double supportEps = std::min(eps, Geometry::kBooleanComparisonEpsilon);
            auto normalizeLoop =
                [&](const PolyhedronLoop3d& loop, PolyhedronLoop3d& normalized, std::vector<std::size_t>* keptIndices) {
                    std::vector<SCPoint3d> vertices;
                    if (keptIndices != nullptr)
                    {
                        keptIndices->clear();
                        keptIndices->reserve(loop.VertexCount());
                    }

                    vertices.reserve(loop.VertexCount());
                    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                    {
                        const SCPoint3d point = loop.VertexAt(i);
                        if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
                        {
                            vertices.push_back(point);
                            if (keptIndices != nullptr)
                            {
                                keptIndices->push_back(i);
                            }
                        }
                    }

                    while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), supportEps))
                    {
                        vertices.pop_back();
                        if (keptIndices != nullptr && !keptIndices->empty())
                        {
                            keptIndices->pop_back();
                        }
                    }

                    normalized = PolyhedronLoop3d(std::move(vertices));
                    return normalized.IsValid(supportEps);
                };

            PolyhedronLoop3d outer{};
            std::vector<std::size_t> outerKeptIndices;
            if (!normalizeLoop(face.OuterLoop(), outer, &outerKeptIndices) || outer.VertexCount() < 3)
            {
                return false;
            }

            std::vector<std::size_t> normalizedOuterRepresentativeIds;
            if (sourceOuterRepresentativeIds != nullptr &&
                sourceOuterRepresentativeIds->size() == face.OuterLoop().VertexCount())
            {
                normalizedOuterRepresentativeIds.reserve(outerKeptIndices.size());
                for (const std::size_t sourceIndex : outerKeptIndices)
                {
                    normalizedOuterRepresentativeIds.push_back((*sourceOuterRepresentativeIds)[sourceIndex]);
                }
            }

            std::vector<bool> normalizedPreferredOuter(outer.VertexCount(), false);
            if (!preferredOuterVertices.empty())
            {
                for (std::size_t i = 0; i < outerKeptIndices.size(); ++i)
                {
                    const std::size_t sourceIndex = outerKeptIndices[i];
                    if (sourceIndex < preferredOuterVertices.size())
                    {
                        normalizedPreferredOuter[i] = preferredOuterVertices[sourceIndex];
                    }
                }
            }

            auto maxLoopScaleSquared = [&](const PolyhedronLoop3d& loop) {
                double maxDistanceSquared = 0.0;
                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    for (std::size_t j = i + 1; j < loop.VertexCount(); ++j)
                    {
                        const double distanceSquared = (loop.VertexAt(i) - loop.VertexAt(j)).LengthSquared();
                        if (distanceSquared > maxDistanceSquared)
                        {
                            maxDistanceSquared = distanceSquared;
                        }
                    }
                }

                return std::max(maxDistanceSquared, supportEps * supportEps);
            };
            const double faceValidityEps = std::min(eps, 0.1 * maxLoopScaleSquared(outer));

            std::vector<PolyhedronLoop3d> holes;
            std::vector<std::vector<std::size_t>> normalizedHoleRepresentativeIds;
            holes.reserve(face.HoleCount());
            normalizedHoleRepresentativeIds.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                PolyhedronLoop3d hole{};
                std::vector<std::size_t> keptIndices;
                if (!normalizeLoop(face.HoleAt(i), hole, &keptIndices))
                {
                    return false;
                }
                holes.push_back(hole);

                std::vector<std::size_t> normalizedHoleIds;
                if (sourceHoleRepresentativeIds != nullptr && i < sourceHoleRepresentativeIds->size() &&
                    (*sourceHoleRepresentativeIds)[i].size() == face.HoleAt(i).VertexCount())
                {
                    const auto& sourceHoleIds = (*sourceHoleRepresentativeIds)[i];
                    normalizedHoleIds.reserve(keptIndices.size());
                    for (const std::size_t sourceIndex : keptIndices)
                    {
                        normalizedHoleIds.push_back(sourceHoleIds[sourceIndex]);
                    }
                }
                normalizedHoleRepresentativeIds.push_back(std::move(normalizedHoleIds));
            }

            bool foundSupport = false;
            SCPoint3d p0{};
            SCVector3d normal{};
            double bestNormalLength = -1.0;
            SCPoint3d bestP0{};
            SCVector3d bestNormal{};
            double bestHoleDistance = std::numeric_limits<double>::infinity();
            double bestTotalDistance = std::numeric_limits<double>::infinity();
            double bestMaxDistance = std::numeric_limits<double>::infinity();
            std::size_t bestOnPlaneCount = 0;
            int bestPreferredCount = -1;
            double bestPreferredNormalLength = -1.0;

            struct CandidatePoint
            {
                SCPoint3d point;
                bool preferred{false};
            };

            std::vector<CandidatePoint> candidatePoints;
            candidatePoints.reserve(
                outer.VertexCount() +
                std::accumulate(
                    holes.begin(), holes.end(), std::size_t{0}, [](std::size_t total, const PolyhedronLoop3d& loop) {
                        return total + loop.VertexCount();
                    }));
            for (std::size_t i = 0; i < outer.VertexCount(); ++i)
            {
                candidatePoints.push_back(CandidatePoint{outer.VertexAt(i), normalizedPreferredOuter[i]});
            }
            for (const PolyhedronLoop3d& hole : holes)
            {
                for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                {
                    candidatePoints.push_back(CandidatePoint{hole.VertexAt(i), false});
                }
            }

            const int preferredOuterCount =
                static_cast<int>(std::count(normalizedPreferredOuter.begin(), normalizedPreferredOuter.end(), true));
            const bool preferQuadOutlierPlane = holes.empty() && outer.VertexCount() == 4 && preferredOuterCount == 3;
            const bool preferHoleDominatedPlane = !holes.empty();

            // For a mildly distorted quad face with exactly one non-shared
            // outer vertex, prefer the plane defined by the three shared
            // vertices so the displaced corner snaps back onto the shared
            // topology instead of becoming a separate BRep vertex.
            auto scorePlane = [&](const SCPlane& plane) {
                std::size_t onPlaneCount = 0;
                double holeDistance = 0.0;
                double totalDistance = 0.0;
                double maxDistance = 0.0;

                const auto accumulateLoopDistance = [&](const PolyhedronLoop3d& loop, double weight) {
                    for (std::size_t vertexIndex = 0; vertexIndex < loop.VertexCount(); ++vertexIndex)
                    {
                        const double distance = std::abs(plane.SignedDistanceTo(loop.VertexAt(vertexIndex), eps));
                        if (distance <= Geometry::kSupportPlaneOnPlaneEpsilon)
                        {
                            ++onPlaneCount;
                        }
                        totalDistance += weight * distance;
                        maxDistance = std::max(maxDistance, distance);
                    }
                };

                accumulateLoopDistance(outer, 1.0);
                for (const PolyhedronLoop3d& hole : holes)
                {
                    const double beforeHoleDistance = totalDistance;
                    accumulateLoopDistance(hole, Geometry::kSupportPlaneHoleDistanceWeight);
                    holeDistance += totalDistance - beforeHoleDistance;
                }

                return std::tuple<std::size_t, double, double, double>{
                    onPlaneCount, holeDistance, totalDistance, maxDistance};
            };

            if (preferHoleDominatedPlane)
            {
                bool seededHolePlane = false;
                for (const PolyhedronLoop3d& hole : holes)
                {
                    for (std::size_t i = 0; i + 2 < hole.VertexCount() && !seededHolePlane; ++i)
                    {
                        const SCPoint3d a = hole.VertexAt(i);
                        for (std::size_t j = i + 1; j + 1 < hole.VertexCount() && !seededHolePlane; ++j)
                        {
                            const SCPoint3d b = hole.VertexAt(j);
                            for (std::size_t k = j + 1; k < hole.VertexCount(); ++k)
                            {
                                const SCPoint3d c = hole.VertexAt(k);
                                const SCVector3d seedNormal = Cross(b - a, c - a);
                                if (seedNormal.Length() <= supportEps)
                                {
                                    continue;
                                }

                                const SCPlane seedPlane = SCPlane::FromPointAndNormal(a, seedNormal);
                                const auto [onPlaneCount, holeDistance, totalDistance, maxDistance] =
                                    scorePlane(seedPlane);
                                if (holeDistance >
                                    Geometry::kSupportPlaneOnPlaneEpsilon * static_cast<double>(hole.VertexCount()))
                                {
                                    continue;
                                }

                                foundSupport = true;
                                p0 = a;
                                normal = seedNormal;
                                bestHoleDistance = holeDistance;
                                bestTotalDistance = totalDistance;
                                bestMaxDistance = maxDistance;
                                bestOnPlaneCount = onPlaneCount;
                                bestPreferredCount = 0;
                                bestPreferredNormalLength = seedNormal.Length();
                                seededHolePlane = true;
                            }
                        }
                    }
                }

                if (foundSupport)
                {
                    const SCVector3d refitNormal = normal.Normalized(supportEps);
                    if (!refitNormal.IsValid())
                    {
                        return false;
                    }

                    const SCPlane refitPlane = SCPlane::FromPointAndNormal(p0, refitNormal);

                    if (holes.empty())
                    {
                        repairedFace = PolyhedronFace3d(refitPlane, outer, holes);
                    } else
                    {
                        auto projectLoopToPlane = [&](const PolyhedronLoop3d& loop) {
                            std::vector<SCPoint3d> projected;
                            projected.reserve(loop.VertexCount());
                            const SCVector3d unitNormal = refitPlane.UnitNormal(eps);
                            for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                            {
                                const SCPoint3d point = loop.VertexAt(i);
                                const double signedDistance = refitPlane.SignedDistanceTo(point, eps);
                                projected.push_back(point - unitNormal * signedDistance);
                            }
                            return PolyhedronLoop3d(std::move(projected));
                        };

                        PolyhedronLoop3d projectedOuter = projectLoopToPlane(outer);
                        std::vector<PolyhedronLoop3d> projectedHoles;
                        projectedHoles.reserve(holes.size());
                        for (const PolyhedronLoop3d& hole : holes)
                        {
                            projectedHoles.push_back(projectLoopToPlane(hole));
                        }
                        repairedFace =
                            PolyhedronFace3d(refitPlane, std::move(projectedOuter), std::move(projectedHoles));
                        if (!repairedFace.IsValid(faceValidityEps))
                        {
                            std::vector<SCPoint3d> reversedOuterVertices = repairedFace.OuterLoop().Vertices();
                            std::reverse(reversedOuterVertices.begin(), reversedOuterVertices.end());
                            std::vector<PolyhedronLoop3d> reversedProjectedHoles;
                            reversedProjectedHoles.reserve(repairedFace.HoleCount());
                            for (std::size_t holeIndex = 0; holeIndex < repairedFace.HoleCount(); ++holeIndex)
                            {
                                std::vector<SCPoint3d> reversedHoleVertices = repairedFace.HoleAt(holeIndex).Vertices();
                                std::reverse(reversedHoleVertices.begin(), reversedHoleVertices.end());
                                reversedProjectedHoles.emplace_back(std::move(reversedHoleVertices));
                            }
                            repairedFace = PolyhedronFace3d(refitPlane,
                                                            PolyhedronLoop3d(std::move(reversedOuterVertices)),
                                                            std::move(reversedProjectedHoles));
                        }
                    }

                    if (repairedRepresentativeIds != nullptr)
                    {
                        repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
                        repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
                    }
                    return true;
                }
            }

            for (std::size_t i = 0; i + 2 < candidatePoints.size(); ++i)
            {
                const SCPoint3d a = candidatePoints[i].point;
                for (std::size_t j = i + 1; j + 1 < candidatePoints.size(); ++j)
                {
                    const SCPoint3d b = candidatePoints[j].point;
                    for (std::size_t k = j + 1; k < candidatePoints.size(); ++k)
                    {
                        const SCPoint3d c = candidatePoints[k].point;
                        const SCVector3d candidateNormal = Cross(b - a, c - a);
                        const double candidateLength = candidateNormal.Length();
                        if (candidateLength > bestNormalLength)
                        {
                            bestNormalLength = candidateLength;
                            bestP0 = a;
                            bestNormal = candidateNormal;
                        }

                        if (candidateLength <= supportEps)
                        {
                            continue;
                        }

                        const SCPlane candidatePlane = SCPlane::FromPointAndNormal(a, candidateNormal);
                        const auto [onPlaneCount, holeDistance, totalDistance, maxDistance] =
                            scorePlane(candidatePlane);
                        const int preferredCount = static_cast<int>(candidatePoints[i].preferred) +
                                                   static_cast<int>(candidatePoints[j].preferred) +
                                                   static_cast<int>(candidatePoints[k].preferred);

                        bool betterFit = !foundSupport;
                        if (!betterFit)
                        {
                            if (preferQuadOutlierPlane)
                            {
                                const bool betterTotalDistance = totalDistance + eps < bestTotalDistance;
                                const bool sameTotalDistance = std::abs(totalDistance - bestTotalDistance) <= eps;
                                const bool betterMaxDistance = maxDistance > bestMaxDistance + eps;
                                const bool sameMaxDistance = std::abs(maxDistance - bestMaxDistance) <= eps;
                                const bool betterPreferredCount = preferredCount > bestPreferredCount;
                                const bool samePreferredCount = preferredCount == bestPreferredCount;
                                betterFit = betterTotalDistance ||
                                            (sameTotalDistance &&
                                             (betterMaxDistance ||
                                              (sameMaxDistance &&
                                               (betterPreferredCount ||
                                                (samePreferredCount && candidateLength > bestPreferredNormalLength)))));
                            } else if (preferHoleDominatedPlane)
                            {
                                const bool betterHoleDistance = holeDistance + eps < bestHoleDistance;
                                const bool sameHoleDistance = std::abs(holeDistance - bestHoleDistance) <= eps;
                                const bool betterTotalDistance = totalDistance + eps < bestTotalDistance;
                                const bool sameTotalDistance = std::abs(totalDistance - bestTotalDistance) <= eps;
                                const bool betterMaxDistance = maxDistance + eps < bestMaxDistance;
                                const bool sameMaxDistance = std::abs(maxDistance - bestMaxDistance) <= eps;
                                const bool betterPreferredCount = preferredCount > bestPreferredCount;
                                const bool samePreferredCount = preferredCount == bestPreferredCount;
                                betterFit =
                                    betterHoleDistance ||
                                    (sameHoleDistance &&
                                     (betterTotalDistance ||
                                      (sameTotalDistance &&
                                       (betterMaxDistance ||
                                        (sameMaxDistance &&
                                         (betterPreferredCount ||
                                          (samePreferredCount && candidateLength > bestPreferredNormalLength)))))));
                            } else
                            {
                                const bool betterOnPlaneCount = onPlaneCount > bestOnPlaneCount;
                                const bool sameOnPlaneCount = onPlaneCount == bestOnPlaneCount;
                                const bool betterTotalDistance = totalDistance + eps < bestTotalDistance;
                                const bool sameTotalDistance = std::abs(totalDistance - bestTotalDistance) <= eps;
                                const bool betterMaxDistance = maxDistance + eps < bestMaxDistance;
                                const bool sameMaxDistance = std::abs(maxDistance - bestMaxDistance) <= eps;
                                const bool betterPreferredCount = preferredCount > bestPreferredCount;
                                const bool samePreferredCount = preferredCount == bestPreferredCount;
                                betterFit =
                                    betterOnPlaneCount ||
                                    (sameOnPlaneCount &&
                                     (betterTotalDistance ||
                                      (sameTotalDistance &&
                                       (betterMaxDistance ||
                                        (sameMaxDistance &&
                                         (betterPreferredCount ||
                                          (samePreferredCount && candidateLength > bestPreferredNormalLength)))))));
                            }
                        }
                        if (betterFit)
                        {
                            foundSupport = true;
                            p0 = a;
                            normal = candidateNormal;
                            bestHoleDistance = holeDistance;
                            bestTotalDistance = totalDistance;
                            bestMaxDistance = maxDistance;
                            bestOnPlaneCount = onPlaneCount;
                            bestPreferredCount = preferredCount;
                            bestPreferredNormalLength = candidateLength;
                        }
                    }
                }
            }

            if (!foundSupport)
            {
                const double scaleAwareThreshold = maxLoopScaleSquared(outer) * eps;
                if (bestNormalLength <= scaleAwareThreshold)
                {
                    return false;
                }

                p0 = bestP0;
                normal = bestNormal;
            }

            const SCVector3d refitNormal = normal.Normalized(supportEps);
            if (!refitNormal.IsValid())
            {
                return false;
            }

            const SCPlane refitPlane = SCPlane::FromPointAndNormal(p0, refitNormal);

            if (holes.empty())
            {
                repairedFace = PolyhedronFace3d(refitPlane, outer, holes);
                if (repairedFace.IsValid(faceValidityEps))
                {
                    if (repairedRepresentativeIds != nullptr)
                    {
                        repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
                        repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
                    }
                    return true;
                }

                std::vector<SCPoint3d> reversedOuterVertices = outer.Vertices();
                std::reverse(reversedOuterVertices.begin(), reversedOuterVertices.end());
                repairedFace = PolyhedronFace3d(refitPlane, PolyhedronLoop3d(std::move(reversedOuterVertices)), holes);
                if (repairedFace.IsValid(faceValidityEps))
                {
                    if (repairedRepresentativeIds != nullptr)
                    {
                        repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
                        repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
                    }
                    return true;
                }
            }

            auto projectLoopToPlane = [&](const PolyhedronLoop3d& loop) {
                std::vector<SCPoint3d> projected;
                projected.reserve(loop.VertexCount());
                const SCVector3d unitNormal = refitPlane.UnitNormal(eps);
                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    const SCPoint3d point = loop.VertexAt(i);
                    const double signedDistance = refitPlane.SignedDistanceTo(point, eps);
                    projected.push_back(point - unitNormal * signedDistance);
                }
                return PolyhedronLoop3d(std::move(projected));
            };

            PolyhedronLoop3d projectedOuter = projectLoopToPlane(outer);
            if (!projectedOuter.IsValid(eps))
            {
                return false;
            }

            std::vector<PolyhedronLoop3d> projectedHoles;
            projectedHoles.reserve(holes.size());
            for (const PolyhedronLoop3d& hole : holes)
            {
                PolyhedronLoop3d projectedHole = projectLoopToPlane(hole);
                if (!projectedHole.IsValid(eps))
                {
                    return false;
                }
                projectedHoles.push_back(std::move(projectedHole));
            }

            repairedFace = PolyhedronFace3d(refitPlane, std::move(projectedOuter), std::move(projectedHoles));
            if (!repairedFace.IsValid(faceValidityEps))
            {
                std::vector<SCPoint3d> reversedOuterVertices = projectedOuter.Vertices();
                std::reverse(reversedOuterVertices.begin(), reversedOuterVertices.end());
                projectedOuter = PolyhedronLoop3d(std::move(reversedOuterVertices));
                std::vector<PolyhedronLoop3d> reversedProjectedHoles;
                reversedProjectedHoles.reserve(projectedHoles.size());
                for (PolyhedronLoop3d hole : projectedHoles)
                {
                    std::vector<SCPoint3d> reversedHoleVertices = hole.Vertices();
                    std::reverse(reversedHoleVertices.begin(), reversedHoleVertices.end());
                    reversedProjectedHoles.emplace_back(std::move(reversedHoleVertices));
                }
                repairedFace =
                    PolyhedronFace3d(refitPlane, std::move(projectedOuter), std::move(reversedProjectedHoles));
                if (!repairedFace.IsValid(faceValidityEps))
                {
                    return false;
                }

                if (repairedRepresentativeIds != nullptr)
                {
                    repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
                    repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
                }
                return true;
            }

            if (repairedRepresentativeIds != nullptr)
            {
                repairedRepresentativeIds->outer = std::move(normalizedOuterRepresentativeIds);
                repairedRepresentativeIds->holes = std::move(normalizedHoleRepresentativeIds);
            }
            return true;
        }

        [[nodiscard]] bool ExecuteSupportPlaneScoringPass(
            const PolyhedronBody& body,
            const std::vector<std::vector<bool>>& facePreferredOuterVertices,
            const std::vector<FaceLoopRepresentativeIds>* sourceRepresentativeIds,
            std::vector<PolyhedronFace3d>& repairedFaces,
            std::vector<FaceLoopRepresentativeIds>& repairedRepresentativeIds,
            double eps)
        {
            repairedFaces.clear();
            repairedFaces.reserve(body.FaceCount());
            repairedRepresentativeIds.clear();
            repairedRepresentativeIds.resize(body.FaceCount());
            const std::vector<bool> emptyPreferredOuterVertices;

            for (std::size_t faceIndex = 0; faceIndex < body.FaceCount(); ++faceIndex)
            {
                const std::vector<bool>& preferredOuterVertices = faceIndex < facePreferredOuterVertices.size()
                                                                      ? facePreferredOuterVertices[faceIndex]
                                                                      : emptyPreferredOuterVertices;
                const std::vector<std::size_t>* sourceOuterIds = nullptr;
                const std::vector<std::vector<std::size_t>>* sourceHoleIds = nullptr;
                if (sourceRepresentativeIds != nullptr && faceIndex < sourceRepresentativeIds->size())
                {
                    sourceOuterIds = &(*sourceRepresentativeIds)[faceIndex].outer;
                    sourceHoleIds = &(*sourceRepresentativeIds)[faceIndex].holes;
                }

                FaceLoopRepresentativeIds repairedIds;
                PolyhedronFace3d repairedFace{};
                if (!BuildFaceWithRefitSupportPlane(body.FaceAt(faceIndex),
                                                    repairedFace,
                                                    eps,
                                                    preferredOuterVertices,
                                                    sourceOuterIds,
                                                    sourceHoleIds,
                                                    sourceRepresentativeIds != nullptr ? &repairedIds : nullptr))
                {
                    if (!BuildFaceWithRefitSupportPlane(body.FaceAt(faceIndex),
                                                        repairedFace,
                                                        eps,
                                                        emptyPreferredOuterVertices,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr))
                    {
                        return false;
                    }

                    if (sourceRepresentativeIds != nullptr && faceIndex < sourceRepresentativeIds->size())
                    {
                        repairedIds = (*sourceRepresentativeIds)[faceIndex];
                    }
                }

                repairedFaces.push_back(std::move(repairedFace));
                if (sourceRepresentativeIds != nullptr)
                {
                    repairedRepresentativeIds[faceIndex] = std::move(repairedIds);
                }
            }

            if (sourceRepresentativeIds == nullptr)
            {
                repairedRepresentativeIds.clear();
            }

            return true;
        }

        [[nodiscard]] bool ExecuteTopologyReconciliationPass(
            const std::vector<PolyhedronFace3d>& faces,
            const std::vector<FaceLoopRepresentativeIds>& representativeIds,
            NonPlanarRepairPassResult& repairResult,
            double eps)
        {
            PolyhedronBody repairedBody(faces);
            if (!repairedBody.IsValid(ComputeBodyValidationEpsilon(repairedBody, eps)))
            {
                return false;
            }

            repairResult.body = std::move(repairedBody);
            repairResult.representativeIds = representativeIds;
            repairResult.representativeTargetPoints.clear();
            if (!representativeIds.empty() &&
                !ExecuteRepresentativeTargetAggregationPass(
                    repairResult.body, repairResult.representativeIds, repairResult.representativeTargetPoints))
            {
                return false;
            }

            return true;
        }

        [[nodiscard]] bool TryRepairPolyhedronBodyForBrepConversion(
            const PolyhedronBody& body,
            const std::vector<FaceLoopRepresentativeIds>* sourceRepresentativeIds,
            NonPlanarRepairPassResult& repairResult,
            double eps)
        {
            if (body.IsEmpty())
            {
                return false;
            }

            std::vector<std::vector<bool>> facePreferredOuterVertices;
            BuildSharedOuterVertexPreferenceMask(body, facePreferredOuterVertices, eps);

            // Support-plane scoring pass: select a refit support plane per face
            // and normalize loop vertices / representative ids.
            std::vector<PolyhedronFace3d> repairedFaces;
            std::vector<FaceLoopRepresentativeIds> repairedRepresentativeIds;
            if (!ExecuteSupportPlaneScoringPass(body,
                                                facePreferredOuterVertices,
                                                sourceRepresentativeIds,
                                                repairedFaces,
                                                repairedRepresentativeIds,
                                                eps))
            {
                return false;
            }

            for (std::size_t faceIndex = 0; faceIndex < repairedFaces.size(); ++faceIndex)
            {
            }

            if (repairedRepresentativeIds.empty())
            {
                repairResult = {};
                repairResult.body = PolyhedronBody(std::move(repairedFaces));
                return repairResult.body.IsValid(eps);
            }

            // Establish a topology-reconciled baseline before iterative
            // representative aggregation / cross-face snapping.
            if (!ExecuteTopologyReconciliationPass(repairedFaces, repairedRepresentativeIds, repairResult, eps))
            {
                repairResult.body = PolyhedronBody(repairedFaces);
                repairResult.representativeIds = repairedRepresentativeIds;
            }

            std::vector<PolyhedronFace3d> currentFaces = repairedFaces;
            for (int iteration = 0; iteration < 2; ++iteration)
            {
                // Representative target aggregation pass.
                std::unordered_map<std::size_t, SCPoint3d> representativeTargetPoints;
                if (!ExecuteRepresentativeTargetAggregationPass(
                        currentFaces, repairedRepresentativeIds, representativeTargetPoints))
                {
                    break;
                }

                // Cross-face snapping pass.
                std::vector<PolyhedronFace3d> snappedFaces = currentFaces;
                if (!ExecuteCrossFaceSnappingPass(
                        repairedRepresentativeIds, representativeTargetPoints, snappedFaces, eps))
                {
                    break;
                }

                // Topology reconciliation pass.
                NonPlanarRepairPassResult snappedResult;
                if (!ExecuteTopologyReconciliationPass(snappedFaces, repairedRepresentativeIds, snappedResult, eps))
                {
                    break;
                }

                currentFaces = std::move(snappedFaces);
                repairResult = std::move(snappedResult);
            }

            return true;
        }
    }  // namespace

    BrepFaceConversion3d ConvertToPolyhedronFace(const SCBrepFace& face, double eps)
    {
        const SCGeometryTolerance3d tolerance{eps, eps, eps};
        if (!face.IsValid(tolerance))
        {
            return {false, BrepConversionIssue3d::InvalidFace, {}};
        }

        const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
        if (planeSurface == nullptr)
        {
            return {false, BrepConversionIssue3d::UnsupportedSurface, {}};
        }

        std::vector<SCPoint3d> outerVertices;
        if (!BuildLoopFromTrim(face.OuterTrim(), outerVertices, eps))
        {
            return {false, BrepConversionIssue3d::InvalidTrim, {}};
        }

        std::vector<PolyhedronLoop3d> holes;
        holes.reserve(face.HoleCount());
        for (const SCCurveOnSurface& trim : face.HoleTrims())
        {
            std::vector<SCPoint3d> holeVertices;
            if (!BuildLoopFromTrim(trim, holeVertices, eps))
            {
                return {false, BrepConversionIssue3d::InvalidTrim, {}};
            }

            holes.emplace_back(std::move(holeVertices));
        }

        PolyhedronFace3d polyFace(
            planeSurface->SupportPlane(), PolyhedronLoop3d(std::move(outerVertices)), std::move(holes));
        if (!polyFace.IsValid(eps))
        {
            return {false, BrepConversionIssue3d::InvalidTrim, {}};
        }

        return {true, BrepConversionIssue3d::None, std::move(polyFace)};
    }

    BrepBodyConversion3d ConvertToPolyhedronBody(const SCBrepBody& body, double eps)
    {
        const SCGeometryTolerance3d tolerance{eps, eps, eps};
        if (!body.IsValid(tolerance))
        {
            return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
        }

        std::vector<PolyhedronFace3d> faces;
        faces.reserve(body.FaceCount());
        std::size_t faceIndex = 0;
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell shell = body.ShellAt(shellIndex);
            for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
            {
                const SCBrepFace face = shell.FaceAt(localFaceIndex);
                BrepFaceConversion3d converted = ConvertToPolyhedronFace(face, eps);
                if (!converted.success)
                {
                    PolyhedronFace3d polyFace{};
                    if (!BuildPolyhedronFaceFromBodyFace(body, face, polyFace, eps))
                    {
                        return {false, converted.issue, faceIndex, {}};
                    }

                    converted = {true, BrepConversionIssue3d::None, std::move(polyFace)};
                }
                faces.push_back(converted.face);
            }
        }

        PolyhedronBody polyBody(std::move(faces));
        if (!polyBody.IsValid(eps))
        {
            return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
        }

        return {true, BrepConversionIssue3d::None, 0, std::move(polyBody)};
    }

    PolyhedronBrepBodyConversion3d ConvertToBrepBody(const PolyhedronBody& body, double eps)
    {
        try
        {
            auto hasCommonSupportNormal = [&](const PolyhedronBody& candidateBody) {
                if (candidateBody.FaceCount() == 0 || candidateBody.FaceCount() > 3)
                {
                    return false;
                }

                const SCPlane referencePlane = candidateBody.FaceAt(0).SupportPlane();
                const SCVector3d referenceNormal = referencePlane.UnitNormal(eps);
                if (!referenceNormal.IsValid())
                {
                    return false;
                }

                for (std::size_t faceIndex = 1; faceIndex < candidateBody.FaceCount(); ++faceIndex)
                {
                    const SCVector3d faceNormal = candidateBody.FaceAt(faceIndex).SupportPlane().UnitNormal(eps);
                    if (!faceNormal.IsValid() || (faceNormal - referenceNormal).Length() > eps)
                    {
                        return false;
                    }
                }

                return true;
            };

            std::vector<FaceLoopRepresentativeIds> sourceRepresentativeIds;
            BuildBodyLoopRepresentativeIds(body, sourceRepresentativeIds, eps);

            NonPlanarRepairPassResult repairResult;
            const PolyhedronBody originalBody = body;
            PolyhedronBody sourceBody = body;
            const bool useCommonSupportPlaneGrouping = hasCommonSupportNormal(sourceBody);
            const SCPlane commonSupportPlane = sourceBody.FaceCount() > 0 ? sourceBody.FaceAt(0).SupportPlane() : SCPlane{};
            const bool requiresRepair = !sourceBody.IsValid(eps);
            if (requiresRepair &&
                !TryRepairPolyhedronBodyForBrepConversion(body, &sourceRepresentativeIds, repairResult, eps))
            {
                return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
            }

            if (requiresRepair)
            {
                sourceBody = repairResult.body;
                if (sourceBody.FaceCount() == 3)
                {
                    for (std::size_t faceIndex = 0; faceIndex < sourceBody.FaceCount(); ++faceIndex)
                    {
                        const PolyhedronFace3d face = sourceBody.FaceAt(faceIndex);
                        const SCPlane plane = face.SupportPlane();
                    }
                }
            }

            auto computeBodyScale = [&sourceBody]() {
                double maxDistanceSquared = 0.0;
                for (std::size_t faceIndex = 0; faceIndex < sourceBody.FaceCount(); ++faceIndex)
                {
                    const PolyhedronFace3d face = sourceBody.FaceAt(faceIndex);
                    auto accumulateLoopScale = [&](const PolyhedronLoop3d& loop) {
                        for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                        {
                            for (std::size_t j = i + 1; j < loop.VertexCount(); ++j)
                            {
                                const double distanceSquared = (loop.VertexAt(i) - loop.VertexAt(j)).LengthSquared();
                                if (distanceSquared > maxDistanceSquared)
                                {
                                    maxDistanceSquared = distanceSquared;
                                }
                            }
                        }
                    };

                    accumulateLoopScale(face.OuterLoop());
                    for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                    {
                        accumulateLoopScale(face.HoleAt(holeIndex));
                    }
                }

                return std::sqrt(maxDistanceSquared);
            };

            const bool repairQuadHolelessBody = [&sourceBody, requiresRepair]() {
                if (!requiresRepair || sourceBody.FaceCount() == 0)
                {
                    return false;
                }

                for (std::size_t faceIndex = 0; faceIndex < sourceBody.FaceCount(); ++faceIndex)
                {
                    const PolyhedronFace3d face = sourceBody.FaceAt(faceIndex);
                    if (face.HoleCount() != 0 || face.OuterLoop().VertexCount() != 4)
                    {
                        return false;
                    }
                }

                return true;
            }();

            std::unordered_map<std::size_t, SCPoint3d> representativeTargetPoints;
            bool hasRepresentativeTargetPoints = ExecuteRepresentativeTargetAggregationPass(
                originalBody, sourceRepresentativeIds, representativeTargetPoints);
            if (requiresRepair && !repairResult.representativeIds.empty() &&
                repairResult.body.FaceCount() == originalBody.FaceCount())
            {
                std::vector<SCPoint3d> projectedClusterRepresentatives;
                std::vector<RepresentativePointAccumulator> projectedClusterAccumulators;
                std::unordered_map<std::size_t, std::size_t> repairIdToProjectedCluster;

                auto accumulateProjectedLoop = [&](const PolyhedronLoop3d& loop,
                                                   const std::vector<std::size_t>& loopIds,
                                                   const SCPlane& supportPlane) {
                    if (loop.VertexCount() != loopIds.size())
                    {
                        return false;
                    }

                    bool hasPrevious = false;
                    SCPoint3d previousPoint{};
                    std::size_t previousId = 0;
                    for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                    {
                        SCPoint3d point = loop.VertexAt(i);
                        const std::size_t representativeId = loopIds[i];
                        if (hasPrevious && representativeId == previousId && point.AlmostEquals(previousPoint, eps))
                        {
                            continue;
                        }

                        const SCVector3d unitNormal = supportPlane.UnitNormal(eps);
                        const double signedDistance = supportPlane.SignedDistanceTo(point, eps);
                        point = point - unitNormal * signedDistance;

                        const std::size_t clusterIndex = FindOrAddRepresentativePoint(
                            point, projectedClusterRepresentatives, Geometry::kRepresentativeGroupingEpsilon);
                        if (clusterIndex >= projectedClusterAccumulators.size())
                        {
                            projectedClusterAccumulators.resize(clusterIndex + 1);
                        }

                        auto& accumulator = projectedClusterAccumulators[clusterIndex];
                        accumulator.sum = accumulator.sum + (point - SCPoint3d{});
                        ++accumulator.count;
                        repairIdToProjectedCluster[representativeId] = clusterIndex;

                        hasPrevious = true;
                        previousPoint = point;
                        previousId = representativeId;
                    }
                    return true;
                };

                for (std::size_t faceIndex = 0; faceIndex < originalBody.FaceCount(); ++faceIndex)
                {
                    const PolyhedronFace3d originalFace = originalBody.FaceAt(faceIndex);
                    const PolyhedronFace3d repairedFace = repairResult.body.FaceAt(faceIndex);
                    const SCPlane supportPlane = repairedFace.SupportPlane();
                    const FaceLoopRepresentativeIds& ids = repairResult.representativeIds[faceIndex];

                    (void)accumulateProjectedLoop(originalFace.OuterLoop(), ids.outer, supportPlane);
                    for (std::size_t holeIndex = 0;
                         holeIndex < originalFace.HoleCount() && holeIndex < ids.holes.size();
                         ++holeIndex)
                    {
                        (void)accumulateProjectedLoop(
                            originalFace.HoleAt(holeIndex), ids.holes[holeIndex], supportPlane);
                    }
                }

                for (const auto& [representativeId, clusterIndex] : repairIdToProjectedCluster)
                {
                    if (clusterIndex >= projectedClusterAccumulators.size())
                    {
                        continue;
                    }

                    const RepresentativePointAccumulator& accumulator = projectedClusterAccumulators[clusterIndex];
                    if (accumulator.count == 0)
                    {
                        continue;
                    }

                    const SCVector3d average = accumulator.sum / static_cast<double>(accumulator.count);
                    representativeTargetPoints.try_emplace(representativeId, SCPoint3d{average.x, average.y, average.z});
                }
            }

            if (requiresRepair && !repairResult.representativeIds.empty() && !sourceRepresentativeIds.empty())
            {
                const std::size_t faceCount =
                    std::min(sourceRepresentativeIds.size(), repairResult.representativeIds.size());
                for (std::size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex)
                {
                    const auto& sourceFaceIds = sourceRepresentativeIds[faceIndex];
                    const auto& repairFaceIds = repairResult.representativeIds[faceIndex];

                    if (sourceFaceIds.outer.size() == repairFaceIds.outer.size())
                    {
                        for (std::size_t vertexIndex = 0; vertexIndex < sourceFaceIds.outer.size(); ++vertexIndex)
                        {
                            const std::size_t sourceId = sourceFaceIds.outer[vertexIndex];
                            const std::size_t repairId = repairFaceIds.outer[vertexIndex];
                            const auto sourceTargetIt = representativeTargetPoints.find(sourceId);
                            if (sourceTargetIt != representativeTargetPoints.end())
                            {
                                representativeTargetPoints.try_emplace(repairId, sourceTargetIt->second);
                            }
                        }
                    }

                    const std::size_t holeCount = std::min(sourceFaceIds.holes.size(), repairFaceIds.holes.size());
                    for (std::size_t holeIndex = 0; holeIndex < holeCount; ++holeIndex)
                    {
                        const auto& sourceHoleIds = sourceFaceIds.holes[holeIndex];
                        const auto& repairHoleIds = repairFaceIds.holes[holeIndex];
                        if (sourceHoleIds.size() != repairHoleIds.size())
                        {
                            continue;
                        }

                        for (std::size_t vertexIndex = 0; vertexIndex < sourceHoleIds.size(); ++vertexIndex)
                        {
                            const std::size_t sourceId = sourceHoleIds[vertexIndex];
                            const std::size_t repairId = repairHoleIds[vertexIndex];
                            const auto sourceTargetIt = representativeTargetPoints.find(sourceId);
                            if (sourceTargetIt != representativeTargetPoints.end())
                            {
                                representativeTargetPoints.try_emplace(repairId, sourceTargetIt->second);
                            }
                        }
                    }
                }
            }

            if (requiresRepair && !repairResult.representativeTargetPoints.empty())
            {
                for (const auto& [representativeId, targetPoint] : repairResult.representativeTargetPoints)
                {
                    representativeTargetPoints.try_emplace(representativeId, targetPoint);
                }
            }

            if (useCommonSupportPlaneGrouping && commonSupportPlane.IsValid(eps))
            {
                const SCVector3d unitNormal = commonSupportPlane.UnitNormal(eps);
                for (auto& [representativeId, targetPoint] : representativeTargetPoints)
                {
                    const double signedDistance = commonSupportPlane.SignedDistanceTo(targetPoint, eps);
                    targetPoint = targetPoint - unitNormal * signedDistance;
                }
            }

            const double vertexDedupEps = repairQuadHolelessBody
                                              ? std::max(Geometry::kBrepVertexDedupEpsilon, computeBodyScale() * 0.1)
                                              : Geometry::kBrepVertexDedupEpsilon;
            const double validationEps = ComputeBodyValidationEpsilon(sourceBody, eps);

            std::vector<SCBrepVertex> vertices;
            std::vector<SCBrepEdge> edges;
            std::vector<SCBrepFace> faces;
            std::unordered_map<std::size_t, std::size_t> representativeToVertexIndex;
            faces.reserve(sourceBody.FaceCount());

            auto buildLoopFromRepresentativeTargets = [&representativeTargetPoints](
                                                          const PolyhedronLoop3d& loop,
                                                          const std::vector<std::size_t>* loopRepresentativeIds) {
                if (loopRepresentativeIds == nullptr || loopRepresentativeIds->size() != loop.VertexCount())
                {
                    return loop;
                }

                std::vector<SCPoint3d> points;
                points.reserve(loop.VertexCount());
                for (std::size_t i = 0; i < loop.VertexCount(); ++i)
                {
                    const auto targetIt = representativeTargetPoints.find((*loopRepresentativeIds)[i]);
                    points.push_back(targetIt != representativeTargetPoints.end() ? targetIt->second
                                                                                  : loop.VertexAt(i));
                }

                return PolyhedronLoop3d(std::move(points));
            };

            for (std::size_t faceIndex = 0; faceIndex < sourceBody.FaceCount(); ++faceIndex)
            {
                const PolyhedronFace3d sourceFace = sourceBody.FaceAt(faceIndex);
                if (!sourceFace.IsValid(validationEps))
                {
                    return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                }

                const std::vector<std::size_t>* outerRepresentativeIds = nullptr;
                if (faceIndex < repairResult.representativeIds.size())
                {
                    const auto& candidateOuterIds = repairResult.representativeIds[faceIndex].outer;
                    if (candidateOuterIds.size() == sourceFace.OuterLoop().VertexCount())
                    {
                        outerRepresentativeIds = &candidateOuterIds;
                    }
                }
                if (outerRepresentativeIds == nullptr && faceIndex < sourceRepresentativeIds.size())
                {
                    const auto& candidateOuterIds = sourceRepresentativeIds[faceIndex].outer;
                    if (candidateOuterIds.size() == sourceFace.OuterLoop().VertexCount())
                    {
                        outerRepresentativeIds = &candidateOuterIds;
                    }
                }
                std::vector<const std::vector<std::size_t>*> holeRepresentativeIds;
                holeRepresentativeIds.resize(sourceFace.HoleCount(), nullptr);
                if (faceIndex < repairResult.representativeIds.size())
                {
                    for (std::size_t holeIndex = 0; holeIndex < sourceFace.HoleCount(); ++holeIndex)
                    {
                        if (holeRepresentativeIds[holeIndex] != nullptr)
                        {
                            continue;
                        }

                        if (holeIndex < repairResult.representativeIds[faceIndex].holes.size())
                        {
                            const auto& candidateHoleIds = repairResult.representativeIds[faceIndex].holes[holeIndex];
                            if (candidateHoleIds.size() == sourceFace.HoleAt(holeIndex).VertexCount())
                            {
                                holeRepresentativeIds[holeIndex] = &candidateHoleIds;
                            }
                        }
                    }
                }
                if (faceIndex < sourceRepresentativeIds.size())
                {
                    for (std::size_t holeIndex = 0; holeIndex < sourceFace.HoleCount(); ++holeIndex)
                    {
                        if (holeRepresentativeIds[holeIndex] != nullptr)
                        {
                            continue;
                        }

                        if (holeIndex < sourceRepresentativeIds[faceIndex].holes.size())
                        {
                            const auto& candidateHoleIds = sourceRepresentativeIds[faceIndex].holes[holeIndex];
                            if (candidateHoleIds.size() == sourceFace.HoleAt(holeIndex).VertexCount())
                            {
                                holeRepresentativeIds[holeIndex] = &candidateHoleIds;
                            }
                        }
                    }
                }

                PolyhedronFace3d face = sourceFace;

                const SCPlaneSurface planeSurface = SCPlaneSurface::FromPlane(
                    face.SupportPlane(), SCIntervald{-1.0, 1.0}, SCIntervald{-1.0, 1.0}, validationEps);
                auto supportSurface = std::make_shared<SCPlaneSurface>(planeSurface);
                const SCPlane supportPlane = planeSurface.SupportPlane();

                SCBrepLoop outerLoop;
                std::vector<SCPoint2d> outerUv;
                if (!AppendSharedBrepLoopFromPolyLoop(
                        face.OuterLoop(),
                        vertices,
                        edges,
                        outerLoop,
                        outerUv,
                        outerRepresentativeIds,
                        hasRepresentativeTargetPoints ? &representativeTargetPoints : nullptr,
                        hasRepresentativeTargetPoints ? &supportPlane : nullptr,
                        useCommonSupportPlaneGrouping,
                        &representativeToVertexIndex,
                        vertexDedupEps,
                        eps))
                {
                    return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                }

                try
                {
                    std::vector<SCPoint3d> outerLoopVertices;
                    if (!AppendLoopVerticesFromBrepTopology(vertices, edges, outerLoop, outerLoopVertices))
                    {
                        return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                    }

                    for (const SCPoint3d& point : outerLoopVertices)
                    {
                        outerUv.push_back(ProjectPointToPlaneUv(point, planeSurface));
                    }
                } catch (const std::exception&)
                {
                    return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                }

                if (outerUv.size() != face.OuterLoop().VertexCount())
                {
                    return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                }

                SCCurveOnSurface outerTrim;
                try
                {
                    outerTrim = SCCurveOnSurface(supportSurface, SCPolyline2d(std::move(outerUv), SCPolylineClosure::Closed));
                    if (!outerTrim.IsValid(SCGeometryTolerance3d{validationEps, validationEps, validationEps}))
                    {
                        outerTrim = {};
                    }
                } catch (const std::exception&)
                {
                    outerTrim = {};
                }

                std::vector<SCBrepLoop> holeLoops;
                std::vector<SCCurveOnSurface> holeTrims;
                holeLoops.reserve(face.HoleCount());
                holeTrims.reserve(face.HoleCount());
                for (std::size_t holeIndex = 0; holeIndex < face.HoleCount(); ++holeIndex)
                {
                    try
                    {
                        const PolyhedronLoop3d hole = face.HoleAt(holeIndex);

                        const std::vector<std::size_t>* holeRepresentativeIdsForFace = holeRepresentativeIds[holeIndex];

                        SCBrepLoop holeLoop;
                        std::vector<SCPoint2d> holeUv;
                        if (!AppendSharedBrepLoopFromPolyLoop(
                                hole,
                                vertices,
                                edges,
                                holeLoop,
                                holeUv,
                                holeRepresentativeIdsForFace,
                                hasRepresentativeTargetPoints ? &representativeTargetPoints : nullptr,
                                hasRepresentativeTargetPoints ? &supportPlane : nullptr,
                                useCommonSupportPlaneGrouping,
                                &representativeToVertexIndex,
                                vertexDedupEps,
                                eps))
                        {
                            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                        }

                        try
                        {
                            std::vector<SCPoint3d> holeLoopVertices;
                            if (!AppendLoopVerticesFromBrepTopology(vertices, edges, holeLoop, holeLoopVertices))
                            {
                                return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                            }

                            for (const SCPoint3d& point : holeLoopVertices)
                            {
                        holeUv.push_back(ProjectPointToPlaneUv(point, planeSurface));
                            }
                        } catch (const std::exception&)
                        {
                            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                        }

                        if (holeUv.size() != hole.VertexCount())
                        {
                            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                        }

                        holeLoops.push_back(std::move(holeLoop));
                        SCCurveOnSurface holeTrim(supportSurface, SCPolyline2d(std::move(holeUv), SCPolylineClosure::Closed));
                        if (!holeTrim.IsValid(SCGeometryTolerance3d{validationEps, validationEps, validationEps}))
                        {
                            holeTrim = {};
                        }
                        holeTrims.emplace_back(std::move(holeTrim));
                    } catch (const std::exception&)
                    {
                        holeTrims.emplace_back();
                    }
                }

                try
                {
                    SCBrepFace brepFace(
                    supportSurface, outerLoop, std::move(holeLoops), std::move(outerTrim), std::move(holeTrims));
                    if (!brepFace.IsValid(SCGeometryTolerance3d{validationEps, validationEps, validationEps}))
                    {
                        if (!requiresRepair)
                        {
                            return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                        }
                    }

                    faces.push_back(std::move(brepFace));
                } catch (const std::exception&)
                {
                    return {false, BrepConversionIssue3d::InvalidFace, faceIndex, {}};
                }
            }

            try
            {
                auto hasClosedSharedTopologySignature = [&faces, &vertices, &edges]() {
                    if (faces.size() != 6 || edges.size() != 12)
                    {
                        return false;
                    }

                    std::map<std::pair<std::size_t, std::size_t>, std::size_t> edgeUseCounts;
                    for (const SCBrepFace& face : faces)
                    {
                        if (face.HoleCount() != 0)
                        {
                            return false;
                        }

                        const SCBrepLoop outerLoop = face.OuterLoop();
                        if (outerLoop.CoedgeCount() != 4)
                        {
                            return false;
                        }

                        for (std::size_t i = 0; i < outerLoop.CoedgeCount(); ++i)
                        {
                            const SCBrepCoedge coedge = outerLoop.CoedgeAt(i);
                            if (coedge.EdgeIndex() >= edges.size())
                            {
                                return false;
                            }

                            const SCBrepEdge& edge = edges[coedge.EdgeIndex()];
                            const std::size_t startVertexIndex =
                                coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
                            const std::size_t endVertexIndex =
                                coedge.Reversed() ? edge.StartVertexIndex() : edge.EndVertexIndex();
                            ++edgeUseCounts[std::minmax(startVertexIndex, endVertexIndex)];
                        }
                    }

                    for (const auto& [edgeKey, count] : edgeUseCounts)
                    {
                        (void)edgeKey;
                        if (count != 2)
                        {
                            return false;
                        }
                    }

                    return true;
                };

                bool shellClosed =
                    ComputeSharedShellClosed(sourceBody, sourceRepresentativeIds) || hasClosedSharedTopologySignature();
                if (!shellClosed && requiresRepair && !repairResult.representativeIds.empty())
                {
                    shellClosed = ComputeSharedShellClosed(sourceBody, repairResult.representativeIds);
                }
                SCBrepBody brepBody(std::move(vertices), std::move(edges), {SCBrepShell(std::move(faces), shellClosed)});
                if (!brepBody.IsValid(SCGeometryTolerance3d{validationEps, validationEps, validationEps}))
                {
                    return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
                }

                return {true, BrepConversionIssue3d::None, 0, std::move(brepBody)};
            } catch (const std::exception&)
            {
                return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
            }
        } catch (const std::exception&)
        {
            return {false, BrepConversionIssue3d::InvalidBody, 0, {}};
        }
    }
}  // namespace Geometry


