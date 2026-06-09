#include "Brep/BodyBoolean.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "Brep/BrepConversion.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] double ResolveTolerance(const BodyBooleanOptions3d& options)
        {
            if (options.tolerance.distanceEpsilon > 0.0)
            {
                return options.tolerance.distanceEpsilon;
            }
            return Geometry::kBodyBooleanDefaultEpsilon;
        }

        [[nodiscard]] BodyBooleanResult3d MakeInvalidInputResult()
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::InvalidInput;
            result.message = "Body boolean input must contain at least one valid face.";
            return result;
        }

        [[nodiscard]] BodyBooleanResult3d MakeUnsupportedResult()
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::UnsupportedOperation;
            result.message =
                "3D body boolean currently supports only deterministic "
                "identical/disjoint closed-body "
                "subsets "
                "plus axis-aligned contained/overlap single-box and "
                "representative touching subsets.";
            return result;
        }

        [[nodiscard]] SCBrepBody EnsureClosedSingleShell(SCBrepBody body)
        {
            if (body.ShellCount() != 1)
            {
                return body;
            }

            const SCBrepShell shell = body.ShellAt(0);
            if (shell.IsClosed())
            {
                return body;
            }

            return SCBrepBody(body.Vertices(), body.Edges(), {SCBrepShell(shell.Faces(), true)});
        }

        [[nodiscard]] bool BoundsLexicographicallyLess(const SCBox3d& first, const SCBox3d& second, double epsilon);

        [[nodiscard]] BodyBooleanResult3d MakeSingleBodyResult(SCBrepBody body, const char* message)
        {
            BodyBooleanResult3d result;
            result.issue = BodyBooleanIssue3d::None;
            result.body = EnsureClosedSingleShell(std::move(body));
            result.message = message;
            return result;
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

        [[nodiscard]] bool BodiesLookIdentical(const SCBrepBody& first, const SCBrepBody& second, double epsilon)
        {
            return first.FaceCount() == second.FaceCount() && first.EdgeCount() == second.EdgeCount() &&
                   first.VertexCount() == second.VertexCount() && first.ShellCount() == second.ShellCount() &&
                   BoundsEqual(first.Bounds(), second.Bounds(), epsilon);
        }

        [[nodiscard]] BodyBooleanResult3d IntersectClosedSubset(const SCBrepBody& first,
                                                                const SCBrepBody& second,
                                                                const BodyBooleanOptions3d& options)
        {
            const double epsilon = ResolveTolerance(options);
            if (BodiesLookIdentical(first, second, epsilon))
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

            return MakeUnsupportedResult();
        }

        [[nodiscard]] BodyBooleanResult3d UnionClosedSubset(const SCBrepBody& first,
                                                            const SCBrepBody& second,
                                                            const BodyBooleanOptions3d& options)
        {
            const double epsilon = ResolveTolerance(options);
            if (BodiesLookIdentical(first, second, epsilon))
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
            if (BodiesLookIdentical(first, second, epsilon))
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
            if (!HasFaces(first) || !HasFaces(second))
            {
                return MakeInvalidInputResult();
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
            if (!HasFaces(first) || !HasFaces(second))
            {
                return MakeInvalidInputResult();
            }

            const PolyhedronBrepBodyConversion3d firstConversion = ConvertToBrepBody(first);
            const PolyhedronBrepBodyConversion3d secondConversion = ConvertToBrepBody(second);
            if (!firstConversion.success || !secondConversion.success)
            {
                return MakeUnsupportedResult();
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

