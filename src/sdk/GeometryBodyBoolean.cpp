#include "sdk/GeometryBodyBoolean.h"

#include <cmath>
#include <utility>

#include "sdk/GeometryBrepConversion.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double ResolveTolerance(const BodyBooleanOptions3d& options)
{
    if (options.tolerance.distanceEpsilon > 0.0)
    {
        return options.tolerance.distanceEpsilon;
    }
    return geometry::kDefaultEpsilon;
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
        "3D body boolean currently supports only deterministic identical/disjoint closed-body subsets.";
    return result;
}

[[nodiscard]] BodyBooleanResult3d MakeSingleBodyResult(BrepBody body, const char* message)
{
    BodyBooleanResult3d result;
    result.issue = BodyBooleanIssue3d::None;
    result.body = std::move(body);
    result.message = message;
    return result;
}

[[nodiscard]] BodyBooleanResult3d MakeMultiBodyResult(std::vector<BrepBody> bodies, const char* message)
{
    BodyBooleanResult3d result;
    result.issue = BodyBooleanIssue3d::None;
    result.bodies = std::move(bodies);
    result.message = message;
    return result;
}

[[nodiscard]] bool HasFaces(const BrepBody& body)
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

[[nodiscard]] bool BoundsEqual(const Box3d& left, const Box3d& right, double epsilon)
{
    return left.IsValid() && right.IsValid() &&
           NearlyEqual(left.MinPoint().x, right.MinPoint().x, epsilon) &&
           NearlyEqual(left.MinPoint().y, right.MinPoint().y, epsilon) &&
           NearlyEqual(left.MinPoint().z, right.MinPoint().z, epsilon) &&
           NearlyEqual(left.MaxPoint().x, right.MaxPoint().x, epsilon) &&
           NearlyEqual(left.MaxPoint().y, right.MaxPoint().y, epsilon) &&
           NearlyEqual(left.MaxPoint().z, right.MaxPoint().z, epsilon);
}

[[nodiscard]] bool BoundsDisjoint(const Box3d& left, const Box3d& right, double epsilon)
{
    return left.IsValid() && right.IsValid() &&
           (left.MaxPoint().x < right.MinPoint().x - epsilon || right.MaxPoint().x < left.MinPoint().x - epsilon ||
            left.MaxPoint().y < right.MinPoint().y - epsilon || right.MaxPoint().y < left.MinPoint().y - epsilon ||
            left.MaxPoint().z < right.MinPoint().z - epsilon || right.MaxPoint().z < left.MinPoint().z - epsilon);
}

[[nodiscard]] bool BodiesLookIdentical(const BrepBody& first, const BrepBody& second, double epsilon)
{
    return first.FaceCount() == second.FaceCount() &&
           first.EdgeCount() == second.EdgeCount() &&
           first.VertexCount() == second.VertexCount() &&
           first.ShellCount() == second.ShellCount() &&
           BoundsEqual(first.Bounds(), second.Bounds(), epsilon);
}

[[nodiscard]] BodyBooleanResult3d IntersectClosedSubset(
    const BrepBody& first,
    const BrepBody& second,
    const BodyBooleanOptions3d& options)
{
    const double epsilon = ResolveTolerance(options);
    if (BodiesLookIdentical(first, second, epsilon))
    {
        return MakeSingleBodyResult(first, "Deterministic identical-body intersection subset.");
    }

    return MakeUnsupportedResult();
}

[[nodiscard]] BodyBooleanResult3d UnionClosedSubset(
    const BrepBody& first,
    const BrepBody& second,
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

    return MakeUnsupportedResult();
}

[[nodiscard]] BodyBooleanResult3d DifferenceClosedSubset(
    const BrepBody& first,
    const BrepBody& second,
    const BodyBooleanOptions3d& options)
{
    const double epsilon = ResolveTolerance(options);
    if (BoundsDisjoint(first.Bounds(), second.Bounds(), epsilon))
    {
        return MakeSingleBodyResult(first, "Deterministic disjoint-body difference subset.");
    }

    return MakeUnsupportedResult();
}

[[nodiscard]] BodyBooleanResult3d MakeResultForBrepBodies(
    const BrepBody& first,
    const BrepBody& second,
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

[[nodiscard]] BodyBooleanResult3d MakeResultForPolyhedronBodies(
    const PolyhedronBody& first,
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
} // namespace

BodyBooleanResult3d IntersectBodies(const BrepBody& first, const BrepBody& second, BodyBooleanOptions3d options)
{
    return MakeResultForBrepBodies(first, second, options, 'i');
}

BodyBooleanResult3d UnionBodies(const BrepBody& first, const BrepBody& second, BodyBooleanOptions3d options)
{
    return MakeResultForBrepBodies(first, second, options, 'u');
}

BodyBooleanResult3d DifferenceBodies(const BrepBody& first, const BrepBody& second, BodyBooleanOptions3d options)
{
    return MakeResultForBrepBodies(first, second, options, 'd');
}

BodyBooleanResult3d IntersectBodies(const PolyhedronBody& first, const PolyhedronBody& second, BodyBooleanOptions3d options)
{
    return MakeResultForPolyhedronBodies(first, second, options, 'i');
}

BodyBooleanResult3d UnionBodies(const PolyhedronBody& first, const PolyhedronBody& second, BodyBooleanOptions3d options)
{
    return MakeResultForPolyhedronBodies(first, second, options, 'u');
}

BodyBooleanResult3d DifferenceBodies(
    const PolyhedronBody& first,
    const PolyhedronBody& second,
    BodyBooleanOptions3d options)
{
    return MakeResultForPolyhedronBodies(first, second, options, 'd');
}
} // namespace geometry::sdk
