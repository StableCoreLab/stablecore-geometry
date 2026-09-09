#include "SegmentBoxRelation2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <limits>
#include <numbers>
#include <vector>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "SegmentKernel2d.h"

namespace Geometry::Detail
{
    namespace
    {
        // 局部数值稳健阈值：仅用于求交解算、参数排序/去重与边界点判定的浮点误差控制，
        // 不扩大或缩小 B_eps。局部坐标量级归一化至 O(1)，故取默认 epsilon 量级。
        constexpr double kLocalNumericalTol = 1e-9;

        [[nodiscard]] bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        [[nodiscard]] double Dot2(const SCVector2d& first, const SCVector2d& second)
        {
            return first.x * second.x + first.y * second.y;
        }

        [[nodiscard]] double Cross2(const SCVector2d& first, const SCVector2d& second)
        {
            return first.x * second.y - first.y * second.x;
        }

        [[nodiscard]] double DeterminantErrorBound(const SCVector2d& first, const SCVector2d& second)
        {
            constexpr double kRoundoff = std::numeric_limits<double>::epsilon();
            const double terms = std::abs(first.x * second.y) + std::abs(first.y * second.x);
            return 8.0 * kRoundoff * terms;
        }

        [[nodiscard]] bool BuildUnitDirection(const SCVector2d& direction,
                                              SCVector2d& unit,
                                              double& length)
        {
            const double maxComponent = std::max(std::abs(direction.x), std::abs(direction.y));
            if (!IsFinite(maxComponent) || maxComponent <= 0.0)
            {
                return false;
            }
            const double scaledX = direction.x / maxComponent;
            const double scaledY = direction.y / maxComponent;
            const double scaledLength = std::sqrt(scaledX * scaledX + scaledY * scaledY);
            if (!IsFinite(scaledLength) || scaledLength <= 0.0)
            {
                return false;
            }
            length = maxComponent * scaledLength;
            unit = SCVector2d{scaledX / scaledLength, scaledY / scaledLength};
            return IsFinite(length) && length > 0.0 && unit.IsValid();
        }

        [[nodiscard]] bool IsUsableLocalSegment(const ISCSegment2d& segment)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                const SCVector2d direction = line->endPoint - line->startPoint;
                return line->startPoint.IsValid() && line->endPoint.IsValid() && direction.IsValid() &&
                       !(direction.x == 0.0 && direction.y == 0.0);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                return arc->center.IsValid() && IsFinite(arc->radius) && arc->radius > 0.0 &&
                       IsFinite(arc->startAngle) && IsFinite(arc->sweepAngle) &&
                       std::abs(arc->sweepAngle) > 0.0 &&
                       std::abs(arc->sweepAngle) <= 2.0 * std::numbers::pi_v<double> + kLocalNumericalTol;
            }
            return false;
        }

        enum class CheckedIntersectionStatus
        {
            NoIntersection,
            Success,
            InvalidInput,
            NumericalIndeterminate
        };

        struct CheckedLocalIntersection
        {
            CheckedIntersectionStatus status{CheckedIntersectionStatus::NumericalIndeterminate};
            SCSegmentIntersection2d intersection{};
        };

        [[nodiscard]] SCPoint2d EvaluateLocalSegment(const ISCSegment2d& segment, double parameter)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                return SCPoint2d{line->startPoint.x + (line->endPoint.x - line->startPoint.x) * parameter,
                                 line->startPoint.y + (line->endPoint.y - line->startPoint.y) * parameter};
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                const double angle = arc->startAngle + arc->sweepAngle * parameter;
                return SCPoint2d{arc->center.x + arc->radius * std::cos(angle),
                                 arc->center.y + arc->radius * std::sin(angle)};
            }
            return {};
        }

        // 盒边可能比默认 epsilon 短，因此不能复用公开 Intersect(line, line)
        // 的段有效性门槛。该实现只服务于已归一化的局部 Line-Line 求交。
        [[nodiscard]] SCSegmentIntersection2d IntersectLocalLines(const SCLineSegment2d& first,
                                                                    const SCLineSegment2d& second,
                                                                    double tolerance)
        {
            const SCVector2d r = first.endPoint - first.startPoint;
            const SCVector2d s = second.endPoint - second.startPoint;
            const SCVector2d qMinusP = second.startPoint - first.startPoint;
            const double determinant = Cross2(r, s);
            const double collinearity = Cross2(qMinusP, r);
            if (!IsFinite(determinant) || !IsFinite(collinearity))
            {
                return {};
            }
            if (determinant == 0.0)
            {
                if (collinearity != 0.0)
                {
                    return {};
                }
                const double rr = Dot2(r, r);
                const double ss = Dot2(s, s);
                if (!IsFinite(rr) || !IsFinite(ss) || rr <= 0.0 || ss <= 0.0)
                {
                    return {};
                }
                double firstOnFirst = Dot2(qMinusP, r) / rr;
                double secondOnFirst = Dot2(second.endPoint - first.startPoint, r) / rr;
                if (!IsFinite(firstOnFirst) || !IsFinite(secondOnFirst))
                {
                    return {};
                }
                if (firstOnFirst > secondOnFirst)
                {
                    std::swap(firstOnFirst, secondOnFirst);
                }
                const double start = std::max(0.0, firstOnFirst);
                const double end = std::min(1.0, secondOnFirst);
                if (end < start)
                {
                    return {};
                }
                const SCPoint2d startPoint = EvaluateLocalSegment(first, start);
                const SCPoint2d endPoint = EvaluateLocalSegment(first, end);
                const double startOnSecond = Dot2(startPoint - second.startPoint, s) / ss;
                const double endOnSecond = Dot2(endPoint - second.startPoint, s) / ss;
                if (!startPoint.IsValid() || !endPoint.IsValid() || !IsFinite(startOnSecond) || !IsFinite(endOnSecond))
                {
                    return {};
                }
                if (end == start)
                {
                    SCSegmentIntersection2d result;
                    result.kind = SCIntersectionKind2d::Point;
                    result.points[0] = SCIntersectionPoint2d{startPoint, start, startOnSecond};
                    result.pointCount = 1;
                    return result;
                }
                SCSegmentIntersection2d result;
                result.kind = SCIntersectionKind2d::Overlap;
                result.points[0] = SCIntersectionPoint2d{startPoint, start, startOnSecond};
                result.points[1] = SCIntersectionPoint2d{endPoint, end, endOnSecond};
                result.pointCount = 2;
                return result;
            }
            const double firstParameter = Cross2(qMinusP, s) / determinant;
            const double secondParameter = Cross2(qMinusP, r) / determinant;
            if (!IsFinite(firstParameter) || !IsFinite(secondParameter) ||
                firstParameter < 0.0 || firstParameter > 1.0 ||
                secondParameter < 0.0 || secondParameter > 1.0)
            {
                return {};
            }
            const SCPoint2d point = EvaluateLocalSegment(first, firstParameter);
            if (!point.IsValid())
            {
                return {};
            }
            SCSegmentIntersection2d result;
            result.kind = SCIntersectionKind2d::Point;
            result.points[0] = SCIntersectionPoint2d{point, firstParameter, secondParameter};
            result.pointCount = 1;
            return result;
        }

        [[nodiscard]] double NormalizeAngle(double angle)
        {
            constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
            angle = std::fmod(angle, kTwoPi);
            return angle < 0.0 ? angle + kTwoPi : angle;
        }

        [[nodiscard]] bool IsAngleOnLocalArc(const SCArcSegment2d& arc, double angle, double tolerance)
        {
            if (arc.sweepAngle >= 0.0)
            {
                return NormalizeAngle(angle - arc.startAngle) <= arc.sweepAngle + tolerance;
            }
            return NormalizeAngle(arc.startAngle - angle) <= -arc.sweepAngle + tolerance;
        }

        // 与 IntersectLocalLines 同样，这里不经过公开 Intersect：局部盒边可短于
        // 默认 epsilon。几何范围采用精确闭区间，避免将局部容差变成 B_eps 的二次扩张。
        [[nodiscard]] SCSegmentIntersection2d IntersectLocalArcLine(const SCArcSegment2d& arc,
                                                                      const SCLineSegment2d& line)
        {
            const SCVector2d direction = line.endPoint - line.startPoint;
            const SCVector2d fromCenter = line.startPoint - arc.center;
            const double a = Dot2(direction, direction);
            const double b = 2.0 * Dot2(fromCenter, direction);
            const double c = Dot2(fromCenter, fromCenter) - arc.radius * arc.radius;
            const double discriminant = b * b - 4.0 * a * c;
            if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c) || !IsFinite(discriminant) || a <= 0.0 ||
                discriminant < 0.0)
            {
                return {};
            }
            const double rootOffset = std::sqrt(discriminant);
            if (!IsFinite(rootOffset))
            {
                return {};
            }
            const double roots[2] = {(-b - rootOffset) / (2.0 * a), (-b + rootOffset) / (2.0 * a)};
            SCSegmentIntersection2d result;
            for (const double lineParameter : roots)
            {
                if (!IsFinite(lineParameter) || lineParameter < 0.0 || lineParameter > 1.0)
                {
                    continue;
                }
                const SCPoint2d point = EvaluateLocalSegment(line, lineParameter);
                const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
                if (!point.IsValid() || !IsFinite(angle) || !IsAngleOnLocalArc(arc, angle, 0.0))
                {
                    continue;
                }
                const double swept = arc.sweepAngle >= 0.0 ? NormalizeAngle(angle - arc.startAngle)
                                                            : NormalizeAngle(arc.startAngle - angle);
                const double arcParameter = swept / std::abs(arc.sweepAngle);
                if (!IsFinite(arcParameter) || arcParameter < 0.0 || arcParameter > 1.0)
                {
                    return {};
                }
                if (result.pointCount == 0)
                {
                    result.kind = discriminant == 0.0 ? SCIntersectionKind2d::Tangent : SCIntersectionKind2d::Point;
                    result.points[0] = SCIntersectionPoint2d{point, arcParameter, lineParameter};
                    result.pointCount = 1;
                }
                else if (!point.AlmostEquals(result.points[0].point, 0.0))
                {
                    result.kind = SCIntersectionKind2d::Point;
                    result.points[1] = SCIntersectionPoint2d{point, arcParameter, lineParameter};
                    result.pointCount = 2;
                }
            }
            return result;
        }

        // 仅当独立的、所有中间值均有限的解析判定能够证明没有公共点时才返回 true。
        // 返回 false 表示存在交点或数值无法可靠排除交点，调用方必须将默认空结果视为失败。
        [[nodiscard]] bool CanProveNoLocalIntersection(const ISCSegment2d& first,
                                                        const SCLineSegment2d& second,
                                                        double tolerance)
        {
            const SCVector2d secondDirection = second.endPoint - second.startPoint;
            if (!secondDirection.IsValid())
            {
                return false;
            }
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&first))
            {
                const SCVector2d firstDirection = line->endPoint - line->startPoint;
                const SCVector2d betweenStarts = second.startPoint - line->startPoint;
                SCVector2d firstUnit{};
                double firstLength = 0.0;
                if (!betweenStarts.IsValid() || !BuildUnitDirection(firstDirection, firstUnit, firstLength))
                {
                    return false;
                }
                const double determinant = Cross2(firstUnit, secondDirection);
                const double collinearity = Cross2(betweenStarts, firstUnit);
                if (!IsFinite(determinant) || !IsFinite(collinearity))
                {
                    return false;
                }
                // 这里是解析“是否平行/共线”分支，而不是几何扩张判定。用局部
                // 数值容差会把合法狭长盒的两条不同直线合并，导致无边界事件时
                // 错误失败；仅精确零值进入平行分支，其他情形继续验证参数范围。
                if (determinant == 0.0)
                {
                    if (collinearity != 0.0)
                    {
                        return true;
                    }
                    const double secondStartOnFirst = Dot2(betweenStarts, firstUnit);
                    const double secondEndOnFirst = secondStartOnFirst + Dot2(secondDirection, firstUnit);
                    if (!IsFinite(secondStartOnFirst) || !IsFinite(secondEndOnFirst))
                    {
                        return false;
                    }
                    return std::min(secondStartOnFirst, secondEndOnFirst) > firstLength + tolerance ||
                           std::max(secondStartOnFirst, secondEndOnFirst) < -tolerance;
                }
                const double firstDistance = Cross2(betweenStarts, secondDirection) / determinant;
                const double secondParameter = Cross2(betweenStarts, firstUnit) / determinant;
                if (!IsFinite(firstDistance) || !IsFinite(secondParameter))
                {
                    return false;
                }
                return firstDistance < -tolerance || firstDistance > firstLength + tolerance ||
                       secondParameter < -tolerance || secondParameter > 1.0 + tolerance;
            }

            const auto* arc = dynamic_cast<const SCArcSegment2d*>(&first);
            if (arc == nullptr)
            {
                return false;
            }
            const SCVector2d fromCenter = second.startPoint - arc->center;
            const double a = Dot2(secondDirection, secondDirection);
            const double b = 2.0 * Dot2(fromCenter, secondDirection);
            const double c = Dot2(fromCenter, fromCenter) - arc->radius * arc->radius;
            const double discriminant = b * b - 4.0 * a * c;
            if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c) || !IsFinite(discriminant) || a <= 0.0)
            {
                return false;
            }
            if (discriminant < -tolerance)
            {
                return true;
            }
            const double rootOffset = discriminant <= 0.0 ? 0.0 : std::sqrt(discriminant);
            if (!IsFinite(rootOffset))
            {
                return false;
            }
            const double roots[2] = {(-b - rootOffset) / (2.0 * a), (-b + rootOffset) / (2.0 * a)};
            for (const double root : roots)
            {
                if (!IsFinite(root) || root < -tolerance || root > 1.0 + tolerance)
                {
                    continue;
                }
                const SCPoint2d point = EvaluateLocalSegment(second, std::clamp(root, 0.0, 1.0));
                if (!point.IsValid())
                {
                    return false;
                }
                const double angle = std::atan2(point.y - arc->center.y, point.x - arc->center.x);
                if (!IsFinite(angle))
                {
                    return false;
                }
                if (IsAngleOnLocalArc(*arc, angle, tolerance))
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] CheckedLocalIntersection CheckedIntersectLocalSegments(const ISCSegment2d& first,
                                                                               const SCLineSegment2d& second,
                                                                               double tolerance)
        {
            CheckedLocalIntersection result;
            // 盒边在按最大边长归一化后可能短于默认 epsilon；不能调用
            // SCLineSegment2d::IsValid()，否则合法的狭长盒会被误判为失败。
            if (!IsUsableLocalSegment(first) || !IsUsableLocalSegment(second) || !IsFinite(tolerance) || tolerance <= 0.0)
            {
                result.status = CheckedIntersectionStatus::InvalidInput;
                return result;
            }

            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&first))
            {
                const SCVector2d firstDirection = line->endPoint - line->startPoint;
                const SCVector2d secondDirection = second.endPoint - second.startPoint;
                const double determinant = Cross2(firstDirection, secondDirection);
                const double errorBound = DeterminantErrorBound(firstDirection, secondDirection);
                if (!IsFinite(determinant) || !IsFinite(errorBound) ||
                    (determinant != 0.0 && std::abs(determinant) <= errorBound))
                {
                    result.status = CheckedIntersectionStatus::NumericalIndeterminate;
                    return result;
                }
                result.intersection = IntersectLocalLines(*line, second, tolerance);
            }
            else if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&first))
            {
                const SCVector2d direction = second.endPoint - second.startPoint;
                const SCVector2d fromCenter = second.startPoint - arc->center;
                const double a = Dot2(direction, direction);
                const double b = 2.0 * Dot2(fromCenter, direction);
                const double c = Dot2(fromCenter, fromCenter) - arc->radius * arc->radius;
                const double discriminant = b * b - 4.0 * a * c;
                const double errorBound = 16.0 * std::numeric_limits<double>::epsilon() *
                                           (std::abs(b * b) + std::abs(4.0 * a * c));
                if (!IsFinite(discriminant) || !IsFinite(errorBound) ||
                    (discriminant != 0.0 && std::abs(discriminant) <= errorBound))
                {
                    result.status = CheckedIntersectionStatus::NumericalIndeterminate;
                    return result;
                }
                result.intersection = IntersectLocalArcLine(*arc, second);
            }
            else
            {
                result.status = CheckedIntersectionStatus::InvalidInput;
                return result;
            }
            if (result.intersection.kind == SCIntersectionKind2d::None)
            {
                if (result.intersection.pointCount == 0 && CanProveNoLocalIntersection(first, second, tolerance))
                {
                    result.status = CheckedIntersectionStatus::NoIntersection;
                }
                return result;
            }
            if (result.intersection.pointCount == 0 || result.intersection.pointCount > result.intersection.points.size())
            {
                return result;
            }

            for (std::size_t i = 0; i < result.intersection.pointCount; ++i)
            {
                const SCIntersectionPoint2d& point = result.intersection.points[i];
                if (!point.IsValid() || !IsFinite(point.parameterOnFirst) || !IsFinite(point.parameterOnSecond) ||
                    point.parameterOnFirst < 0.0 || point.parameterOnFirst > 1.0 ||
                    point.parameterOnSecond < 0.0 || point.parameterOnSecond > 1.0)
                {
                    return result;
                }
                const SCPoint2d onFirst = EvaluateLocalSegment(first, std::clamp(point.parameterOnFirst, 0.0, 1.0));
                const SCPoint2d onSecond = EvaluateLocalSegment(second, std::clamp(point.parameterOnSecond, 0.0, 1.0));
                if (!onFirst.IsValid() || !onSecond.IsValid() ||
                    std::abs(onFirst.x - point.point.x) > kLocalNumericalTol ||
                    std::abs(onFirst.y - point.point.y) > kLocalNumericalTol ||
                    std::abs(onSecond.x - point.point.x) > kLocalNumericalTol ||
                    std::abs(onSecond.y - point.point.y) > kLocalNumericalTol)
                {
                    return result;
                }
            }
            result.status = CheckedIntersectionStatus::Success;
            return result;
        }

        [[nodiscard]] SCBoxSegmentClassification2d MakeSuccess(SCBoxSegmentRelation2d relation)
        {
            return {true, relation, SCBoxSegmentFailure2d::None};
        }

        [[nodiscard]] SCBoxSegmentClassification2d MakeFailure(SCBoxSegmentFailure2d failure)
        {
            return {false, SCBoxSegmentRelation2d::Unknown, failure};
        }

        struct LocalFrame
        {
            SCPoint2d center{};
            double scale{1.0};
            double maxEdge{0.0};
        };

        // 盒中心用溢出安全形式计算（先各自除以二再相加）。最大边长与缩放因子非有限或为零时失败。
        [[nodiscard]] bool BuildLocalFrame(const SCBox2d& box, LocalFrame& frame)
        {
            const SCPoint2d lo = box.MinPoint();
            const SCPoint2d hi = box.MaxPoint();
            const double halfLoX = lo.x * 0.5;
            const double halfHiX = hi.x * 0.5;
            const double halfLoY = lo.y * 0.5;
            const double halfHiY = hi.y * 0.5;
            const double cx = halfLoX + halfHiX;
            const double cy = halfLoY + halfHiY;
            const double width = hi.x - lo.x;
            const double height = hi.y - lo.y;
            const double maxEdge = std::max(width, height);
            if (!IsFinite(cx) || !IsFinite(cy) || !IsFinite(maxEdge) || maxEdge <= 0.0)
            {
                return false;
            }
            frame.center = SCPoint2d{cx, cy};
            frame.maxEdge = maxEdge;
            frame.scale = 1.0 / maxEdge;
            return true;
        }

        [[nodiscard]] SCPoint2d ToLocal(const SCPoint2d& point, const LocalFrame& frame)
        {
            return SCPoint2d{(point.x - frame.center.x) * frame.scale,
                             (point.y - frame.center.y) * frame.scale};
        }

        [[nodiscard]] SCBox2d ToLocalBox(const SCBox2d& box, const LocalFrame& frame)
        {
            return SCBox2d{ToLocal(box.MinPoint(), frame), ToLocal(box.MaxPoint(), frame)};
        }

        // 将段变换到局部坐标。段参数 t 保持与原段一致（均匀缩放 + 平移为仿射且保参）。
        // 失败返回 nullptr。
        [[nodiscard]] std::unique_ptr<ISCSegment2d> ToLocalSegment(const ISCSegment2d& segment,
                                                                    const LocalFrame& frame)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                const SCPoint2d start = ToLocal(line->startPoint, frame);
                const SCPoint2d end = ToLocal(line->endPoint, frame);
                if (!start.IsValid() || !end.IsValid())
                {
                    return nullptr;
                }
                return std::make_unique<SCLineSegment2d>(start, end);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                const SCPoint2d center = ToLocal(arc->center, frame);
                const double radius = arc->radius * frame.scale;
                if (!center.IsValid() || !IsFinite(radius) || radius <= 0.0)
                {
                    return nullptr;
                }
                return std::make_unique<SCArcSegment2d>(center, radius, arc->startAngle, arc->sweepAngle);
            }
            return nullptr;
        }

        enum class CellState
        {
            Interior,
            Boundary,
            Exterior
        };

        // 判定点相对局部盒的位置。B_eps 已在公共入口唯一构造完成；此处必须使用
        // 精确闭盒边界，局部数值阈值不得再参与位置比较，否则等价于二次扩张 B_eps。
        [[nodiscard]] CellState ClassifyPointLocal(const SCPoint2d& point, const SCBox2d& localBox, double tol)
        {
            const SCPoint2d lo = localBox.MinPoint();
            const SCPoint2d hi = localBox.MaxPoint();
            (void)tol;
            if (point.x < lo.x || point.x > hi.x || point.y < lo.y || point.y > hi.y)
            {
                return CellState::Exterior;
            }
            const bool onEdge = point.x == lo.x || point.x == hi.x || point.y == lo.y || point.y == hi.y;
            return onEdge ? CellState::Boundary : CellState::Interior;
        }

        [[nodiscard]] std::array<SCLineSegment2d, 4> BuildLocalBoxEdges(const SCBox2d& localBox)
        {
            const SCPoint2d lo = localBox.MinPoint();
            const SCPoint2d hi = localBox.MaxPoint();
            return {SCLineSegment2d{SCPoint2d{lo.x, lo.y}, SCPoint2d{hi.x, lo.y}},  // bottom
                    SCLineSegment2d{SCPoint2d{hi.x, lo.y}, SCPoint2d{hi.x, hi.y}},  // right
                    SCLineSegment2d{SCPoint2d{hi.x, hi.y}, SCPoint2d{lo.x, hi.y}},  // top
                    SCLineSegment2d{SCPoint2d{lo.x, hi.y}, SCPoint2d{lo.x, lo.y}}}; // left
        }

        [[nodiscard]] bool ParamInUnitRange(double t)
        {
            return IsFinite(t) && t >= 0.0 && t <= 1.0;
        }

        struct Event
        {
            double parameter{0.0};
            bool fromIntersection{false};
        };

        [[nodiscard]] bool EventsMonotonic(const std::vector<Event>& events)
        {
            for (std::size_t i = 1; i < events.size(); ++i)
            {
                if (!(events[i].parameter >= events[i - 1].parameter))
                {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool IntervalInsideAny(double a, double b,
                                             const std::vector<std::pair<double, double>>& ranges,
                                             double tol)
        {
            (void)tol;
            for (const auto& range : ranges)
            {
                if (a >= range.first && b <= range.second)
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] SCBoxSegmentClassification2d ClassifyLineOrArc(const ISCSegment2d& localSegment,
                                                                     const SCBox2d& localBox)
        {
            const std::array<SCLineSegment2d, 4> edges = BuildLocalBoxEdges(localBox);

            std::vector<Event> events;
            events.push_back({0.0, false});
            events.push_back({1.0, false});
            std::vector<std::pair<double, double>> boundaryRanges;

            for (const SCLineSegment2d& edge : edges)
            {
                // checked 具象求交：短线段不受默认 epsilon 拒绝，且任何非有限或
                // 不满足参数/方程约束的输出都会转为 IntersectionSolveFailure。
                const CheckedLocalIntersection checked =
                    CheckedIntersectLocalSegments(localSegment, edge, kLocalNumericalTol);
                if (checked.status == CheckedIntersectionStatus::NumericalIndeterminate)
                {
                    return MakeFailure(SCBoxSegmentFailure2d::IntersectionSolveFailure);
                }
                if (checked.status == CheckedIntersectionStatus::InvalidInput)
                {
                    return MakeFailure(SCBoxSegmentFailure2d::InvalidSegment);
                }
                if (checked.status == CheckedIntersectionStatus::NoIntersection)
                {
                    continue;
                }
                const SCSegmentIntersection2d& hit = checked.intersection;
                if (hit.kind == SCIntersectionKind2d::Overlap)
                {
                    if (hit.pointCount != 2 || !IsFinite(hit.points[0].parameterOnFirst) ||
                        !IsFinite(hit.points[1].parameterOnFirst))
                    {
                        return MakeFailure(SCBoxSegmentFailure2d::IntersectionSolveFailure);
                    }
                    double ta = hit.points[0].parameterOnFirst;
                    double tb = hit.points[1].parameterOnFirst;
                    if (!IsFinite(ta) || !IsFinite(tb))
                    {
                        return MakeFailure(SCBoxSegmentFailure2d::IntersectionSolveFailure);
                    }
                    if (ta > tb)
                    {
                        std::swap(ta, tb);
                    }
                    events.push_back({ta, true});
                    events.push_back({tb, true});
                    boundaryRanges.emplace_back(ta, tb);
                    continue;
                }
                // Point or Tangent：单点（或重合于端点）接触盒边。
                for (std::size_t i = 0; i < hit.pointCount; ++i)
                {
                    const double raw = hit.points[i].parameterOnFirst;
                    if (!IsFinite(raw))
                    {
                        return MakeFailure(SCBoxSegmentFailure2d::IntersectionSolveFailure);
                    }
                    if (!ParamInUnitRange(raw))
                    {
                        return MakeFailure(SCBoxSegmentFailure2d::IntersectionSolveFailure);
                    }
                    events.push_back({raw, true});
                }
            }

            std::sort(events.begin(), events.end(),
                      [](const Event& lhs, const Event& rhs) { return lhs.parameter < rhs.parameter; });

            // 去重：同一参数（数值阈值内）合并，任一来自交点则保留交点标记。
            std::vector<Event> merged;
            merged.reserve(events.size());
            for (const Event& event : events)
            {
                if (!merged.empty() && std::abs(merged.back().parameter - event.parameter) <= kLocalNumericalTol)
                {
                    merged.back().fromIntersection = merged.back().fromIntersection || event.fromIntersection;
                    continue;
                }
                merged.push_back(event);
            }
            if (!EventsMonotonic(merged))
            {
                return MakeFailure(SCBoxSegmentFailure2d::EventOrderingFailure);
            }

            bool hasInterior = false;
            bool hasExterior = false;
            bool hasBoundary = false;

            // 分区开区间中点：拓扑判定，非几何离散采样。
            for (std::size_t i = 0; i + 1 < merged.size(); ++i)
            {
                const double a = merged[i].parameter;
                const double b = merged[i + 1].parameter;
                if (b <= a)
                {
                    continue;
                }
                if (IntervalInsideAny(a, b, boundaryRanges, kLocalNumericalTol))
                {
                    hasBoundary = true;
                    continue;
                }
                const double mid = 0.5 * (a + b);
                const SCPoint2d sample = EvaluateLocalSegment(localSegment, mid);
                if (!sample.IsValid())
                {
                    return MakeFailure(SCBoxSegmentFailure2d::RepresentativePointFailure);
                }
                switch (ClassifyPointLocal(sample, localBox, kLocalNumericalTol))
                {
                    case CellState::Interior:
                        hasInterior = true;
                        break;
                    case CellState::Exterior:
                        hasExterior = true;
                        break;
                    case CellState::Boundary:
                        hasBoundary = true;
                        break;
                }
            }

            // 分区端点：交点派生参数按构造标记为边界；纯端点按位置判定。
            for (const Event& event : merged)
            {
                if (event.fromIntersection)
                {
                    hasBoundary = true;
                    continue;
                }
                const SCPoint2d sample = EvaluateLocalSegment(localSegment, event.parameter);
                if (!sample.IsValid())
                {
                    return MakeFailure(SCBoxSegmentFailure2d::RepresentativePointFailure);
                }
                switch (ClassifyPointLocal(sample, localBox, kLocalNumericalTol))
                {
                    case CellState::Interior:
                        hasInterior = true;
                        break;
                    case CellState::Exterior:
                        hasExterior = true;
                        break;
                    case CellState::Boundary:
                        hasBoundary = true;
                        break;
                }
            }

            if (hasExterior && hasInterior)
            {
                return MakeSuccess(SCBoxSegmentRelation2d::Crossing);
            }
            if (hasExterior && !hasInterior)
            {
                return hasBoundary ? MakeSuccess(SCBoxSegmentRelation2d::Touching)
                                   : MakeSuccess(SCBoxSegmentRelation2d::Disjoint);
            }
            if (!hasExterior && hasInterior)
            {
                return hasBoundary ? MakeSuccess(SCBoxSegmentRelation2d::ContainedTouching)
                                   : MakeSuccess(SCBoxSegmentRelation2d::Contained);
            }
            if (!hasExterior && !hasInterior && hasBoundary)
            {
                return MakeSuccess(SCBoxSegmentRelation2d::ContainedTouching);
            }
            // 无任何可判定状态不得伪装为 Disjoint；公共 bool 入口会将失败一致地
            // 映射为 false，避免把数值不确定性解释为可靠的不相交。
            return MakeFailure(SCBoxSegmentFailure2d::RepresentativePointFailure);
        }
    }  // namespace

    SCBoxSegmentClassification2d ClassifySegmentBox(const ISCSegment2d& segment, const SCBox2d& expandedBox)
    {
        if (!segment.IsValid() || !expandedBox.IsValid())
        {
            return MakeFailure(SCBoxSegmentFailure2d::InvalidSegment);
        }
        if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
        {
            const SCPoint2d minPoint = expandedBox.MinPoint();
            const SCPoint2d maxPoint = expandedBox.MaxPoint();
            const SCPoint2d samples[] = {
                line->startPoint,
                SCPoint2d{0.5 * (line->startPoint.x + line->endPoint.x),
                          0.5 * (line->startPoint.y + line->endPoint.y)},
                line->endPoint};
            bool allInside = true;
            for (const SCPoint2d& point : samples)
            {
                allInside = allInside && point.IsValid() && point.x >= minPoint.x && point.x <= maxPoint.x &&
                            point.y >= minPoint.y && point.y <= maxPoint.y;
            }
            if (allInside)
            {
                bool touching = false;
                for (const SCPoint2d& point : samples)
                {
                    touching = touching || point.x == minPoint.x || point.x == maxPoint.x ||
                               point.y == minPoint.y || point.y == maxPoint.y;
                }
                return MakeSuccess(touching ? SCBoxSegmentRelation2d::ContainedTouching
                                             : SCBoxSegmentRelation2d::Contained);
            }
        }
        // 无额外几何容差的盒-盒粗筛：段包围盒与 B_eps 不相交则必不相交。
        const SCBox2d segmentBounds = segment.Bounds();
        if (segmentBounds.IsValid())
        {
            const SCPoint2d sLo = segmentBounds.MinPoint();
            const SCPoint2d sHi = segmentBounds.MaxPoint();
            const SCPoint2d bLo = expandedBox.MinPoint();
            const SCPoint2d bHi = expandedBox.MaxPoint();
            const bool disjoint = sHi.x < bLo.x || bHi.x < sLo.x || sHi.y < bLo.y || bHi.y < sLo.y;
            if (disjoint)
            {
                return MakeSuccess(SCBoxSegmentRelation2d::Disjoint);
            }
        }

        LocalFrame frame;
        if (!BuildLocalFrame(expandedBox, frame))
        {
            return MakeFailure(SCBoxSegmentFailure2d::LocalTransformFailure);
        }
        const SCBox2d localBox = ToLocalBox(expandedBox, frame);
        if (!localBox.IsValid())
        {
            return MakeFailure(SCBoxSegmentFailure2d::NonFiniteInput);
        }

        std::unique_ptr<ISCSegment2d> localSegment = ToLocalSegment(segment, frame);
        if (localSegment == nullptr || !IsUsableLocalSegment(*localSegment))
        {
            return MakeFailure(SCBoxSegmentFailure2d::LocalTransformFailure);
        }

        return ClassifyLineOrArc(*localSegment, localBox);
    }
}  // namespace Geometry::Detail
