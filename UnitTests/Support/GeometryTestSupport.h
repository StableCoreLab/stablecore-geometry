#pragma once

#include <cmath>
#include <concepts>
#include <cstdlib>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "Geometry.h"

namespace Geometry::Test
{
    namespace Detail
    {
        template <typename T>
        concept HasXY = requires(const T& value) {
                            value.x;
                            value.y;
                        };

        template <typename T>
        concept HasMinMaxFields = requires(const T& value) {
                                      value.minPoint;
                                      value.maxPoint;
                                  };

        template <typename T>
        concept HasMinMaxGetters = requires(const T& value) {
                                       value.MinPoint();
                                       value.MaxPoint();
                                   };

        template <typename T>
        concept HasProjectionFields = requires(const T& value) {
                                          value.point;
                                          value.parameter;
                                          value.distanceSquared;
                                          value.isOnSegment;
                                      };

        template <typename T>
        concept HasPointCount = requires(const T& value) {
                                    {
                                        value.PointCount()
                                        } -> std::convertible_to<std::size_t>;
                                };

        template <typename T>
        concept HasPointAt = requires(const T& value, std::size_t index) { value.PointAt(index); };

        template <typename T>
        concept HasHoleCount = requires(const T& value) {
                                   {
                                       value.HoleCount()
                                       } -> std::convertible_to<std::size_t>;
                               };

        template <typename T>
        concept HasHoleAt = requires(const T& value, std::size_t index) { value.HoleAt(index); };

        template <typename T>
        concept HasIsValid = requires(const T& value) {
                                 {
                                     value.IsValid()
                                     } -> std::convertible_to<bool>;
                             };

        template <typename T>
        [[nodiscard]] decltype(auto) MinPointLike(const T& value)
        {
            if constexpr (HasMinMaxFields<T>)
            {
                return (value.minPoint);
            } else
            {
                return (value.MinPoint());
            }
        }

        template <typename T>
        [[nodiscard]] decltype(auto) MaxPointLike(const T& value)
        {
            if constexpr (HasMinMaxFields<T>)
            {
                return (value.maxPoint);
            } else
            {
                return (value.MaxPoint());
            }
        }

        template <typename T>
        [[nodiscard]] std::string FormatNumber(T value)
        {
            std::ostringstream out;
            out << std::setprecision(std::numeric_limits<long double>::max_digits10) << static_cast<long double>(value);
            return out.str();
        }

        [[nodiscard]] inline std::string FormatLocation(std::string_view file, int line)
        {
            std::ostringstream out;
            out << file << ':' << line;
            return out.str();
        }

        [[noreturn]] inline void Fail(std::string_view kind, std::string_view file, int line, std::string_view message)
        {
            std::ostringstream out;
            out << FormatLocation(file, line) << ": " << kind << ": " << message;
            throw std::runtime_error(out.str());
        }
        template <typename T>
        [[nodiscard]] bool NearlyEqual(T lhs, T rhs, double eps)
        {
            using std::abs;
            return abs(static_cast<long double>(lhs) - static_cast<long double>(rhs)) <= eps;
        }

        template <HasXY T>
        [[nodiscard]] std::string DescribePointLike(const T& value)
        {
            std::ostringstream out;
            out << '(' << FormatNumber(value.x) << ", " << FormatNumber(value.y) << ')';
            return out.str();
        }

        template <typename T>
        [[nodiscard]] std::string DescribeBoxLike(const T& value)
        {
            std::ostringstream out;
            out << '{';
            if constexpr (HasMinMaxFields<T>)
            {
                out << "min=" << DescribePointLike(MinPointLike(value))
                    << ", max=" << DescribePointLike(MaxPointLike(value));
                if constexpr (HasIsValid<T>)
                {
                    out << ", valid=" << (value.IsValid() ? "true" : "false");
                }
            } else if constexpr (HasMinMaxGetters<T>)
            {
                out << "min=" << DescribePointLike(MinPointLike(value))
                    << ", max=" << DescribePointLike(MaxPointLike(value));
                if constexpr (HasIsValid<T>)
                {
                    out << ", valid=" << (value.IsValid() ? "true" : "false");
                }
            } else
            {
                out << "<unformattable box-like type>";
            }
            out << '}';
            return out.str();
        }

        template <typename T>
        [[nodiscard]] std::string DescribeProjectionLike(const T& value)
        {
            std::ostringstream out;
            out << "{point=" << DescribePointLike(value.point) << ", parameter=" << FormatNumber(value.parameter)
                << ", distanceSquared=" << FormatNumber(value.distanceSquared)
                << ", isOnSegment=" << (value.isOnSegment ? "true" : "false") << '}';
            return out.str();
        }

        template <typename T>
        [[nodiscard]] std::string DescribePolylineLike(const T& value)
        {
            std::ostringstream out;
            out << "{pointCount=" << value.PointCount();
            if constexpr (requires { value.IsClosed(); })
            {
                out << ", closure=" << (value.IsClosed() ? "closed" : "open");
            }
            out << ", points=[";
            for (std::size_t i = 0; i < value.PointCount(); ++i)
            {
                if (i > 0)
                {
                    out << ", ";
                }
                out << DescribePointLike(value.PointAt(i));
            }
            out << "]}";
            return out.str();
        }

        template <typename T>
        [[nodiscard]] std::string DescribePolygonLike(const T& value)
        {
            std::ostringstream out;
            out << "{outer=" << DescribePolylineLike(value.OuterRing()) << ", holeCount=" << value.HoleCount();
            if constexpr (HasHoleAt<T>)
            {
                out << ", holes=[";
                for (std::size_t i = 0; i < value.HoleCount(); ++i)
                {
                    if (i > 0)
                    {
                        out << ", ";
                    }
                    out << DescribePolylineLike(value.HoleAt(i));
                }
                out << "]";
            }
            out << "}";
            return out.str();
        }
    }  // namespace Detail

    template <Detail::HasXY Actual, Detail::HasXY Expected>
    inline void AssertPointNear(const Actual& actual,
                                const Expected& expected,
                                double eps,
                                std::string_view actualExpr,
                                std::string_view expectedExpr,
                                std::string_view file,
                                int line)
    {
        if (Detail::NearlyEqual(actual.x, expected.x, eps) && Detail::NearlyEqual(actual.y, expected.y, eps))
        {
            return;
        }

        Detail::Fail("point mismatch",
                     file,
                     line,
                     std::string(actualExpr) + " = " + Detail::DescribePointLike(actual) + ", " +
                         std::string(expectedExpr) + " = " + Detail::DescribePointLike(expected) +
                         ", eps = " + Detail::FormatNumber(eps));
    }

    template <Detail::HasXY Actual, Detail::HasXY Expected>
    inline void AssertVectorNear(const Actual& actual,
                                 const Expected& expected,
                                 double eps,
                                 std::string_view actualExpr,
                                 std::string_view expectedExpr,
                                 std::string_view file,
                                 int line)
    {
        AssertPointNear(actual, expected, eps, actualExpr, expectedExpr, file, line);
    }

    template <typename Actual, typename Expected>
    inline void AssertNear(const Actual& actual,
                           const Expected& expected,
                           double eps,
                           std::string_view actualExpr,
                           std::string_view expectedExpr,
                           std::string_view file,
                           int line)
    {
        if (Detail::NearlyEqual(actual, expected, eps))
        {
            return;
        }

        Detail::Fail("value mismatch",
                     file,
                     line,
                     std::string(actualExpr) + " = " + Detail::FormatNumber(actual) + ", " + std::string(expectedExpr) +
                         " = " + Detail::FormatNumber(expected) + ", eps = " + Detail::FormatNumber(eps));
    }

    template <typename Actual, typename Expected>
    inline void AssertBoxNear(const Actual& actual,
                              const Expected& expected,
                              double eps,
                              std::string_view actualExpr,
                              std::string_view expectedExpr,
                              std::string_view file,
                              int line)
    {
        if constexpr (Detail::HasIsValid<Actual> && Detail::HasIsValid<Expected>)
        {
            const bool actualValid = actual.IsValid();
            const bool expectedValid = expected.IsValid();
            if (actualValid != expectedValid)
            {
                Detail::Fail("box validity mismatch",
                             file,
                             line,
                             std::string(actualExpr) + " = " + Detail::DescribeBoxLike(actual) + ", " +
                                 std::string(expectedExpr) + " = " + Detail::DescribeBoxLike(expected));
            }

            if (!actualValid && !expectedValid)
            {
                return;
            }
        }

        AssertPointNear(
            Detail::MinPointLike(actual), Detail::MinPointLike(expected), eps, actualExpr, expectedExpr, file, line);
        AssertPointNear(
            Detail::MaxPointLike(actual), Detail::MaxPointLike(expected), eps, actualExpr, expectedExpr, file, line);
    }

    template <typename Actual, typename Expected>
    inline void AssertProjectionNear(const Actual& actual,
                                     const Expected& expected,
                                     double eps,
                                     std::string_view actualExpr,
                                     std::string_view expectedExpr,
                                     std::string_view file,
                                     int line)
    {
        AssertPointNear(actual.point, expected.point, eps, actualExpr, expectedExpr, file, line);
        AssertNear(actual.parameter, expected.parameter, eps, actualExpr, expectedExpr, file, line);
        AssertNear(actual.distanceSquared, expected.distanceSquared, eps, actualExpr, expectedExpr, file, line);

        if (actual.isOnSegment != expected.isOnSegment)
        {
            Detail::Fail("projection flag mismatch",
                         file,
                         line,
                         std::string(actualExpr) + " = " + Detail::DescribeProjectionLike(actual) + ", " +
                             std::string(expectedExpr) + " = " + Detail::DescribeProjectionLike(expected));
        }
    }

    template <Detail::HasPointCount Actual, Detail::HasPointCount Expected>
    inline void AssertPolylineNear(const Actual& actual,
                                   const Expected& expected,
                                   double eps,
                                   std::string_view actualExpr,
                                   std::string_view expectedExpr,
                                   std::string_view file,
                                   int line)
    {
        if (actual.PointCount() != expected.PointCount())
        {
            Detail::Fail("polyline point-count mismatch",
                         file,
                         line,
                         std::string(actualExpr) + " = " + Detail::DescribePolylineLike(actual) + ", " +
                             std::string(expectedExpr) + " = " + Detail::DescribePolylineLike(expected));
        }

        if constexpr (requires {
                          actual.IsClosed();
                          expected.IsClosed();
                      })
        {
            if (actual.IsClosed() != expected.IsClosed())
            {
                Detail::Fail("polyline closure mismatch",
                             file,
                             line,
                             std::string(actualExpr) + " = " + Detail::DescribePolylineLike(actual) + ", " +
                                 std::string(expectedExpr) + " = " + Detail::DescribePolylineLike(expected));
            }
        }

        for (std::size_t i = 0; i < actual.PointCount(); ++i)
        {
            AssertPointNear(actual.PointAt(i), expected.PointAt(i), eps, actualExpr, expectedExpr, file, line);
        }
    }

    template <typename Actual, typename Expected>
    inline void AssertPolygonNear(const Actual& actual,
                                  const Expected& expected,
                                  double eps,
                                  std::string_view actualExpr,
                                  std::string_view expectedExpr,
                                  std::string_view file,
                                  int line)
    {
        if (actual.HoleCount() != expected.HoleCount())
        {
            Detail::Fail("polygon hole-count mismatch",
                         file,
                         line,
                         std::string(actualExpr) + " = " + Detail::DescribePolygonLike(actual) + ", " +
                             std::string(expectedExpr) + " = " + Detail::DescribePolygonLike(expected));
        }

        AssertPolylineNear(actual.OuterRing(), expected.OuterRing(), eps, actualExpr, expectedExpr, file, line);
        for (std::size_t i = 0; i < actual.HoleCount(); ++i)
        {
            AssertPolylineNear(actual.HoleAt(i), expected.HoleAt(i), eps, actualExpr, expectedExpr, file, line);
        }
    }

    [[nodiscard]] inline Geometry::SCPolyline2d MakePolyline(
        std::initializer_list<Geometry::SCPoint2d> points,
        Geometry::SCPolylineClosure closure = Geometry::SCPolylineClosure::Open)
    {
        return Geometry::SCPolyline2d(std::vector<Geometry::SCPoint2d>(points), closure);
    }

    [[nodiscard]] inline Geometry::SCPolyline2d MakeRectangleRing(const Geometry::SCPoint2d& minPoint,
                                                                const Geometry::SCPoint2d& maxPoint)
    {
        return MakePolyline(
            {
                minPoint,
                Geometry::SCPoint2d{maxPoint.x, minPoint.y},
                maxPoint,
                Geometry::SCPoint2d{minPoint.x, maxPoint.y},
            },
            Geometry::SCPolylineClosure::Closed);
    }

    [[nodiscard]] inline Geometry::SCPolyline2d MakeRectangleHoleRing(const Geometry::SCPoint2d& minPoint,
                                                                    const Geometry::SCPoint2d& maxPoint)
    {
        return MakePolyline(
            {
                minPoint,
                Geometry::SCPoint2d{minPoint.x, maxPoint.y},
                maxPoint,
                Geometry::SCPoint2d{maxPoint.x, minPoint.y},
            },
            Geometry::SCPolylineClosure::Closed);
    }

    [[nodiscard]] inline Geometry::SCPolygon2d MakeRectanglePolygon(const Geometry::SCPoint2d& minPoint,
                                                                  const Geometry::SCPoint2d& maxPoint)
    {
        return Geometry::SCPolygon2d(MakeRectangleRing(minPoint, maxPoint));
    }
}  // namespace Geometry::Test

#define GEOMETRY_TEST_ASSERT_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_POINT_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertPointNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_VECTOR_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertVectorNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_BOX_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertBoxNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_PROJECTION_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertProjectionNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertPolylineNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)

#define GEOMETRY_TEST_ASSERT_POLYGON_NEAR(actual, expected, eps) \
    ::Geometry::Test::AssertPolygonNear((actual), (expected), (eps), #actual, #expected, __FILE__, __LINE__)
