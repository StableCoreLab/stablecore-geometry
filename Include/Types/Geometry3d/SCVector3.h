#pragma once

#include <cmath>
#include <string>
#include <type_traits>

#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    template <typename T>
    using SCVectorLengthType3 = std::conditional_t<std::is_floating_point_v<T>, T, double>;

    template <typename T>
    struct SCVector3 : SCVec3Storage<T>
    {
        using SCVec3Storage<T>::SCVec3Storage;
        using LengthType = SCVectorLengthType3<T>;

        [[nodiscard]] static constexpr SCVector3 FromXYZ(T xValue, T yValue, T zValue)
        {
            return SCVector3(xValue, yValue, zValue);
        }

        [[nodiscard]] constexpr auto LengthSquared() const -> LengthType
        {
            return static_cast<LengthType>(this->x) * static_cast<LengthType>(this->x) +
                   static_cast<LengthType>(this->y) * static_cast<LengthType>(this->y) +
                   static_cast<LengthType>(this->z) * static_cast<LengthType>(this->z);
        }

        [[nodiscard]] auto Length() const -> LengthType
        {
            using std::sqrt;
            return sqrt(LengthSquared());
        }

        [[nodiscard]] SCVector3 Normalized(double eps = kDefaultEpsilon) const
        {
            const auto length = Length();
            if (length <= eps)
            {
                return SCVector3{};
            }
            return *this / length;
        }

        [[nodiscard]] std::string DebugString() const
        {
            return this->SCVec3Storage<T>::DebugString("SCVector3");
        }
    };

    template <typename T>
    [[nodiscard]] constexpr SCVector3<T> operator+(const SCVector3<T>& lhs, const SCVector3<T>& rhs)
    {
        return SCVector3<T>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector3<T> operator-(const SCVector3<T>& lhs, const SCVector3<T>& rhs)
    {
        return SCVector3<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector3<T> operator-(const SCVector3<T>& vector)
    {
        return SCVector3<T>(-vector.x, -vector.y, -vector.z);
    }

    template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector3<T> operator*(const SCVector3<T>& vector, Scalar scalar)
    {
        return SCVector3<T>(
            static_cast<T>(vector.x * scalar), static_cast<T>(vector.y * scalar), static_cast<T>(vector.z * scalar));
    }

    template <typename Scalar, typename T, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector3<T> operator*(Scalar scalar, const SCVector3<T>& vector)
    {
        return vector * scalar;
    }

    template <typename T, typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
    [[nodiscard]] constexpr SCVector3<T> operator/(const SCVector3<T>& vector, Scalar scalar)
    {
        return SCVector3<T>(
            static_cast<T>(vector.x / scalar), static_cast<T>(vector.y / scalar), static_cast<T>(vector.z / scalar));
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector3<T> operator-(const SCPoint3<T>& lhs, const SCPoint3<T>& rhs)
    {
        return SCVector3<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <typename T>
    [[nodiscard]] constexpr auto Dot(const SCVector3<T>& lhs, const SCVector3<T>& rhs) -> SCVectorLengthType3<T>
    {
        return static_cast<SCVectorLengthType3<T>>(lhs.x) * static_cast<SCVectorLengthType3<T>>(rhs.x) +
               static_cast<SCVectorLengthType3<T>>(lhs.y) * static_cast<SCVectorLengthType3<T>>(rhs.y) +
               static_cast<SCVectorLengthType3<T>>(lhs.z) * static_cast<SCVectorLengthType3<T>>(rhs.z);
    }

    template <typename T>
    [[nodiscard]] constexpr SCVector3<T> Cross(const SCVector3<T>& lhs, const SCVector3<T>& rhs)
    {
        return SCVector3<T>(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
    }

    using SCVector3d = SCVector3<double>;
}  // namespace Geometry
