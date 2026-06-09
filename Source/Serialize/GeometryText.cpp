#include "Serialize/GeometryText.h"

#include <iomanip>
#include <istream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace Geometry::Serialize
{
    namespace
    {
        template <typename... Ts>
        [[nodiscard]] std::string MakeText(const char* tag, Ts... values)
        {
            std::ostringstream oss;
            oss.setf(std::ios::fmtflags(0), std::ios::floatfield);
            oss << std::setprecision(std::numeric_limits<double>::max_digits10) << tag;
            ((oss << ' ' << values), ...);
            return oss.str();
        }

        [[nodiscard]] bool ExpectTag(std::istringstream& iss, const char* tag)
        {
            std::string actualTag;
            if (!(iss >> actualTag))
            {
                return false;
            }

            return actualTag == tag;
        }

        [[nodiscard]] bool ReadDouble(std::istringstream& iss, double& value)
        {
            return static_cast<bool>(iss >> value);
        }

        [[nodiscard]] bool ReadBool(std::istringstream& iss, bool& value)
        {
            int raw = 0;
            if (!(iss >> raw))
            {
                return false;
            }

            if (raw != 0 && raw != 1)
            {
                return false;
            }

            value = raw != 0;
            return true;
        }

        [[nodiscard]] bool ExpectEnd(std::istringstream& iss)
        {
            iss >> std::ws;
            return iss.eof();
        }

        [[nodiscard]] std::string ClosureToken(Geometry::SCPolylineClosure closure)
        {
            return closure == Geometry::SCPolylineClosure::Closed ? "closed" : "open";
        }

        [[nodiscard]] bool ReadClosure(std::istringstream& iss, Geometry::SCPolylineClosure& closure)
        {
            std::string token;
            if (!(iss >> token))
            {
                return false;
            }

            if (token == "open")
            {
                closure = Geometry::SCPolylineClosure::Open;
                return true;
            }

            if (token == "closed")
            {
                closure = Geometry::SCPolylineClosure::Closed;
                return true;
            }

            return false;
        }
    }  // namespace

    std::string ToText(const Geometry::SCPoint2d& value)
    {
        return MakeText("SCPoint2d", value.x, value.y);
    }

    bool FromText(std::string_view text, Geometry::SCPoint2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCPoint2d"))
        {
            return false;
        }

        if (!ReadDouble(iss, value.x) || !ReadDouble(iss, value.y))
        {
            return false;
        }

        return ExpectEnd(iss);
    }

    std::string ToText(const Geometry::SCVector2d& value)
    {
        return MakeText("SCVector2d", value.x, value.y);
    }

    bool FromText(std::string_view text, Geometry::SCVector2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCVector2d"))
        {
            return false;
        }

        if (!ReadDouble(iss, value.x) || !ReadDouble(iss, value.y))
        {
            return false;
        }

        return ExpectEnd(iss);
    }

    std::string ToText(const Geometry::SCBox2d& value)
    {
        const Geometry::SCPoint2d minPoint = value.MinPoint();
        const Geometry::SCPoint2d maxPoint = value.MaxPoint();
        return MakeText("SCBox2d", minPoint.x, minPoint.y, maxPoint.x, maxPoint.y);
    }

    bool FromText(std::string_view text, Geometry::SCBox2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCBox2d"))
        {
            return false;
        }

        Geometry::SCPoint2d minPoint{};
        Geometry::SCPoint2d maxPoint{};
        if (!ReadDouble(iss, minPoint.x) || !ReadDouble(iss, minPoint.y) || !ReadDouble(iss, maxPoint.x) ||
            !ReadDouble(iss, maxPoint.y))
        {
            return false;
        }

        value = Geometry::SCBox2d::FromMinMax(minPoint, maxPoint);
        return ExpectEnd(iss);
    }

    std::string ToText(const Geometry::SCSegmentProjection2d& value)
    {
        return MakeText("SCSegmentProjection2d",
                        value.point.x,
                        value.point.y,
                        value.parameter,
                        value.distanceSquared,
                        value.isOnSegment ? 1 : 0);
    }

    bool FromText(std::string_view text, Geometry::SCSegmentProjection2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCSegmentProjection2d"))
        {
            return false;
        }

        if (!ReadDouble(iss, value.point.x) || !ReadDouble(iss, value.point.y) || !ReadDouble(iss, value.parameter) ||
            !ReadDouble(iss, value.distanceSquared) || !ReadBool(iss, value.isOnSegment))
        {
            return false;
        }

        return ExpectEnd(iss);
    }

    std::string ToText(const Geometry::SCArcSegment2d& value)
    {
        return MakeText(
            "SCArcSegment2d", value.center.x, value.center.y, value.radius, value.startAngle, value.sweepAngle);
    }

    bool FromText(std::string_view text, Geometry::SCArcSegment2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCArcSegment2d"))
        {
            return false;
        }

        if (!ReadDouble(iss, value.center.x) || !ReadDouble(iss, value.center.y) || !ReadDouble(iss, value.radius) ||
            !ReadDouble(iss, value.startAngle) || !ReadDouble(iss, value.sweepAngle))
        {
            return false;
        }

        return ExpectEnd(iss) && value.IsValid();
    }

    std::string ToText(const Geometry::SCPolyline2d& value)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fmtflags(0), std::ios::floatfield);
        oss << std::setprecision(std::numeric_limits<double>::max_digits10) << "SCPolyline2d "
            << ClosureToken(value.IsClosed() ? Geometry::SCPolylineClosure::Closed : Geometry::SCPolylineClosure::Open)
            << ' ' << value.PointCount();
        for (std::size_t i = 0; i < value.PointCount(); ++i)
        {
            const Geometry::SCPoint2d point = value.PointAt(i);
            oss << ' ' << point.x << ' ' << point.y;
        }
        return oss.str();
    }

    bool FromText(std::string_view text, Geometry::SCPolyline2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCPolyline2d"))
        {
            return false;
        }

        Geometry::SCPolylineClosure closure{};
        std::size_t pointCount = 0;
        if (!ReadClosure(iss, closure) || !(iss >> pointCount))
        {
            return false;
        }

        std::vector<Geometry::SCPoint2d> points(pointCount);
        for (std::size_t i = 0; i < pointCount; ++i)
        {
            if (!ReadDouble(iss, points[i].x) || !ReadDouble(iss, points[i].y))
            {
                return false;
            }
        }

        if (!ExpectEnd(iss))
        {
            return false;
        }

        value = Geometry::SCPolyline2d(std::move(points), closure);
        return true;
    }

    std::string ToText(const Geometry::SCPolygon2d& value)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fmtflags(0), std::ios::floatfield);
        oss << std::setprecision(std::numeric_limits<double>::max_digits10) << "SCPolygon2d " << ToText(value.OuterRing())
            << ' ' << value.HoleCount();
        for (std::size_t i = 0; i < value.HoleCount(); ++i)
        {
            oss << ' ' << ToText(value.HoleAt(i));
        }
        return oss.str();
    }

    bool FromText(std::string_view text, Geometry::SCPolygon2d& value)
    {
        std::istringstream iss{std::string(text)};
        if (!ExpectTag(iss, "SCPolygon2d"))
        {
            return false;
        }

        std::string outerTag;
        if (!(iss >> outerTag) || outerTag != "SCPolyline2d")
        {
            return false;
        }

        std::string outerText = outerTag;
        std::string token;
        std::size_t outerPointCount = 0;
        if (!(iss >> token))
        {
            return false;
        }
        outerText += " " + token;
        if (token != "open" && token != "closed")
        {
            return false;
        }
        if (!(iss >> outerPointCount))
        {
            return false;
        }
        outerText += " " + std::to_string(outerPointCount);
        for (std::size_t i = 0; i < outerPointCount * 2; ++i)
        {
            if (!(iss >> token))
            {
                return false;
            }
            outerText += " " + token;
        }

        Geometry::SCPolyline2d outerRing;
        if (!FromText(outerText, outerRing))
        {
            return false;
        }
        if (!outerRing.IsClosed())
        {
            return false;
        }

        std::size_t holeCount = 0;
        if (!(iss >> holeCount))
        {
            return false;
        }

        std::vector<Geometry::SCPolyline2d> holes;
        holes.reserve(holeCount);
        for (std::size_t holeIndex = 0; holeIndex < holeCount; ++holeIndex)
        {
            std::string holeTag;
            if (!(iss >> holeTag) || holeTag != "SCPolyline2d")
            {
                return false;
            }

            std::string holeText = holeTag;
            if (!(iss >> token))
            {
                return false;
            }
            holeText += " " + token;
            if (token != "open" && token != "closed")
            {
                return false;
            }

            std::size_t holePointCount = 0;
            if (!(iss >> holePointCount))
            {
                return false;
            }
            holeText += " " + std::to_string(holePointCount);
            for (std::size_t i = 0; i < holePointCount * 2; ++i)
            {
                if (!(iss >> token))
                {
                    return false;
                }
                holeText += " " + token;
            }

            Geometry::SCPolyline2d hole;
            if (!FromText(holeText, hole))
            {
                return false;
            }
            if (!hole.IsClosed())
            {
                return false;
            }
            holes.push_back(std::move(hole));
        }

        if (!ExpectEnd(iss))
        {
            return false;
        }

        value = Geometry::SCPolygon2d(std::move(outerRing), std::move(holes));
        return value.IsValid();
    }
}  // namespace Geometry::Serialize
