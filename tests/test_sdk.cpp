#include <cassert>
#include <cmath>

#include "sdk/Geometry.h"

using geometry::sdk::Contains;
using geometry::sdk::Distance;
using geometry::sdk::GeoBox2d;
using geometry::sdk::GeoPoint2d;
using geometry::sdk::GeoVector2d;
using geometry::sdk::Intersects;
using geometry::sdk::ProjectPointToSegment;

int main()
{
    const GeoPoint2d a = GeoPoint2d::FromXY(0.0, 0.0);
    const GeoPoint2d b = GeoPoint2d::FromXY(3.0, 4.0);
    const GeoVector2d offset = b - a;

    assert(std::abs(Distance(a, b) - 5.0) < 1e-12);
    assert(offset == GeoVector2d(GeoVector2d{3.0, 4.0}));
    assert(std::abs(offset.LengthSquared() - 25.0) < 1e-12);
    assert(std::abs(offset.Length() - 5.0) < 1e-12);
    assert(a + offset == b);
    assert(b - offset == a);

    const auto projection = ProjectPointToSegment(GeoPoint2d{3.0, 1.0}, a, b);
    assert(std::abs(projection.point.x - 1.56) < 1e-12);
    assert(std::abs(projection.point.y - 2.08) < 1e-12);
    assert(projection.isOnSegment);

    const GeoBox2d boxA = GeoBox2d::FromMinMax(GeoPoint2d{0.0, 0.0}, GeoPoint2d{2.0, 2.0});
    const GeoBox2d boxB = GeoBox2d::FromMinMax(GeoPoint2d{1.0, 1.0}, GeoPoint2d{3.0, 3.0});
    const GeoBox2d boxC = GeoBox2d::FromMinMax(GeoPoint2d{3.1, 3.1}, GeoPoint2d{4.0, 4.0});
    const GeoBox2d invalidBox = GeoBox2d::FromMinMax(GeoPoint2d{2.0, 2.0}, GeoPoint2d{1.0, 1.0});

    assert(boxA.IsValid());
    assert(!invalidBox.IsValid());
    assert(std::abs(boxA.Width() - 2.0) < 1e-12);
    assert(std::abs(boxA.Height() - 2.0) < 1e-12);
    assert(boxA.Center() == GeoPoint2d(GeoPoint2d{1.0, 1.0}));
    assert(Contains(boxA, GeoPoint2d{1.0, 1.0}));
    assert(Intersects(boxA, boxB));
    assert(!Intersects(boxA, boxC));
    assert(!Contains(invalidBox, GeoPoint2d{1.0, 1.0}));

    return 0;
}
