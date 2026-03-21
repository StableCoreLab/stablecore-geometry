#include <cassert>
#include <cmath>

#include "types/Box2.h"

using geometry::Box2d;
using geometry::Box2i;
using geometry::Point2d;
using geometry::Point2i;

int main()
{
    Box2i emptyBox;
    assert(!emptyBox.IsValid());

    const Box2i invalidBox(Point2i(3, 4), Point2i(1, 2));
    assert(!invalidBox.IsValid());

    const Box2i pointBox(Point2i(2, 3), Point2i(2, 3));
    assert(pointBox.IsValid());
    assert(pointBox.Width() == 0.0);
    assert(pointBox.Height() == 0.0);
    assert(pointBox.Area() == 0.0);
    assert(pointBox.Center() == geometry::Point2<double>(2.0, 3.0));

    const Box2i boxA(Point2i(1, 2), Point2i(4, 6));
    assert(boxA.IsValid());
    assert(boxA.Width() == 3.0);
    assert(boxA.Height() == 4.0);
    assert(boxA.Area() == 12.0);
    assert(boxA.Center() == geometry::Point2<double>(2.5, 4.0));

    Box2i expanded;
    expanded.ExpandToInclude(Point2i(5, 7));
    assert(expanded.IsValid());
    assert(expanded.GetMinPoint() == Point2i(5, 7));
    assert(expanded.GetMaxPoint() == Point2i(5, 7));

    expanded.ExpandToInclude(Point2i(2, 9));
    assert(expanded.GetMinPoint() == Point2i(2, 7));
    assert(expanded.GetMaxPoint() == Point2i(5, 9));

    expanded.ExpandToInclude(Box2i(Point2i(0, 1), Point2i(3, 8)));
    assert(expanded.GetMinPoint() == Point2i(0, 1));
    assert(expanded.GetMaxPoint() == Point2i(5, 9));

    const Box2i beforeIgnored = expanded;
    expanded.ExpandToInclude(Box2i(Point2i(10, 10), Point2i(4, 12)));
    assert(expanded == beforeIgnored);

    Box2d floatingBox;
    floatingBox.ExpandToInclude(Point2d(1.5, 2.5));
    floatingBox.ExpandToInclude(Point2d(-0.5, 3.0));
    assert(floatingBox.IsValid());
    assert(std::abs(floatingBox.Width() - 2.0) < 1e-12);
    assert(std::abs(floatingBox.Height() - 0.5) < 1e-12);
    assert(std::abs(floatingBox.Area() - 1.0) < 1e-12);
    assert(std::abs(floatingBox.Center().x - 0.5) < 1e-12);
    assert(std::abs(floatingBox.Center().y - 2.75) < 1e-12);

    const Box2i sameA(Point2i(1, 2), Point2i(3, 4));
    const Box2i sameB(Point2i(1, 2), Point2i(3, 4));
    const Box2i different(Point2i(1, 2), Point2i(3, 5));
    assert(sameA == sameB);
    assert(!(sameA != sameB));
    assert(sameA != different);

    return 0;
}
