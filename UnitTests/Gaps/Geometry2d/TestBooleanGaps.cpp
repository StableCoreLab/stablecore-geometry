#include <gtest/gtest.h>

#include "Core/Boolean.h"
#include "Core/ShapeOps.h"

using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;

TEST(BooleanGapTest, HarderArrangementDegeneraciesRemainOpen)
{
    const SCPolygon2d first(SCPolyline2d({SCPoint2d{0.0, 0.0},
                                      SCPoint2d{8.0, 0.0},
                                      SCPoint2d{8.0, 2.0},
                                      SCPoint2d{5.0, 2.0},
                                      SCPoint2d{5.0, 2.0 + 1e-12},
                                      SCPoint2d{0.0, 2.0 + 1e-12}},
                                     SCPolylineClosure::Closed));
    const SCPolygon2d second(SCPolyline2d({SCPoint2d{3.0, 0.0},
                                       SCPoint2d{10.0, 0.0},
                                       SCPoint2d{10.0, 2.0 + 1e-12},
                                       SCPoint2d{6.0, 2.0 + 1e-12},
                                       SCPoint2d{6.0, 4.0},
                                       SCPoint2d{3.0, 4.0}},
                                      SCPolylineClosure::Closed));
    (void)first;
    (void)second;
    GTEST_SKIP() << "Known gap: arrangement degeneracies below current "
                    "tolerance-scale recovery are still open.";
}
