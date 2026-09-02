#include <cmath>

#include "Support/Epsilon.h"

#ifndef GEOMETRY_USE_DLL
#error "The installed SCGeometry target must publish GEOMETRY_USE_DLL."
#endif

int main()
{
    return std::abs(Geometry::kPi - 3.14159265358979323846) < 1e-15
               && Geometry::kArcBoundsCriticalAngles.size() == 4U
           ? 0
           : 1;
}
