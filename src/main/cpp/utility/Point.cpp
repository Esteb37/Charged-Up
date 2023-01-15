#include "utility/Point.h"

#include <math.h>

namespace TD { namespace utility {

    Point2D::Point2D(float x, float y): x(x), y(y) {}
    Point3D::Point3D(float x, float y, float z): Point2D(x, y), z(z) {}

    auto Point2D::GetX() -> float { return x; }
    auto Point2D::GetY() -> float { return y; }
    auto Point3D::GetZ() -> float { return z; }

    auto Point2D::squaredDistanceTo(Point2D const& to) -> double {
        return abs(pow(to.x - x, 2) + pow((to.y - y), 2));
    }

    auto Point2D::distanceTo(Point2D const& to) -> double {
        return sqrt(squaredDistanceTo(to));
    }

    auto Point3D::squaredDistanceTo(Point3D const& to) -> double {
        return abs(pow(to.x - x, 2) + pow((to.y - y), 2) + pow(to.z - z, 2));
    }

    auto Point3D::distanceTo(Point3D const& to) -> double {
        return sqrt(squaredDistanceTo(to));
    }

}}
