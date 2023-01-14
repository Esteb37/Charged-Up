#include "utility/Point.h"

#include <math.h>

namespace TD { namespace utility {

    auto Point2D::squaredDistanceTo(Point2D const& to) -> double {
        return sqrt(pow(to.x - x, 2) + pow((to.y - y), 2));
    }

    auto Point2D::distanceTo(Point2D const& to) -> double {
        return sqrt(squaredDistanceTo(to));
    }

    auto Point3D::squaredDistanceTo(Point3D const& to) -> double {
        return sqrt(pow(to.x - x, 2) + pow((to.y - y), 2) + pow(to.z - z, 2));
    }

    auto Point3D::distanceTo(Point3D const& to) -> double {
        return sqrt(squaredDistanceTo(to));
    }

}}
