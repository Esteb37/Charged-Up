#ifndef POINT_HH
#define POINT_HH

#include "Numbers.h"

namespace TD { namespace utility {

    class Point2D {
        public:
        float x;
        float y;

        Point2D(float x, float y): x(x), y(y) {}

        auto distanceTo(Point2D& to) -> double;
        auto squaredDistanceTo(Point2D& to) -> double;
    };

    class Point3D: public Point2D {
        public:
        float z;

        Point3D(float x, float y, float z): Point2D(x, y), z(z) {}

        auto distanceTo(Point3D& to) -> double;
        auto squaredDistanceTo(Point3D& to) -> double;
    };

}}

#endif