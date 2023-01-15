#pragma once

#include "Numbers.h"

namespace TD { namespace utility {

    class Point2D {
        public:
        Point2D(float x, float y);

        auto distanceTo(Point2D const& to) -> double;
        auto squaredDistanceTo(Point2D const& to) -> double;

        auto GetX() -> float;
        auto GetY() -> float;

        protected:
        float x;
        float y;
    };

    class Point3D: public Point2D {
        public:
        Point3D(float x, float y, float z);

        auto distanceTo(Point3D const& to) -> double;
        auto squaredDistanceTo(Point3D const& to) -> double;

        auto GetZ() -> float;

        protected:
        float z;
    };

}}
