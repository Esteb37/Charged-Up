#pragma once

#include "Numbers.h"
#include "Point.h"

using degree_t = double;

namespace TD { namespace utility {

    class Line2D {
        public:
        Line2D(degree_t standardAngle);

        protected:
        degree_t standardAngle;
    };

    class Ray2D: public Line2D {
        public:
        Ray2D(Point2D &startingPosition, degree_t standardAngle);

        protected:
        Point2D startingPosition;
    };

    class Segment2D: public Ray2D {
        public:
        Segment2D(Point2D &startingPosition, Point2D &endingPosition);

        auto squaredLength() -> double;
        auto length() -> double;

        protected:
        Point2D endingPosition;
    };

    class Line3D {
        public:
        Line3D(degree_t xAngle, degree_t zAngle);

        protected:
        degree_t xAngle;
        degree_t zAngle;
    };

    class Ray3D: public Line3D {
        public:
        Ray3D(Point3D &startingPosition, degree_t xAngle, degree_t zAngle);

        protected:
        Point3D startingPosition;
    };

    class Segment3D: public Ray3D {
        public:
        Segment3D(Point3D &startingPosition, Point3D &endingPosition);

        auto squaredLength() -> double;
        auto length() -> double;

        protected:
        Point3D endingPosition;
    };

}}
