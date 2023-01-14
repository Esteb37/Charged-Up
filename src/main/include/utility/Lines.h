#ifndef LINES_HH
#define LINES_HH

#include "Numbers.h"
#include "Point.h"

namespace TD { namespace utility {

    class Line2D {
        public:
        const double standardAngle;

        Line2D(double standardAngle = 0): standardAngle(standardAngle) {}
    };

    class Ray2D: public Line2D {
        public:
        Point2D startingPosition;

        Ray2D(Point2D startingPosition, double standardAngle = 0): startingPosition(startingPosition) {}

    };

    class Segment2D: public Ray2D {
        public:
        Point2D endingPosition;

        auto length() -> double;
    };

}}

#endif