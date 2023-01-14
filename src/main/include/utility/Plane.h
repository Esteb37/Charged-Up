#ifndef PLANE_HH
#define PLANE_HH

#include <array>

#include "Point.h"

namespace TD { namespace utility {

    template<int AxisLength>
    class BoundPlane {
        public:

        static constexpr int axisLength = AxisLength;

        BoundPlane() = default;

        auto origin() -> Point2D;

        auto topCenter() -> Point2D;
        auto bottomCenter() -> Point2D;
        auto leftCenter() -> Point2D;
        auto rightCenter() -> Point2D;

        auto bottomLeft() -> Point2D;
        auto bottomRight() -> Point2D;
        auto topLeft() -> Point2D;
        auto topRight() -> Point2D;

    };

}}
#endif