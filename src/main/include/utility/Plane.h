#pragma once

#include <array>

#include "Point.h"

namespace TD { namespace utility {

    template<int AxisLength>
    class BoundPlane {
        public:

        static constexpr int axisLength = AxisLength;

        BoundPlane() = default;

        auto origin() -> const Point2D;

        auto topCenter() -> const Point2D;
        auto bottomCenter() -> const Point2D;
        auto leftCenter() -> const Point2D;
        auto rightCenter() -> const Point2D;

        auto bottomLeft() -> const Point2D;
        auto bottomRight() -> const Point2D;
        auto topLeft() -> const Point2D;
        auto topRight() -> const Point2D;

    };

}}
