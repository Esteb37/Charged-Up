#pragma once

#include <type_traits>

namespace TD { namespace utility {

    class Vector2D {
        public:
        Vector2D(float x, float y);

        auto GetX() -> float;
        auto GetY() -> float;

        protected:
        float x;
        float y;
    };


}}
