#ifndef VECTOR_HH
#define VECTOR_HH

#include <type_traits>

namespace TD { namespace utility {

    class Vector2D {
        public:
        float x;
        float y;

        Vector2D(float x, float y): x(x), y(y) {}
    };


}}

#endif