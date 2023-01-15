#include "utility/Vector.h"

namespace TD { namespace utility {

    Vector2D::Vector2D(float x, float y):
    x(x), y(y) {}

    auto Vector2D::GetX() -> float {
        return x;
    }

    auto Vector2D::GetY() -> float {
        return y;
    }

}}
