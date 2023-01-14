#include "utility/Lines.h"

namespace TD { namespace utility {

    auto Segment2D::length() -> double {
        return startingPosition.distanceTo(endingPosition);
    }

}}
