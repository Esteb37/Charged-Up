#include "utility/Threshold.hh"

namespace TD { namespace utility {

    bool IsInThreshold(Threshold threshold, double value) {
        return value >= threshold.min && value <= threshold.max;
    }

}}