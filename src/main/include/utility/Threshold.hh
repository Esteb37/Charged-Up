#pragma once

namespace TD { namespace utility {
    struct Threshold {
        double min;
        double max;

        Threshold(double min, double max): min(min), max(max) {}
    };

    bool IsInThreshold(Threshold threshold, double value);
}}
