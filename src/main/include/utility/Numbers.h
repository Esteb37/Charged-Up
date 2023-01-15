#pragma once

#include <type_traits>

namespace TD { namespace utility {

    template<typename T>
    struct of_arithmetic_kin {
        static_assert(std::is_arithmetic<T>::value, "T is not an arithmetic type! (int, double, long, etc.)");
    };

}}
