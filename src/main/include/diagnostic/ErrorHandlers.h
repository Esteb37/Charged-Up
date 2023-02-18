#pragma once

#include <rev/REVLibError.h>

#include <memory>
#include <string>
#include <map>

namespace TD { namespace ErrorHandlers {
    constexpr unsigned short REVLIB_MULTIPLIER = 1000;

    std::map<int, int> emmitedErrors;

    void HandleRevLibError(rev::REVLibError);
}}
