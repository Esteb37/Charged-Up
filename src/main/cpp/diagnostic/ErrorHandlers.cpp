#include "diagnostic/ErrorHandlers.h"

#include <frc/smartdashboard/SmartDashboard.h>

namespace TD { namespace ErrorHandlers {
    std::map<int, int> emmitedErrors{};

    void HandleRevLibError(rev::REVLibError error) {
        if (error == rev::REVLibError::kOk) return;

        int errorN = ((int) error) * REVLIB_MULTIPLIER;
        emmitedErrors[errorN] += 1;

        std::string errorMsg = "REVLib error {\ntype: " + std::to_string(errorN) + "\n" + std::to_string(emmitedErrors[errorN]) + "\n}";
        frc::SmartDashboard::PutString("REVLib Error", errorMsg);
    }

}}
