#include "diagnostic/ErrorHandlers.h"

#include <frc/SmartDashboard.h>
#include <map>

namespace TD { namespace ErrorHandlers {
    namespace rev {
        std::map<int, int> emmitedErrors;

        void HandleRevLibError(rev::REVLibError error) {
            if (error == rev::REVLibError::kOk) return;

            emmitedErrors[error] += 1;

            frc::SmartDashboard::PutString("REVLib error: oftype(" + error + "), instances(" + emmitedErrors[error] + ")");
        }

    }

}}
