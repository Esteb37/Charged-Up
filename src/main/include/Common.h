#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>

namespace TD {

    namespace Common {

        namespace Peripherals {

            frc::XboxController controller{0};
            frc2::CommandXboxController controller{0};

        }

    }

}
