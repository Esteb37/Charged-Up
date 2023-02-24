#pragma once

#include "utility/Threshold.hh"
#include <frc2/command/button/CommandXboxController.h>

using namespace frc2;

namespace TD {

    class CustomController : public CommandXboxController {
        public:

            CustomController(int port);


            double GetLeftX();
            double GetLeftY();
            double GetRightX();
            double GetRightY();
            double GetLeftTriggerAxis();
            double GetRightTriggerAxis();

            void SetLeftAxisThreshold(double min, double max);
            void SetRightAxisThreshold(double min, double max);

            void SetLeftTriggerThreshold(double min, double max);
            void SetRightTriggerThreshold(double min, double max);

            void SetRightAxisSensibility(double);
            void SetLeftAxisSensibility(double);
            void SetRightTriggerSensibility(double);
            void SetLeftTriggerSensibility(double);

            double GetTriggerDifference();

        protected:
            utility::Threshold axisThresholdLeft{0, 1};
            utility::Threshold axisThresholdRight{0, 1};
            utility::Threshold triggerThresholdLeft{0, 1};
            utility::Threshold triggerThresholdRight{0, 1};

            double rightAxisSensibility = 1;
            double leftAxisSensibility = 1;
            double rightTriggerSensibility = 1;
            double leftTriggerSensibility = 1;
    };
}