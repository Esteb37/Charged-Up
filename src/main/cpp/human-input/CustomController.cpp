#include "human-input/CustomController.h"

#include <cmath>

using namespace frc2;

namespace TD{

    CustomController::CustomController(int port): CommandXboxController(port){

    }

    double CustomController::GetLeftX() {
        double value = CommandXboxController::GetLeftX();

        if (utility::IsInThreshold(axisThresholdLeft, std::fabs(value))) {
            return value * leftAxisSensibility;
        }

        return 0.0;
    }

    double CustomController::GetLeftY() {
        double value = CommandXboxController::GetLeftY();

        if (utility::IsInThreshold(axisThresholdLeft, std::fabs(value))) {
            return value * leftAxisSensibility;
        }

        return 0.0;
    }


    double CustomController::GetRightX() {
        double value = CommandXboxController::GetRightX();

        if (utility::IsInThreshold(axisThresholdRight, std::fabs(value))) {
            return value * rightAxisSensibility;
        }

        return 0.0;
    }


    double CustomController::GetRightY() {
        double value = CommandXboxController::GetRightY();

        if (utility::IsInThreshold(axisThresholdRight, std::fabs(value))) {
            return value * rightAxisSensibility;
        }

        return 0.0;
    }

    double CustomController::GetLeftTriggerAxis() {
        double value = CommandXboxController::GetLeftTriggerAxis();

        if (utility::IsInThreshold(triggerThresholdLeft, value)) {
            return value * leftTriggerSensibility;
        }

        return 0.0;
    }

    double CustomController::GetRightTriggerAxis() {
        double value = CommandXboxController::GetRightTriggerAxis();

        if (utility::IsInThreshold(triggerThresholdRight, value)) {
            return value * rightTriggerSensibility;
        }

        return 0.0;
    }

    void CustomController::SetLeftAxisThreshold(double min, double max) {
        axisThresholdLeft = utility::Threshold{min, max};
    }

    void CustomController::SetRightAxisThreshold(double min, double max) {
        axisThresholdRight = utility::Threshold{min, max};
    }


    void CustomController::SetLeftTriggerThreshold(double min, double max) {
        triggerThresholdLeft = utility::Threshold{min, max};
    }

    void CustomController::SetRightTriggerThreshold(double min, double max) {
        triggerThresholdRight = utility::Threshold{min, max};
    }

    void CustomController::SetRightAxisSensibility(double sensibility){
        rightAxisSensibility = sensibility;

    }
    void CustomController::SetLeftAxisSensibility(double sensibility){
        leftAxisSensibility = sensibility;

    }

    void CustomController::SetRightTriggerSensibility(double sensibility){
        rightTriggerSensibility = sensibility;

    }

    void CustomController::SetLeftTriggerSensibility(double sensibility){
        leftTriggerSensibility = sensibility;
    }

    double CustomController::GetTriggerDifference(){
        return GetRightTriggerAxis() - GetLeftTriggerAxis();
    }

}