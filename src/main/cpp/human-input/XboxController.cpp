#include "human-input/XboxController.hh"

#include <cmath>

bool TD::XboxController::ButtonTop() {
    return controller.GetYButton();
}

bool TD::XboxController::ButtonBottom() {
    return controller.GetAButton();
}

bool TD::XboxController::ButtonLeft() {
    return controller.GetXButton();
}

bool TD::XboxController::ButtonRight() {
    return controller.GetBButton();
}

int TD::XboxController::Pov() {
    return controller.GetPOV();
}

bool TD::XboxController::PovTop() {
    return Pov() == Constants::TOP_POV_ID;
}

bool TD::XboxController::PovBottom() {
    return Pov() == Constants::BOTTOM_POV_ID;
}

bool TD::XboxController::PovLeft() {
    return Pov() == Constants::LEFT_POV_ID;
}

bool TD::XboxController::PovRight() {
    return Pov() == Constants::RIGHT_POV_ID;
}

double TD::XboxController::AxisXLeft() {
    double value = controller.GetLeftX();

    if (utility::IsInThreshold(axisThresholdLeft, std::fabs(value))) {
        return value;
    }

    return 0.0;
}

double TD::XboxController::AxisYLeft() {
    double value = controller.GetLeftY();

    if (utility::IsInThreshold(axisThresholdLeft, std::fabs(value))) {
        return value;
    }

    return 0.0;
}

double TD::XboxController::AxisXRight() {
    double value = controller.GetRightX();

    if (utility::IsInThreshold(axisThresholdRight, std::fabs(value))) {
        return value;
    }

    return 0.0;
}

double TD::XboxController::AxisYRight() {
    double value = controller.GetRightY();

    if (utility::IsInThreshold(axisThresholdRight, std::fabs(value))) {
        return value;
    }

    return 0.0;
}

bool TD::XboxController::AxisButtonLeft() {
    return controller.GetLeftStickButton();
}

bool TD::XboxController::AxisButtonRight() {
    return controller.GetRightStickButton();
}

double TD::XboxController::TriggerLeft() {
    double value = controller.GetLeftTriggerAxis();

    if (utility::IsInThreshold(triggerThresholdLeft, value)) {
        return value;
    }

    return 0.0;
}

double TD::XboxController::TriggerRight() {
    double value = controller.GetRightTriggerAxis();

    if (utility::IsInThreshold(triggerThresholdRight, value)) {
        return value;
    }

    return 0.0;
}

bool TD::XboxController::BumperLeft() {
    return controller.GetLeftBumper();
}

bool TD::XboxController::BumperRight() {
    return controller.GetRightBumper();
}


bool TD::XboxController::CenterButtonLeft() {
    return controller.GetBackButton();
}

bool TD::XboxController::CenterButtonRight() {
    return controller.GetStartButton();
}

void TD::XboxController::SetAxisThresholdLeft(double min, double max) {
    axisThresholdLeft = utility::Threshold{min, max};
}

void TD::XboxController::SetAxisThresholdRight(double min, double max) {
    axisThresholdRight = utility::Threshold{min, max};
}


void TD::XboxController::SetTriggerThresholdLeft(double min, double max) {
    triggerThresholdLeft = utility::Threshold{min, max};
}

void TD::XboxController::SetTriggerThresholdRight(double min, double max) {
    triggerThresholdRight = utility::Threshold{min, max};
}
