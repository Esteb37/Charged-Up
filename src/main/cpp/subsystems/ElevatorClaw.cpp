#include "subsystems/ElevatorClaw.h"

namespace TD {
    ElevatorClaw::ElevatorClaw() = default;

    void ElevatorClaw::Periodic() {}

    frc2::CommandPtr ElevatorClaw::InvertAngle() {
        auto degrees = wrist.GetAngle();
        return this -> RunOnce([](){}).Until([this, &degrees] {
            return wrist.SetAngle(-degrees);
        });
    } 

    frc2::CommandPtr ElevatorClaw::SetAngle(units::degree_t degrees) {
        return this -> RunOnce([](){}).Until([this, &degrees] {
            return wrist.SetAngle(degrees);
        });
    } 

    frc2::CommandPtr ElevatorClaw::GotoGroundAngle() {
        return this -> RunOnce([](){}).Until([this] {
            return wrist.SetAngle(units::degree_t(0));
        });
    }

    frc2::CommandPtr ElevatorClaw::OpenClaw() {

    }
    frc2::CommandPtr ElevatorClaw::CloseClaw() {

    }
}
