#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>

#include "EncoderSubsystemBase.h"
#include "MotorClawBase.h"

namespace TD {

    class ElevatorClaw : public frc2::SubsystemBase {
        public:
            ElevatorClaw();

            void Periodic() override;

            frc2::CommandPtr InvertAngle();
            frc2::CommandPtr SetAngle(units::degree_t);
            frc2::CommandPtr GotoGroundAngle();

            frc2::CommandPtr OpenClaw();
            frc2::CommandPtr CloseClaw();

        private:
        EncoderSubsystemBase wrist;
        MotorClawBase claw;
    };
}
