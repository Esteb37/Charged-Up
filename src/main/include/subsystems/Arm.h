#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>

#include "subsystems/EncoderSubsystemBase.h"

namespace TD {
    class Arm : public frc2::SubsystemBase {
    public:
        Arm(uint8_t shoulderPort, uint8_t armPort);

        void Periodic() override;

        frc2::CommandPtr SetShoulderAngle(units::angle::degree_t, double speed);
        frc2::CommandPtr SetArmPosition(units::angle::degree_t, double speed);
        frc2::CommandPtr SetSubsystemPosiotions(units::angle::degree_t, units::angle::degree_t, double speed);

    private:
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> shoulder;
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> arm;

    };
}