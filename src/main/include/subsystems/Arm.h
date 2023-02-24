#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>

#include "subsystems/EncoderSubsystemBase.h"

using namespace frc2;

namespace TD {
    class Arm : public frc2::SubsystemBase {
    public:

       
        Arm(uint8_t shoulderPort, uint8_t elbowPort, uint8_t wristPort);

        enum Poses{
            kHome,
            kLow,
            kMiddle,
            kHigh,
            kTray,
            kMoving,
        };

        void Periodic() override;

        CommandPtr SetShoulderAngle(units::angle::degree_t, double speed);
        CommandPtr SetElbowAngle(units::angle::degree_t, double speed);
        CommandPtr SetWristAngle(units::angle::degree_t, double speed);

        CommandPtr SetPose(Poses);

        void PrintPose();
        void PrintAngles();
        Poses GetPose();
        std::string GetPoseStr();

    private:
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_shoulder;
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_elbow;
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_wrist;

        Poses m_pose = Poses::kHome;
    };
}