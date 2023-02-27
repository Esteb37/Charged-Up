#pragma once

#include "Constants.h"
#include "subsystems/EncoderSubsystemBase.h"
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

using namespace frc2;
using namespace frc2::cmd;
namespace TD
{
    class Arm : public frc2::SubsystemBase
    {
    public:
        Arm(uint8_t shoulderPort, uint8_t elbowPort, uint8_t wristPort);

        // DO NOT CHANGE ORDER
        enum Poses
        {
            kHome,
            kPickup,
            kConeLow,
            kConeMiddle,
            kConeHigh,
            kBoxLow,
            kBoxMiddle,
            kBoxHigh,
            kTray,
            kMoving,
            kTaxi
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

        void Configure();

        void ResetEncoders();

        static std::string PoseToString(Poses pose);

    private:
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_shoulder;
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_elbow;
        EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO> m_wrist;

        frc2::CommandPtr *m_currentCommand = nullptr;

        Poses m_pose = Poses::kHome;
    };
}