#include "subsystems/Arm.h"

namespace TD {

        Arm::Arm(uint8_t shoulderPort, uint8_t armPort):
            shoulder(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(shoulderPort)),
            arm(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(armPort)) {

        }

        void Arm::Periodic() {}

        frc2::CommandPtr Arm::SetShoulderAngle(units::angle::degree_t angle, double speed) {
             return shoulder.SetAngleCmd(angle, speed);
        }

        frc2::CommandPtr Arm::SetArmPosition(units::angle::degree_t angle, double speed) {
            return arm.SetAngleCmd(angle, speed);
        }

        frc2::CommandPtr Arm::SetSubsystemPosiotions(units::angle::degree_t shoulderAngle, units::angle::degree_t armAngle, double speed) {
            return SetShoulderAngle(shoulderAngle, speed).AlongWith(SetArmPosition(armAngle, speed));
        }

}
