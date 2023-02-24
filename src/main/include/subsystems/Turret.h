#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Turret : public frc2::SubsystemBase {
    public:
        Turret();

        void Periodic() override;

    private:
        rev::CANSparkMax leadController{M::CAN::TURRET, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax followerController{M::CAN::TURRET_FOLLOWER, rev::CANSparkMax::MotorType::kBrushless};

        rev::SparkMaxRelativeEncoder leadEncoder = leadController.GetEncoder();
        rev::SparkMaxRelativeEncoder followerEncoder = followerController.GetEncoder();
};
