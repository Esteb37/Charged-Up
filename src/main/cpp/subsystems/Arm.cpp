#include "subsystems/Arm.h"

namespace TD
{

    Arm::Arm(uint8_t shoulderPort, uint8_t elbowPort, uint8_t wristPort) : m_shoulder(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(shoulderPort)),
                                                                           m_elbow(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(elbowPort)),
                                                                           m_wrist(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(wristPort))
    {
        SetName("Arm");
        m_shoulder.SetName("Shoulder");
        m_elbow.SetName("Elbow");
        m_wrist.SetName("Wrist");
    }

    void Arm::Configure()
    {
        m_shoulder.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_elbow.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_wrist.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        m_shoulder.SetMaxSpeed(Speed::SHOULDER);
        m_elbow.SetMaxSpeed(Speed::ELBOW);
        m_wrist.SetMaxSpeed(Speed::WRIST);

        m_shoulder.ConfigurePositionPID(PID::Shoulder::P, PID::Shoulder::I, PID::Shoulder::D, PID::Shoulder::TOLERANCE);
        m_elbow.ConfigurePositionPID(PID::Elbow::P, PID::Elbow::I, PID::Elbow::D, PID::Elbow::TOLERANCE);
        m_wrist.ConfigurePositionPID(PID::Wrist::P, PID::Wrist::I, PID::Wrist::D, PID::Wrist::TOLERANCE);

        m_shoulder.SetPositionConversionFactor(DPR::SHOULDER);
        m_elbow.SetPositionConversionFactor(DPR::ELBOW);
        m_wrist.SetPositionConversionFactor(DPR::WRIST);
    }

    void Arm::Periodic()
    {
        PrintPose();
    }

    CommandPtr Arm::SetShoulderAngle(units::angle::degree_t angle, double speed)
    {
        return m_shoulder.SetPositionCmd(angle.value(), speed);
    }

    CommandPtr Arm::SetElbowAngle(units::angle::degree_t angle, double speed)
    {
        return m_elbow.SetPositionCmd(angle.value(), speed);
    }

    CommandPtr Arm::SetWristAngle(units::angle::degree_t angle, double speed)
    {
        return m_wrist.SetPositionCmd(angle.value(), speed);
    }

    CommandPtr Arm::SetPose(Poses pose)
    {
        auto shoulderAngle = 0_deg;
        auto elbowAngle = 0_deg;
        auto wristAngle = 0_deg;

        switch (pose)
        {
        case Poses::kHome:
            shoulderAngle = Angles::Shoulder::HOME;
            elbowAngle = Angles::Elbow::HOME;
            wristAngle = Angles::Wrist::HOME;
            break;
        case Poses::kPickup:
            shoulderAngle = Angles::Shoulder::PICKUP;
            elbowAngle = Angles::Elbow::PICKUP;
            wristAngle = Angles::Wrist::PICKUP;
            break;
        case Poses::kConeLow:
            shoulderAngle = Angles::Shoulder::CONE_LOW;
            elbowAngle = Angles::Elbow::CONE_LOW;
            wristAngle = Angles::Wrist::CONE_LOW;
            break;
        case Poses::kConeMiddle:
            shoulderAngle = Angles::Shoulder::CONE_MIDDLE;
            elbowAngle = Angles::Elbow::CONE_MIDDLE;
            wristAngle = Angles::Wrist::CONE_MIDDLE;
            break;
        case Poses::kConeHigh:
            shoulderAngle = Angles::Shoulder::CONE_HIGH;
            elbowAngle = Angles::Elbow::CONE_HIGH;
            wristAngle = Angles::Wrist::CONE_HIGH;
            break;
        case Poses::kBoxLow:
            shoulderAngle = Angles::Shoulder::BOX_LOW;
            elbowAngle = Angles::Elbow::BOX_LOW;
            wristAngle = Angles::Wrist::BOX_LOW;
            break;
        case Poses::kBoxMiddle:
            shoulderAngle = Angles::Shoulder::BOX_MIDDLE;
            elbowAngle = Angles::Elbow::BOX_MIDDLE;
            wristAngle = Angles::Wrist::BOX_MIDDLE;
            break;
        case Poses::kBoxHigh:
            shoulderAngle = Angles::Shoulder::BOX_HIGH;
            elbowAngle = Angles::Elbow::BOX_HIGH;
            wristAngle = Angles::Wrist::BOX_HIGH;
            break;
        case Poses::kTray:
            shoulderAngle = Angles::Shoulder::TRAY;
            elbowAngle = Angles::Elbow::TRAY;
            wristAngle = Angles::Wrist::TRAY;
            break;
        }

        auto moveCommand = Parallel(
            SetShoulderAngle(shoulderAngle, Speed::SHOULDER),
            SetElbowAngle(elbowAngle, Speed::ELBOW),
            SetWristAngle(wristAngle, Speed::WRIST));

        auto cmd = Either(
            InstantCommand().ToPtr(),
            Sequence(
                InstantCommand([this, pose]()
                               { m_pose = Poses::kMoving;
                               SmartDashboard::PutString("Target", PoseToString(pose)); }),
                std::move(moveCommand),
                InstantCommand([this, pose]()
                               { m_pose = pose; })),
            [this, pose]()
            {
                return m_pose == pose;
            });

        return cmd;
    }

    Arm::Poses Arm::GetPose()
    {
        return m_pose;
    }

    std::string Arm::GetPoseStr()
    {
        return PoseToString(m_pose);
    }

    std::string Arm::PoseToString(Poses pose)
    {
        switch (pose)
        {
        case Poses::kMoving:
            return "Moving";
        case Poses::kHome:
            return "Home";
        case Poses::kTray:
            return "Tray";
        case Poses::kPickup:
            return "Pickup";
        case Poses::kConeLow:
            return "Cone Low";
        case Poses::kConeMiddle:
            return "Cone Middle";
        case Poses::kConeHigh:
            return "ConeHigh";
        case Poses::kBoxLow:
            return "Box Low";
        case Poses::kBoxMiddle:
            return "Box Middle";
        case Poses::kBoxHigh:
            return "Box High";
        }
    }

    void Arm::PrintPose()
    {
        SmartDashboard::PutString(GetName() + " Pose", GetPoseStr());
    }

    void Arm::PrintAngles()
    {
        m_shoulder.PrintPosition();
        m_elbow.PrintPosition();
        m_wrist.PrintPosition();
    }

    void Arm::ResetEncoders()
    {
        m_shoulder.ResetEncoder();
        m_elbow.ResetEncoder();
        m_wrist.ResetEncoder();
    }
}
