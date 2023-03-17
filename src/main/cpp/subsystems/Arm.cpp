#include "subsystems/Arm.h"

namespace TD
{

    Arm::Arm(uint8_t shoulderPort, uint8_t elbowPort, uint8_t wristPort) : m_shoulder(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(shoulderPort, true)),
                                                                           m_elbow(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(elbowPort, true)),
                                                                           m_wrist(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(wristPort, true))
    {
        SetName("Arm");
        m_shoulder.SetName("Shoulder");
        m_elbow.SetName("Elbow");
        m_wrist.SetName("Wrist");

    }

    void Arm::Configure()
    {

        // TODO : Check Max Speeds
        m_shoulder.SetMaxSpeed(Speed::SHOULDER);
        m_elbow.SetMaxSpeed(Speed::ELBOW);
        m_wrist.SetMaxSpeed(Speed::WRIST);

        // TODO : Test PID
        m_shoulder.ConfigurePositionPID(PID::Shoulder::P, PID::Shoulder::I, PID::Shoulder::D, PID::Shoulder::TOLERANCE);
        m_elbow.ConfigurePositionPID(PID::Elbow::P, PID::Elbow::I, PID::Elbow::D, PID::Elbow::TOLERANCE);
        m_wrist.ConfigurePositionPID(PID::Wrist::P, PID::Wrist::I, PID::Wrist::D, PID::Wrist::TOLERANCE);

        // TODO : Check PCFs
        m_shoulder.SetPositionConversionFactor(DPR::SHOULDER);
        m_elbow.SetPositionConversionFactor(DPR::ELBOW);
        m_wrist.SetPositionConversionFactor(DPR::WRIST);

        // TODO : Check Inversions
        m_shoulder.InvertEncoder(false);
        m_elbow.InvertEncoder(false);
        m_wrist.InvertEncoder(false);
        
        m_shoulder.InvertMotor(false);
        m_elbow.InvertMotor(false);
        m_wrist.InvertMotor(false);

        // TODO : If PID works, change this
        // m_shoulder.SetMinMaxPosition(-180,180);
        // m_shoulder.SetPositionSafety(true);
        // m_elbow.SetMinMaxPosition(-180,180);
        // m_elbow.SetPositionSafety(true);
        // m_wrist.SetMinMaxPosition(-180,180);
        // m_wrist.SetPositionSafety(true);
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
        return m_wrist.SetPositionCmd(angle.value(), speed );
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
        case Poses::kTaxi:
            shoulderAngle = Angles::Shoulder::TAXI;
            elbowAngle = Angles::Elbow::TAXI;
            wristAngle = Angles::Wrist::TAXI;
            break;
        case Poses::kMoving:
        default:
            break;
        }

        return Sequence(
            InstantCommand([this, pose]()
                           { m_pose = Poses::kMoving;
                               SmartDashboard::PutString("Arm Target", PoseToString(pose)); }),
            Parallel(
                SetWristAngle(wristAngle, Speed::WRIST)),
                SetShoulderAngle(shoulderAngle, Speed::SHOULDER),
                SetElbowAngle(elbowAngle, Speed::ELBOW),
            InstantCommand([this, pose]()
                           { m_pose = pose; }));
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
            return "Cone High";
        case Poses::kBoxLow:
            return "Box Low";
        case Poses::kBoxMiddle:
            return "Box Middle";
        case Poses::kBoxHigh:
            return "Box High";
        case Poses::kTaxi:
            return "Taxi";
        default:
            return "Unknown";
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
        m_shoulder.ResetEncoder(Angles::Shoulder::HOME.value());
        m_elbow.ResetEncoder(Angles::Elbow::HOME.value());
        m_wrist.ResetEncoder(Angles::Wrist::HOME.value());
    }
}
