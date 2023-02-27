#include "subsystems/Turret.h"

namespace TD
{

    Turret::Turret(unsigned int motorPort) : EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(motorPort), MotorSubsystemBase<MotorTypes::SPARK>(motorPort, true)
    {
        SetName("Turret");
    }

    void Turret::Periodic()
    {
        PrintPose();
    }

    CommandPtr Turret::SetPose(Poses pose)
    {
        double angle = 0;

        switch (pose)
        {
        case Poses::kFront:
            angle = 0;
            break;
        case Poses::kBack:
            angle = 180;
            break;
        case Poses::kSide:
            angle = 90;
            break;
        }

        return Sequence(
            InstantCommand([this, angle, pose]()
                           { 
                            m_pose = Poses::kMoving;
                            SmartDashboard::PutString(
                                "Turret Target", PoseToString(pose)); }),
            SetPositionCmd(angle, Speed::TURRET),
            InstantCommand([this, pose]()
                           { m_pose = pose; }));
    }

    Turret::Poses Turret::GetPose()
    {
        return m_pose;
    }

    std::string Turret::GetPoseStr()
    {
        return PoseToString(m_pose);
    }

    std::string Turret::PoseToString(Poses pose)
    {
        switch (pose)
        {
        case Poses::kMoving:
            return "Moving";
        case Poses::kFront:
            return "Front";
        case Poses::kBack:
            return "Back";
        case Poses::kSide:
            return "Side";
        default:
            return "Unknown";
        }
    }

    void Turret::PrintPose()
    {
        SmartDashboard::PutString(GetName() + " Pose", GetPoseStr());
    }

    void Turret::PrintAngle()
    {
        PrintPosition();
    }
}
