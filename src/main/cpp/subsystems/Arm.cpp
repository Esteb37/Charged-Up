#include "subsystems/Arm.h"

namespace TD {

        Arm::Arm(uint8_t shoulderPort, uint8_t elbowPort, uint8_t wristPort):
            m_shoulder(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(shoulderPort)),
            m_elbow(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(elbowPort)),  
            m_wrist(EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>(wristPort)){
            SetName("Arm");
            m_shoulder.SetName("Shoulder");
            m_elbow.SetName("Elbow");
            m_wrist.SetName("Wrist");

        }

        void Arm::Configure(){
            m_shoulder.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_elbow.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_wrist.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);

            m_shoulder.SetMaxSpeed(Speed::SHOULDER);
            m_elbow.SetMaxSpeed(Speed::ELBOW);
            m_wrist.SetMaxSpeed(Speed::WRIST);

            m_shoulder.ConfigurePositionPID(PID::Shoulder::P,PID::Shoulder::I,PID::Shoulder::D,PID::Shoulder::TOLERANCE);
            m_elbow.ConfigurePositionPID(PID::Elbow::P,PID::Elbow::I,PID::Elbow::D,PID::Elbow::TOLERANCE);
            m_wrist.ConfigurePositionPID(PID::Wrist::P,PID::Wrist::I,PID::Wrist::D,PID::Wrist::TOLERANCE);

            m_shoulder.SetPositionConversionFactor(DPR::SHOULDER);
            m_elbow.SetPositionConversionFactor(DPR::ELBOW);
            m_wrist.SetPositionConversionFactor(DPR::WRIST);
        }

        void Arm::Periodic() {}

        frc2::CommandPtr Arm::SetShoulderAngle(units::angle::degree_t angle, double speed) {
             return m_shoulder.SetAngleCmd(angle, speed);
        }

        frc2::CommandPtr Arm::SetElbowAngle(units::angle::degree_t angle, double speed) {
            return m_elbow.SetAngleCmd(angle, speed);
        }

        frc2::CommandPtr Arm::SetWristAngle(units::angle::degree_t angle, double speed) {
            return m_wrist.SetAngleCmd(angle, speed);
        }

        frc2::CommandPtr Arm::SetPose(Poses pose) {
            m_pose = Poses::kMoving;

            // TEMP
            return m_elbow.SetAngleCmd(0_deg,0);

           //switch(pose){
                // TODO       
           //}
        }

        Arm::Poses Arm::GetPose(){
            return m_pose;
        }

        std::string Arm::GetPoseStr(){
            switch(m_pose){
                case Poses::kMoving:
                    return "Moving";
                case Poses::kHome:
                    return "Home";
                case Poses::kTray:
                    return "Tray";
                case Poses::kLow:
                    return "Low";
                case Poses::kMiddle:
                    return "Middle";
                case Poses::kHigh:
                    return "High";
            }
        }

        void Arm::PrintPose(){
            SmartDashboard::PutString(GetName()+" Pose",GetPoseStr());
        }

        void Arm::PrintAngles(){
            m_shoulder.PrintPosition();
            m_elbow.PrintPosition();
            m_wrist.PrintPosition();
        }

        void Arm::ResetEncoders(){
            m_shoulder.ResetEncoder();
            m_elbow.ResetEncoder();
            m_wrist.ResetEncoder();
        }

}
