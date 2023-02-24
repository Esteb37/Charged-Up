// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>

RobotContainer::RobotContainer()
{
	
}

void RobotContainer::RobotInit()
{
	m_gyro.Calibrate();
	ConfigureControllerBindings();

	mc_controller.SetAxisThresholdLeft(0.2, 1.0);
	mc_controller.SetAxisThresholdRight(0.2, 1.0);

	shoulder.SetSparkMaxIdleMode(CANSparkMax::IdleMode::kBrake);
	arm.SetSparkMaxIdleMode(CANSparkMax::IdleMode::kBrake);

	shoulder.InvertMotor(true);

	ConfigureSubsystems();
}

void RobotContainer::RobotPeriodic()
{

}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetGyro(&m_gyro);
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);

	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::KP, Path::KP);

	m_drivetrain.ResetPose();

	shoulder.SetPositionConversionFactor(90/80.1);
	//shoulder.InvertEncoder(true);

	arm.SetName("Arm");
	shoulder.SetName("Shoulder");
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{

	DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
	deployDirectory = deployDirectory / "output" / "auto.wpilib.json";
	auto trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

	RamseteCommand ramseteCommand{
		trajectory,
		[this]()
		{ return m_drivetrain.GetPose(); },
		RamseteController{Path::RAMSETE_B,
						  Path::RAMSETE_ZETA},
		SimpleMotorFeedforward<units::meters>{
			Path::KS,
			Path::KV,
			Path::KA},
		kinematics,
		[this]
		{ return m_drivetrain.GetWheelSpeeds(); },
		frc2::PIDController{Path::KP, 0, 0},
		frc2::PIDController{Path::KP, 0, 0},
		[this](auto left, auto right)
		{ m_drivetrain.TankDriveVolts(left, right); },
		{&m_drivetrain}};

	// Reset odometry to the starting pose of the trajectory.
	m_drivetrain.SetPose(trajectory.InitialPose());

	// no auto
	return new frc2::SequentialCommandGroup(
		std::move(ramseteCommand),
		frc2::InstantCommand([this]
							 { m_drivetrain.TankDriveVolts(0_V, 0_V); },
							 {}));
}

void RobotContainer::TeleopInit()
{
	m_gyro.Reset();
	m_drivetrain.ResetPose();
	arm.ResetEncoder();
	shoulder.ResetEncoder();
}
void RobotContainer::TeleopPeriodic()
{
	double output = mc_controller.TriggerRight() - mc_controller.TriggerLeft();
	double rotation = mc_controller.AxisXLeft();

	m_drivetrain.Drive(output, rotation * (output == 0 ? 0.5 : 1.0));

	// arm.SetMotor(mc_controller.AxisYLeft()/5);
	// shoulder.SetMotor(mc_controller.AxisYRight()/5);
	// arm.PrintPosition();
	// shoulder.PrintPosition();
	// intake1.SetMotor(mc_controller.TriggerRight()-mc_controller.TriggerLeft());
	// intake2.SetMotor(-(mc_controller.TriggerRight()-mc_controller.TriggerLeft()));
}

void RobotContainer::AutonomousInit()
{
	m_gyro.Reset();
	m_drivetrain.ResetEncoders();
	arm.ResetEncoder();
	shoulder.ResetEncoder();

}

void RobotContainer::AutonomousPeriodic()
{
	frc2::CommandScheduler::GetInstance().Schedule(shoulder.SetAngleCmd(-45_deg,0.6));	
}

void RobotContainer::ConfigureControllerBindings() {
	m_commandController.A().OnTrue(arm.SetAngleCmd(25_deg, 0.05));
}
