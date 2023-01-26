// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	ConfigureSubsystems();
}

void RobotContainer::RobotInit()
{
	m_drivetrain.ResetEncoders();
}

void RobotContainer::RobotPeriodic()
{
	m_gyro.Print();
	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintPose();
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetGyro(&m_gyro);
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);

	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::KP, Path::KP);

	m_drivetrain.ResetPose();
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
	DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
	deployDirectory = deployDirectory / "output" / "Path.wpilib.json";
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
	m_drivetrain.ResetEncoders();
	m_drivetrain.ResetPose();
}
void RobotContainer::TeleopPeriodic()
{
	m_drivetrain.Drive(m_controller.GetLeftY(), m_controller.GetLeftX());
}

void RobotContainer::AutonomousInit()
{
}

void RobotContainer::AutonomousPeriodic()
{
}

void RobotContainer::ConfigureControllerBindings() {}
