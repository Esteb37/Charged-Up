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

	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::RIGHT_P, Path::LEFT_P);

	m_drivetrain.ResetPose();

	m_drivetrain.InvertMove(true);
	m_drivetrain.InvertRightEncoders(true);
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{

	frc::DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	PathPlannerTrajectory trajectory = PathPlanner::loadPath("path", PathConstraints(Path::MAX_SPEED, Path::MAX_ACCELERATION));

	RamseteAutoBuilder autoBuilder(
		[this]()
		{ return m_drivetrain.GetPose(); },
		[this](auto initPose)
		{ return m_drivetrain.SetPose(initPose); },
		RamseteController(Path::RAMSETE_B, Path::RAMSETE_ZETA),
		kinematics,
		SimpleMotorFeedforward<units::meters>(Path::KS, Path::KV, Path::KA),
		[this]
		{ return m_drivetrain.GetWheelSpeeds(); },
		PIDConstants(Path::LEFT_P, 0, 0),
		[this](auto left, auto right)
		{ m_drivetrain.TankDriveVolts(left, right); },
		{},
		{&m_drivetrain});

	frc2::CommandPtr command = autoBuilder.followPath(trajectory);
	return command.get();
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
