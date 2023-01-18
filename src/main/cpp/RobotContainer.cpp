// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

double GetNumber(string name, double alt)
{
	return SmartDashboard::GetNumber(name, alt);
}

double GetBool(string name, double alt)
{
	return SmartDashboard::GetBoolean(name, alt);
}

RobotContainer::RobotContainer()
{
	InitializeSubsystems();
	ConfigureSubsystems();
}

void RobotContainer::InitializeSubsystems()
{
	m_test.Initialize(MotorConfig::NEO, EncoderConfig::REV, 0);
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);
	m_drivetrain.ConfigureTurnPID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE);
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
	auto [command, trajectory] = m_drivetrain.OpenPath("path.json");

	// Reset odometry to the starting pose of the trajectory.
	m_drivetrain.ConfigurePosition(trajectory.InitialPose());

	// no auto
	return new SequentialCommandGroup(
		move(command),
		InstantCommand([this]
					   { m_drivetrain.TankDriveVolts(0_V, 0_V); },
					   {}));
}

void RobotContainer::TeleopInit()
{
	m_drivetrain.ResetGyro();
	m_drivetrain.ResetEncoders();
}
void RobotContainer::TeleopPeriodic()
{

	m_drivetrain.Drive(m_controller.GetLeftY(), m_controller.GetLeftX());

	m_drivetrain.PrintPosition();
	m_drivetrain.PrintMotors();
	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintGyro();

	elevator.MoveHorizontally(1);
	elevator.MoveVertically(1);
}

void RobotContainer::AutonomousInit()
{
	m_drivetrain.ResetGyro();
}

void RobotContainer::AutonomousPeriodic()
{
	m_test.SetPosition(90, 1);
}

void RobotContainer::ConfigureControllerBindings() {}
