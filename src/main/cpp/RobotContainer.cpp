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
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);
	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);
	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Move::TOLERANCE);
	m_drivetrain.InvertRightEncoders(true);
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
	m_drivetrain.PrintEncoders();
}

void RobotContainer::AutonomousInit()
{
	m_drivetrain.ResetEncoders();
}

void RobotContainer::AutonomousPeriodic()
{
	m_drivetrain.PrintEncoders();
	m_drivetrain.Move(3, 0.6);
}

void RobotContainer::ConfigureControllerBindings() {}
