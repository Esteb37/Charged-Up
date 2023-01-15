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

	m_drivetrain.Initialize(
		M::CAN::FRONT_RIGHT,
		M::CAN::FRONT_LEFT,
		M::CAN::BACK_RIGHT,
		M::CAN::BACK_LEFT,
		DIO::Encoder::DRIVETRAIN_RA,
		DIO::Encoder::DRIVETRAIN_RB,
		DIO::Encoder::DRIVETRAIN_LA,
		DIO::Encoder::DRIVETRAIN_LB);
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);
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

void RobotContainer::TeleopInit() {}
void RobotContainer::TeleopPeriodic()
{

	m_drivetrain.Drive(m_controller.GetLeftY(), m_controller.GetLeftX());

	m_drivetrain.PrintPosition();
	m_drivetrain.PrintMotors();
	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintGyro();
}
void RobotContainer::ConfigureControllerBindings() {}
