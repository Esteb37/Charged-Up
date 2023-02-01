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

void RobotContainer::RobotInit()
{
	m_drivetrain.ResetGyro();
	m_drivetrain.ResetEncoders();
}

void RobotContainer::RobotPeriodic()
{

	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintGyro();
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);
	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.InvertMove(true);
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
	m_drivetrain.TeleopDrive(*Common::Peripherals::controller);
}

	m_drivetrain.PrintPosition();
	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintGyro();
}

void RobotContainer::ConfigureControllerBindings() {
	m_commandController.POVUp().OnTrue(std::move(m_elevator.GotoPositiveY()););
	m_commandController.POVDown().OnTrue(std::move(m_elevator.GotoNegativeY()););

	m_commandController.POVRight().OnTrue(std::move(m_elevator.GotoPositiveX()););
	m_commandController.POVLeft().OnTrue(std::move(m_elevator.GotoNegativeX()););
}
