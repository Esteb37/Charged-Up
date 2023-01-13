// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/ElevatorBase.h"
#include "subsystems/FeederBase.h"
#include "subsystems/IntakeBase.h"
#include "subsystems/Limelight.h"
#include "subsystems/ShooterBase.h"
#include "subsystems/TurretBase.h"
#include "subsystems/SolenoidClawBase.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

using namespace TD;
using namespace std;
using namespace frc;
using namespace frc2;

class RobotContainer
{
public:
	RobotContainer();

	frc2::Command *GetAutonomousCommand();

	void InitializeSubsystems();

	void ConfigureSubsystems();

	void ConfigureControllerBindings();

	void RobotInit();

	void RobotPeriodic();

	void AutonomousInit();

	void AutonomousPeriodic();

	void TeleopInit();

	void TeleopPeriodic();

private:
	Drivetrain m_drivetrain;

	ShooterBase m_shooter;

	IntakeBase m_intake;

	FeederBase m_feeder;

	ElevatorBase m_elevator;

	SolenoidClawBase m_claw;

	TurretBase m_turret;

	Limelight m_limelight = Limelight::GetInstance();

	CommandXboxController m_controller = CommandXboxController(0);

	DifferentialDriveKinematics kDriveKinematics{0.77_m};
};
