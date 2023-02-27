// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "human-input/CustomController.h"
#include "subsystems/Arm.h"
#include "subsystems/CustomGyro.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Limelight.h"
#include "subsystems/MotorSubsystemBase.h"
#include "subsystems/Turret.h"

#include <frc/Filesystem.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/RamseteAutoBuilder.h>
#include <wpi/fs.h>

using namespace pathplanner;

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
using namespace frc2::cmd;

class RobotContainer
{
public:
	RobotContainer();

	Command *GetAutonomousCommand();

	void ConfigureSubsystems();

	void ConfigureControllerBindings();

	void RobotInit();

	void RobotPeriodic();

	void AutonomousInit();

	void AutonomousPeriodic();

	void TeleopInit();

	void TeleopPeriodic();

	void Reset();

	CommandPtr GetArmPoseCmd(Arm::Poses);

	CommandPtr SetArmPose(Arm::Poses);

	CommandPtr SetTurretPose(Turret::Poses);

private:
	Drivetrain<DrivetrainTypes::SPX> m_drivetrain{
		M::DT::FRONT_RIGHT,
		M::DT::FRONT_LEFT,
		M::DT::BACK_RIGHT,
		M::DT::BACK_LEFT,
		ENC::DT::FRONT_RIGHT,
		ENC::DT::FRONT_LEFT,
		ENC::DT::BACK_RIGHT,
		ENC::DT::BACK_LEFT,
	};

	Arm m_arm{M::Arm::SHOULDER, M::Arm::ELBOW, M::Arm::WRIST};

	Intake m_intake{{M::Intake::RIGHT, M::Intake::LEFT}};

	Turret m_turret{M::TURRET};

	CustomController m_controller = CustomController(0);

	CustomGyro<GyroTypes::NAVX> m_gyro;

	CommandPtr m_currentArmCommand = InstantCommand().ToPtr();

	CommandPtr m_currentTurretCommand = InstantCommand().ToPtr();

	bool auto_done = false;
};
