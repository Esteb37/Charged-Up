// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#define _USE_MATH_DEFINES
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <math.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

using port = uint8_t;

// MOTORS
namespace M
{
	namespace Arm
	{
		constexpr port SHOULDER = 8;
		constexpr port ELBOW = 1;
		constexpr port WRIST = 6;
	}

	namespace DT
	{
		constexpr port FRONT_RIGHT = 1;
		constexpr port FRONT_LEFT = 2;
		constexpr port BACK_RIGHT = 3;
		constexpr port BACK_LEFT = 4;
	}

	namespace Intake
	{
		constexpr port RIGHT = 4;
		constexpr port LEFT = 5;
	}
}


namespace ENC
{
	namespace DT
	{
		constexpr port FRONT_RIGHT = 0;
		constexpr port FRONT_LEFT = 1;
		constexpr port BACK_RIGHT = 2;
		constexpr port BACK_LEFT = 3;

	}
}

namespace PID
{
	namespace Move
	{
		constexpr double P = 8.6846;
		constexpr double I = 0;
		constexpr double D = 1.3567;
		constexpr double TOLERANCE = 0.06;
	}

	namespace Turn
	{
		constexpr double P = 100;
		constexpr double I = 0;
		constexpr double D = 100;
		constexpr double TOLERANCE = 0.01;
	}

	namespace Align
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Distance
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace TurretAlign
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace TurretAngle
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Shoulder{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Elbow{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Wrist{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}
}

namespace LL
{
	constexpr double OBJECTIVE_HEIGHT = 135;
	constexpr double HEIGHT = 35;
	constexpr double ANGLE_DEG = 45;
	constexpr double ANGLE_RAD = ANGLE_DEG * M_PI / 180;
	constexpr double TO_FRONT = 68;
}

namespace Wheel
{
	constexpr double DIAMETER_IN = 6;
	constexpr double DIAMETER = DIAMETER_IN * 2.54 / 100;
	constexpr double CIRCUMFERENCE = DIAMETER * M_PI;
	constexpr double RADIUS = DIAMETER / 2;
	constexpr double GEAR_RATIO = 10.71;
	constexpr auto TRACK_WIDTH = 0.60775_m;
}

namespace DPR
{
	constexpr double ENCODER_PULSE = 2048;
	constexpr double DRIVETRAIN = Wheel::CIRCUMFERENCE / ENCODER_PULSE;
	constexpr double SHOULDER = 1;
	constexpr double ELBOW = 1;
	constexpr double WRIST = 1;
}

namespace Speed
{
	constexpr double DRIVETRAIN_MOVE = 1;
	constexpr double DRIVETRAIN_TURN = 1;
	constexpr double SHOULDER = 1;
	constexpr double ELBOW = 1;
	constexpr double WRIST = 1;
	constexpr double INTAKE = 1;
}


using Velocity =
	units::compound_unit<units::meters, units::inverse<units::seconds>>;
using Acceleration =
	units::compound_unit<Velocity, units::inverse<units::seconds>>;
using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
using ka_unit =
	units::compound_unit<units::volts, units::inverse<Acceleration>>;
using b_unit =
	units::compound_unit<units::squared<units::radians>,
						 units::inverse<units::squared<units::meters>>>;
using zeta_unit = units::inverse<units::radians>;

namespace Path
{
	constexpr units::unit_t<Velocity> MAX_SPEED = 2_mps;
	constexpr units::unit_t<Acceleration> MAX_ACCELERATION = 3.0_mps_sq;

	constexpr units::unit_t<b_unit> RAMSETE_B = 2 * 0 * 1_rad * 1_rad / (1_m * 1_m);
	constexpr units::unit_t<zeta_unit> RAMSETE_ZETA = 0.7 / 1_rad;

	constexpr auto KS = 0.88469_V;
	constexpr units::unit_t<kv_unit> KV = 3.0056 * 1_V * 1_s / 1_m;
	constexpr units::unit_t<ka_unit> KA = 0.59519 * 1_V * 1_s * 1_s / 1_m;

	constexpr double KP = 0.05;
}

constexpr double shooterRPMFromDistance(double distance)
{
	return 60; // TODO: place RPM function here
}
