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
#include <math.h>
#include <units/length.h>

using port = unsigned int;

// MOTORS
namespace M
{
	namespace CAN
	{
		constexpr port SHOOTER = 4;
		constexpr port ELEVATOR_RIGHT = 5;
		constexpr port ELEVATOR_LEFT = 6;
		constexpr port INTAKE = 7;
	}

	namespace PWM
	{
		constexpr port FRONT_RIGHT = 0;
		constexpr port FRONT_LEFT = 1;
		constexpr port BACK_RIGHT = 2;
		constexpr port BACK_LEFT = 3;
		constexpr port FEEDER = 4;
		constexpr port TURRET = 5;
	}

	namespace Servo
	{
		constexpr port SHOOTER_RIGHT = 2;
		constexpr port SHOOTER_LEFT = 3;
	}
}

namespace Solenoid
{
	constexpr port CLAW_FORWARD = 0;
	constexpr port CLAW_REVERSE = 1;
	constexpr port WRIST_FORWARD = 2;
	constexpr port WRIST_REVERSE = 3;
	constexpr port INTAKE_RIGHT_FORWARD = 4;
	constexpr port INTAKE_RIGHT_REVERSE = 5;
	constexpr port INTAKE_LEFT_FORWARD = 6;
	constexpr port INTAKE_LEFT_REVERSE = 7;
}

namespace DIO
{
	namespace Encoder
	{
		constexpr port SHOOTER_A = 0;
		constexpr port SHOOTER_B = 1;
		constexpr port TURRET_A = 2;
		constexpr port TURRET_B = 3;
		constexpr port ELEVATOR_A = 6;
		constexpr port ELEVATOR_B = 7;
		constexpr port DRIVETRAIN_RA = 0;
		constexpr port DRIVETRAIN_RB = 1;
		constexpr port DRIVETRAIN_LA = 2;
		constexpr port DRIVETRAIN_LB = 3;

	}

	namespace Limit
	{
		constexpr port TURRET_LEFT = 4;
		constexpr port TURRET_RIGHT = 5;
		constexpr port ELEVATOR_TOP = 8;
		constexpr port ELEVATOR_BOTTOM = 9;
	}
}

namespace PID
{
	namespace Move
	{
		constexpr double P = 30;
		constexpr double I = 10;
		constexpr double D = 0;
		constexpr double TOLERANCE = 0.01;
	}

	namespace Turn
	{
		constexpr double P = 0.1;
		constexpr double I = 1;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
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

	namespace Shooter
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

	namespace Elevator
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace PlanarElevator
	{
		constexpr double P_X = 0.01;
		constexpr double I_X = 0.0;
		constexpr double D_X = 0.0;
		constexpr double TOLERANCE_X = 0.5;

		constexpr double P_Y = 0.01;
		constexpr double I_Y = 0.0;
		constexpr double D_Y = 0.0;
		constexpr double TOLERANCE_Y = 0.5;

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
	constexpr auto TRACK_WIDTH = 0.55_m;
	constexpr double ENCODER_PULSE = 48;
}

namespace DPR
{
	constexpr double DRIVETRAIN = Wheel::CIRCUMFERENCE / Wheel::ENCODER_PULSE;
	constexpr double TURRET = 360;	 // TODO Define turret encoder to angle ratio
	constexpr double ELEVATOR = 180; // TODO Define elevator distance per revolution
	constexpr double SHOOTER = 180;	 // TODO Define shooter conversion factor
}

namespace Speed
{
	constexpr double DRIVETRAIN_MOVE = 1;
	constexpr double DRIVETRAIN_TURN = 1;
	constexpr double SHOOTER = 1;
	constexpr double TURRET = 1;
	constexpr double FEEDER = 1;
	constexpr double ELEVATOR = 1;
	constexpr double PLANAR_ELEVATOR = 1;
}

namespace Time
{
	constexpr double SHOOTER_LOAD = 2;
	constexpr double FEEDER = 5;
}

namespace Minmax
{
	constexpr double TURRET_MIN = 0;
	constexpr double TURRET_MAX = 360;
	constexpr double ELEVATOR_MIN = 250;
	constexpr double ELEVATOR_MAX = 30;
}

constexpr double shooterRPMFromDistance(double distance)
{
	return 60; // TODO: place RPM function here
}
