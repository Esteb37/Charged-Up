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
	constexpr port TURRET = 7;

	namespace Arm
	{
		constexpr port SHOULDER = 8;
		constexpr port ELBOW = 1;
		constexpr port WRIST = 6;
	}

	namespace DT
	{
		constexpr port FRONT_LEFT = 10;
		constexpr port FRONT_RIGHT = 13;
		constexpr port BACK_RIGHT = 12;
		constexpr port BACK_LEFT = 11;
	}

	namespace Intake
	{
		constexpr port RIGHT = 5;
		constexpr port LEFT = 4;
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
	// TODO : Tune PIDs
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
		constexpr double P = 0.05;
		constexpr double I = 0.0001;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Shoulder
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Elbow
	{
		constexpr double P = 0.01;
		constexpr double I = 0.0;
		constexpr double D = 0.0;
		constexpr double TOLERANCE = 0.5;
	}

	namespace Wrist
	{
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

	// TODO : Find encoder pulses per revolution
	constexpr double SHOULDER = 360 / (100.0 * 2.9);
	constexpr double ELBOW =    360 / (100.0 * 2.9);
	constexpr double WRIST =    360 / (100.0);
	constexpr double TURRET =   360 / (64.0  * 4.5);
}

namespace Speed
{
	// TODO : Tune speeds
	constexpr double DRIVETRAIN_MOVE = 1.0;
	constexpr double DRIVETRAIN_TURN = 1.0;
	constexpr double SHOULDER =  0.45;
	constexpr double ELBOW =     0.45;
	constexpr double WRIST =     0.45;
	constexpr double INTAKE =    0.45;
	constexpr double TURRET =    1;
	constexpr double BOX_SPIT =  0.45;
	constexpr double CONE_SPIT = 0.45;
}

namespace Angles
{
	// TODO : Calculate angles for each position
	namespace Shoulder
	{
		constexpr auto HOME = 8_deg;
		constexpr auto PICKUP = 15_deg;
		constexpr auto CONE_LOW = -5_deg;
		constexpr auto CONE_MIDDLE = 45_deg;
		constexpr auto CONE_HIGH = 93.94_deg;
		constexpr auto BOX_LOW = -5_deg;
		constexpr auto BOX_MIDDLE = 10_deg;
		constexpr auto BOX_HIGH = 52.92_deg;
		constexpr auto TRAY = 30.2_deg;
		constexpr auto TAXI = 10_deg;
	}

	namespace Elbow
	{
		constexpr auto HOME = 20_deg;
		constexpr auto PICKUP = 139.51_deg;
		constexpr auto CONE_LOW = 75_deg;
		constexpr auto CONE_MIDDLE = 92.6_deg;
		constexpr auto CONE_HIGH = 170_deg;
		constexpr auto BOX_LOW = 210_deg;
		constexpr auto BOX_MIDDLE = 63.97_deg;
		constexpr auto BOX_HIGH = 101.88_deg;
		constexpr auto TRAY = 43.03_deg;
		constexpr auto TAXI = 20_deg;
	}

	namespace Wrist
	{
		constexpr auto HOME = 30_deg;
		constexpr auto PICKUP = 145.49_deg;
		constexpr auto CONE_LOW = 210_deg;
		constexpr auto CONE_MIDDLE = 222.4_deg;
		constexpr auto CONE_HIGH = 193.94_deg;
		constexpr auto BOX_LOW = 221.03_deg;
		constexpr auto BOX_MIDDLE = 216.03_deg;
		constexpr auto BOX_HIGH = 221.03_deg;
		constexpr auto TRAY = 257.9_deg;
		constexpr auto TAXI = 330_deg;
	}
}

namespace Times
{
	// TODO : Check if this time is correct
	constexpr auto SPIT = 1_s;
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

namespace Positions
{
	struct Position
	{
		units::unit_t<units::meters> x;
		units::unit_t<units::meters> y;
		units::unit_t<units::radians> theta;
	};

	constexpr Position INITIAL = Position{0_m, 0_m, 0_rad};

	namespace PLACE
	{
		constexpr auto CONE_RR = Position{0_m, 0_m, 0_rad};
		constexpr auto CONE_RL = Position{0_m, 0_m, 0_rad};
		constexpr auto CONE_MR = Position{0_m, 0_m, 0_rad};
		constexpr auto CONE_ML = Position{0_m, 0_m, 0_rad};
		constexpr auto CONE_LR = Position{0_m, 0_m, 0_rad};
		constexpr auto CONE_LL = Position{0_m, 0_m, 0_rad};
		constexpr auto BOX_R = Position{0_m, 0_m, 0_rad};
		constexpr auto BOX_M = Position{0_m, 0_m, 0_rad};
		constexpr auto BOX_L = Position{0_m, 0_m, 0_rad};
	}

	namespace PICK
	{
		constexpr auto ITEM_1 = Position{0_m, 0_m, 0_rad};
		constexpr auto ITEM_2 = Position{0_m, 0_m, 0_rad};
		constexpr auto ITEM_3 = Position{0_m, 0_m, 0_rad};
		constexpr auto ITEM_4 = Position{0_m, 0_m, 0_rad};
	}

	namespace PREPARE
	{
		constexpr auto PICK_ITEM_1 = Position{0_m, 0_m, 0_rad};
		constexpr auto BALANCE = Position{0_m, 0_m, 0_rad};
	}

	constexpr auto BALANCE = Position{0_m, 0_m, 0_rad};

}
constexpr double shooterRPMFromDistance(double distance)
{
	return 60; // TODO: place RPM function here
}
