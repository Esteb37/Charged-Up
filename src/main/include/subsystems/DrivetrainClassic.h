/*------------------------------------------------------------
						&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&,
					.&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&&&&&%
				.&&&&/ &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&&&&&&&&*
			.%&&&(      &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&% %&&&&&&&&&&&&&
		%&&&#     %#&&#           (&&&&&&&&&&&              %&&&&&&&&&&&&&
	*&&&#                          (&&&&&&&&&&&    /           %&&&&&&&&&&&
*&%  ,                           (&&&&&&&&&&&(&&&&(           &&&&&&&&&&&
& (,..                          (&&&&&&&&&&&&&&&&            %&&&&&&&&&&
	&*                             (&&&&&&&&&&&&&&&&            &&&&&&&&&&&
	&/                             (&&&&&&&&&&&&&&&&%          &&&&&&&&&&&(
	#&&    .&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&#**(&&&&&&&&&&&&&#
	&#  (&                        ......... &&&&&&&&&&&&&&&&&&&&&&&&&&
	/&   &                                   .&&&&&&&&&&&&&&&&&&&&&&
		%&&* &*                                   ,%&&&&&&&&&&&&&%*

	Author: Esteban Padilla Cerdio
	Email: esteban37padilla@gmail.com
	URL: github.com/esteb37
		 github.com/tecdroid-3354
	Date: 12/04/2022
	Language: cpp
	Copyright (c) TecDroid 3354 and Esteban Padilla Cerdio
	Open Source Software; you can modify and/or share it under the terms of
*/

#pragma once

#include <frc/Encoder.h>
#include <frc/motorcontrol/VictorSP.h>

#include "subsystems/Drivetrain.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace frc2;
using namespace frc;
using namespace rev;
using namespace std;

namespace TD
{
	class DrivetrainClassic : public Drivetrain
	{
	public:
		DrivetrainClassic() = default;

		static DrivetrainClassic &GetInstance();

		/**
		 * @brief Construct a new Drivetrain object
		 * @param frontRight The CAN ID of the front right motor
		 * @param frontLeft The CAN ID of the front left motor
		 * @param backRight The CAN ID of the back right motor
		 * @param backLeft The CAN ID of the back left motor
		 */
		void Initialize(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft, unsigned int encoderRightA, unsigned int encoderRightB, unsigned int encoderLeftA, unsigned int encoderLeftB);

		// --------------------- Encoders ---------------------

		/**
		 * @brief Gets the average of the right encoder
		 * @return double average
		 */
		double GetRightEncoder();

		/**
		 * @brief Gets the average of the left encoder
		 * @return double average
		 */
		double GetLeftEncoder();

		double GetRightEncoderTotal();

		double GetLeftEncoderTotal();

		/**
		 * @brief Resets the encoders to 0
		 */
		void ResetEncoders();

		/**
		 * @brief Invert the direction of the right encoder
		 * @param invert True to invert, false to not
		 */
		void InvertRightEncoder(bool);

		/**
		 * @brief Invert the direction of the left encoder
		 * @param invert True to invert, false to not
		 */
		void InvertLeftEncoder(bool);

		/**
		 * @brief Publish the value of the encoders to the dashboard
		 */
		void PrintEncoders();

		/**
		 * @brief Sets the encoders' distance per revolution
		 * @param pcf Position conversion factor
		 */
		void SetPositionConversionFactor(double);

		double GetEncodersAverage();

		// ----- Odometry -----

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

		// ----- Motors -----

		VictorSP *m_frontLeft;

		VictorSP *m_frontRight;

		VictorSP *m_backLeft;

		VictorSP *m_backRight;

		// ----- Sensors -----

		Encoder *m_rightEncoder;

		Encoder *m_leftEncoder;

	protected:
		// ----- Attributes -----s

		int m_rightEncoderDirection = 1;

		int m_leftEncoderDirection = 1;

		double m_rightEncoderTotal = 0;

		double m_leftEncoderTotal = 0;

		// ---- Kinematics ----

		DifferentialDriveOdometry m_odometry{
			GetRotation2d(),
			units::meter_t{GetLeftEncoder()},
			units::meter_t{GetRightEncoder()},
			frc::Pose2d{0_m, 0_m, 0_rad}};
	};
}