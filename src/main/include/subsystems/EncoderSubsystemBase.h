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
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "subsystems/MotorSubsystemBase.h"

namespace TD
{

	namespace EncoderTypes
	{
		typedef Encoder CLASSIC;
		typedef SparkMaxRelativeEncoder NEO;
	}

	template <class MotorType, class EncoderType>
	class EncoderSubsystemBase : virtual public MotorSubsystemBase<MotorType>
	{
	public:
		/**
		 * @brief Construct a new EncoderSubsystemBase object without encoder and a single motor
		 * @param motorPort The CAN ID of the motor
		 * @param isBrushless
		 */
		EncoderSubsystemBase(unsigned int motorPort, bool isBrushless = true);

		/**
		 * @brief Construct a new EncoderSubsystemBase object without encoder and various motors
		 * @param motorPorts The CAN IDs of the motors
		 */
		EncoderSubsystemBase(vector<unsigned int>, bool isBrushless = true);

		/**
		 * @brief Construct a new EncoderSubsystemBase object with an FRC encoder and a single motor
		 * @param motorPort The CAN ID or PWM port of the motor
		 * @param encoderA The A port of the encoder
		 * @param encoderB The B port of the encoder
		 */
		EncoderSubsystemBase(unsigned int, unsigned int, unsigned int);

		/**
		 * @brief Construct a new EncoderSubsystemBase object with an FRC encoder and various motors
		 * @param motorPorts The CAN IDs or PWM ports of the motors
		 * @param encoderA The A port of the encoder
		 * @param encoderB The B port of the encoder
		 */
		EncoderSubsystemBase(vector<unsigned int>, unsigned int, unsigned int);

		void Periodic() override;

		/**
		 * @brief Reset encoder and PIDs
		 */
		void Reset();

		// ---------- Motors -----------

		/**
		 * @brief Set the motor speed with safety considered
		 * @param speed The speed to set the motor to
		 */
		void SetMotor(double);

		/**
		 * @brief Set all motors to the specified speed
		 * @param speed The speed to set the motors to
		 */
		void SetMotors(double);

		/**
		 * @brief Set speeds of each motor with safety considered
		 * @param speeds The speeds to set the motors to
		 */
		void SetMotors(vector<double>);

		void SetAngleSetpoint(units::angle::degree_t);
		bool ReachAngle(double);

		units::degree_t GetAngle();

		// ---------- Encoder ----------

		/**
		 * @brief Sets the encoder's value to 0
		 */
		void ResetEncoder();

		/**
		 * @brief Inverts the encoder's direction
		 * @param invert True to invert, false to not
		 */
		void InvertEncoder(bool);

		// ---------- Position PID -----------

		/**
		 * @brief Set the subsystem to a specified position
		 * @param position The position to set the subsystem to
		 * @param speed The speed at which the position will be set
		 * @return True if the subsystem is at the position, false otherwise
		 */
		bool SetPosition(double, double);

		/**
		 * @brief Get the encoder's position
		 * @return The encoder's position
		 */
		double GetPosition();

		/**
		 * @brief Configures the position's PID values, tolerance and direction
		 * @param p The proportional value
		 * @param i The integral value
		 * @param d The derivative value
		 * @param tolerance The tolerance of the position
		 * @param inverted Invert PID calculation
		 */
		void ConfigurePositionPID(double, double, double, double, bool = false);

		/**
		 * @brief Set the encoder's Position Conversion Factor
		 * @param conversionFactor The conversion factor
		 */
		void SetPositionConversionFactor(double);

		/**
		 * @brief Resets the position PID
		 */
		void ResetPositionPID();

		/**
		 * @brief Prints encoder's position
		 */
		void PrintPosition();

		/**
		 * @brief Prints position PID error
		 */
		void PrintPositionError();

		/**
		 * @brief Enables safety based on encoder position
		 *
		 */
		void SetPositionSafety(bool);

		/**
		 * @brief Sets the minimum and maximum position for safety
		 */
		void SetMinMaxPosition(double, double);

		// ---------- Speed PID -----------

		/**
		 * @brief Set the subsystem to a specified speed
		 * @param rpm The velocity to set the subsystem to
		 * @param acceleration The acceleration at which the speed will be set
		 * @return True if the subsystem is at the speed, false otherwise
		 */
		bool SetRPM(double, double);

		/**
		 * @brief Get the encoder's rate
		 * @return The encoder's rate
		 */
		double GetRPM();

		/**
		 * @brief Configures the RPM's PID values, tolerance and direction
		 * @param p The proportional value
		 * @param i The integral value
		 * @param d The derivative value
		 * @param tolerance The tolerance of the RPM
		 * @param inverted Invert PID calculation
		 */
		void ConfigureRPMPID(double, double, double, double, bool = false);

		/**
		 * @brief Set the encoder's RPM Conversion Factor
		 * @param conversionFactor The conversion factor
		 */
		void SetRPMConversionFactor(double);

		/**
		 * @brief Resets the RPM PID
		 */
		void ResetRPMPID();

		/**
		 * @brief Prints encoder's RPM
		 */
		void PrintRPM();

		/**
		 * @brief Prints RPM PID error
		 */
		void PrintRPMError();


		// ---------- Commands ---------- 

		frc2::CommandPtr MatchDegrees(units::angle::degree_t, double);
		frc2::CommandPtr GotoZeroDegrees(double);


		// ---------- Elements ----------

		EncoderType *m_encoder;

		PIDController m_positionPID{0.1, 0, 0};

		PIDController m_RPMPID{0.1, 0, 0};

	protected:
		int m_positionPIDDirection = 1;

		int m_RPMPIDDirection = 1;

		double m_maxPosition = 100000;

		double m_minPosition = -100000;

		double m_maxRPM = 100000;

		bool m_keepingStill = false;

		bool m_positionSafetyActive = false;
	};
}
