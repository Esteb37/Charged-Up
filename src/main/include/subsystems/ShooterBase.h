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

#include <frc/Servo.h>

#include "subsystems/EncoderSubsystemBase.h"
#include "subsystems/Limelight.h"
#include "subsystems/SolenoidSubsystemBase.h"

namespace TD
{
	class ShooterBase : public EncoderSubsystemBase, public SolenoidSubsystemBase
	{

	public:
		ShooterBase();

		static ShooterBase &GetInstance();

		using EncoderSubsystemBase::Initialize;

		void ConfigureSolenoid(unsigned int, unsigned int);

		void ConfigureSolenoids(unsigned int, unsigned int, unsigned int, unsigned int);

		void Periodic() override;

		/**
		 * @brief Resets the encoder and speed PID
		 */
		void Reset();

		// ---------- Actions -----------

		/**
		 * @brief Sets shooter to full speed
		 */
		void Shoot();

		// --------- Angle with Servos --------

		/**
		 * @brief Sets the shooting angle by using servos
		 * @param angle The angle to set the shooter to
		 */
		void SetAngleServos(double);

		/**
		 * @brief Configure angle servo's ports
		 *
		 * @param rightServo The right servo port
		 * @param leftServo The left servo port
		 */
		void ConfigureServos(unsigned int, unsigned int);

		/**
		 * @brief Resets the angle servos to 0
		 */
		void ResetServos();

		/**
		 * @brief Gets the length of the right servo
		 * @return The length of the right servo
		 */
		double GetRightServo();

		/**
		 * @brief Gets the length of the left servo
		 * @return The length of the left servo
		 */
		double GetLeftServo();

		/**
		 * @brief Prints the length of the servos to the dashboard
		 */
		void PrintServos();

		/**
		 * @brief Calculates the required RPM to shoot at the current distance
		 * @param targetHeight THe height of the shooting target marked with reflective tape
		 * @return The required RPM
		 */
		double CalculateRPM(double);

		/**
		 * @brief Sets the function that will be used to calculate rpm according to distance
		 *
		 * @param func The function to set
		 */
		void SetRPMCalculationFunction(double (*func)(double));

		// ---------- Components ----------

		Servo *m_rightServo;

		Servo *m_leftServo;

	protected:
		double (*m_rpmCalculationFunction)(double);

		Limelight m_limelight = Limelight::GetInstance();
	};
}