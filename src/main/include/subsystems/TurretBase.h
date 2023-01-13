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

#include <frc/controller/PIDController.h>

#include "subsystems/EncoderSubsystemBase.h"
#include "subsystems/Limelight.h"

namespace TD
{
	class TurretBase : public EncoderSubsystemBase
	{
	public:
		TurretBase();

		static TurretBase &GetInstance();

		void Periodic() override;

		// ---------- Actions -----------

		/**
		 * @brief Resets encoder and PID controllers
		 */
		void Reset();

		/**
		 * @brief Keeps the turret at the last angle
		 */
		void KeepStill(double);

		/**
		 * @brief Turns the turret
		 * @param speed Speed and direction to turn
		 */
		void Turn(double);

		/**
		 * @brief Sets the turret to angle 0
		 * @param speed Speed and direction to turn
		 * @return True if the turret is centered
		 */
		bool Center(double);

		// ---------- Angle ---------

		/**
		 * @brief Sets the turret to a specific angle
		 * @param angle The angle to set the turret to
		 * @param speed The speed to turn the turret at
		 * @return Has the turret reached the angle
		 */
		bool SetAngle(double, double);

		/**
		 * @brief Gets the turret's current angle
		 * @return The turret's current angle
		 */
		double GetAngle();

		/**
		 * @brief Configure angle auto PID's values, tolerance and direction
		 * @param p The proportional value
		 * @param i The integral value
		 * @param d The derivative value
		 * @param tolerance The tolerance value
		 * @param inverted Is the PID inverted
		 */
		void ConfigureAnglePID(double, double, double, double, bool = false);

		/**
		 * @brief Prints the turret's angle
		 */
		void PrintAngle();

		/**
		 * @brief Prints the turret's angle PID error
		 */
		void PrintAnglePIDError();

		// --------- Align ---------

		/**
		 * @brief Aligns the turret to the target
		 * @param speed The speed to turn the turret at
		 * @return Has the turret reached the target
		 */
		bool Align(double);

		/**
		 * @brief Resets the PID controller for aligning to target
		 */
		void ResetAlignPID();

		/**
		 * @brief Configure align PID's values, tolerance and direction
		 * @param p The proportional value
		 * @param i The integral value
		 * @param d The derivative value
		 * @param tolerance The tolerance value
		 * @param inverted Is the PID inverted
		 */
		void ConfigureAlignPID(double, double, double, double, bool = false);

		/**
		 * @brief Publishes the alignPID error on the SmartDashboard
		 */
		void PrintAlignPIDError();

		/**
		 * @brief Sets the turret's freedom of revolution
		 *
		 */
		void SetFreedom(double);

		// ---------- Components ----------

		PIDController m_alignPID{0.1, 0, 0};

	protected:
		Limelight m_limelight = Limelight::GetInstance();

		// ---------- Attributes ----------

		double m_angle;

		bool m_keepingStill = false;

		bool m_limitSafety = false;

		bool m_angleSafety = false;

		double m_freedom = 360;

		int m_anglePIDDirection = 1;

		int m_alignPIDDirection = 1;
	};
}