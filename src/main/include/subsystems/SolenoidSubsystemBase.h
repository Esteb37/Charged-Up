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

#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

using namespace frc;
using namespace frc2;

namespace TD
{

	class SolenoidSubsystemBase : virtual public SubsystemBase
	{

	public:
		/**
		 * @brief Construct a new SolenoidSubsystemBase object with no solenoids attached
		 */
		SolenoidSubsystemBase();

		static SolenoidSubsystemBase &GetInstance();

		/**
		 * @brief Construct a new SolenoidSubsystemBase object with a single solenoid
		 *
		 * @param solenoidForward The solenoid forward port
		 * @param solenoidReverse The solenoid reverse port
		 */
		void Initialize(int, int);

		/**
		 * @brief Construct a new SolenoidSubsystemBase object with two solenoids
		 *
		 * @param rightSolenoidForward The right solenoid forward port
		 * @param rightSolenoidReverse The left solenoid reverse port
		 * @param leftSolenoidForward The left solenoid forward port
		 * @param leftSolenoidReverse The left solenoid reverse port
		 */
		void Initialize(int, int, int, int);

		void Periodic() override;

		// ---------- Solenoids -----------

		/**
		 * @brief Set the solenoid ports
		 *
		 * @param forward The forward port
		 * @param reverse The reverse port
		 */
		void ConfigureSolenoid(int, int);

		/**
		 * @brief Set the solenoid ports
		 *
		 * @param rightForward The right forward port
		 * @param rightReverse The right reverse port
		 * @param leftForward The left forward port
		 * @param leftReverse The left reverse port
		 */
		void ConfigureSolenoids(int, int, int, int);

		/**
		 * @brief Invert which value is up or down
		 */
		void InvertSolenoids(bool invert);

		/**
		 * @brief Sets the solenoids to kOFF
		 */
		void SolenoidsOff();

		/**
		 * @brief Sets the solenoids to kForward
		 */
		void OpenSolenoids();

		/**
		 * @brief Sets the solenoids to kReverse
		 */
		void CloseSolenoids();

		/**
		 * @brief Toggles between forward and reverse
		 */
		void ToggleSolenoids();

		/**
		 * @brief Get the right solenoid state
		 */
		unsigned int GetRightSolenoid();

		/**
		 * @brief Get the left solenoid state
		 */
		unsigned int GetLeftSolenoid();

		/**
		 * @brief Get the single solenoid state
		 */
		unsigned int GetSolenoid();

		/**
		 * @brief Publishes the solenoids' value on the SmartDashboard
		 */
		void PrintSolenoids();

		DoubleSolenoid *m_rightSolenoid;

		DoubleSolenoid *m_leftSolenoid;

	protected:
		bool m_isDoubleSolenoid = false;

		bool m_solenoidsInverted = false;

		bool m_isOpen = false;
	};
}
