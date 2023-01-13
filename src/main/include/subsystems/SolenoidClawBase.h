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

#include "subsystems/SolenoidSubsystemBase.h"

namespace TD
{

	class SolenoidClawBase : public SubsystemBase
	{

	public:
		/**
		 * @brief Construct a new Claw object with no solenoids attached
		 */
		SolenoidClawBase();

		static SolenoidClawBase &GetInstance();

		/**
		 * @brief Construct a new Claw object with a single solenoid
		 *
		 * @param solenoidForward The solenoid forward port
		 * @param solenoidReverse The solenoid reverse port
		 */
		void Initialize(unsigned int, unsigned int);

		/**
		 * @brief Construct a new Claw object with hand and wrist solenoids
		 *
		 * @param solenoidHandForward The hand solenoid forward port
		 * @param solenoidHandReverse The hand solenoid reverse port
		 * @param solenoidWristForward The wrist solenoid forward port
		 * @param solenoidWristReverse The wrist solenoid reverse port
		 */
		void Initialize(unsigned int, unsigned int, unsigned int, unsigned int);

		void Periodic() override;

		// ---------- Actions -----------

		/**
		 * @brief Opens the claw
		 */
		void OpenHand();

		/**
		 * @brief Closes the claw
		 */
		void CloseHand();

		/**
		 * @brief Toggles between open and closed claw
		 */
		void ToggleHand();

		/**
		 * @brief Lowers the claw
		 */
		void LowerWrist();

		/**
		 * @brief Raises the claw
		 */
		void RaiseWrist();

		/**
		 * @brief Toggles between low and up claw
		 */
		void ToggleWrist();

		// ---------- Hand Solenoid --------

		/**
		 * @brief Inverts open and close status
		 * @param invert True to invert, false to not
		 */
		void InvertHand(bool);

		/**
		 * @brief Turns the hand solenoid off
		 */
		void HandOff();

		/**
		 * @brief Get the hand solenoid status
		 * @return the solenoid status
		 */
		unsigned int GetHand();

		/**
		 * @brief Publishes the hand solenoid status to the dashboard
		 */
		void PrintHand();

		// ---------- Wrist Solenoid --------

		/**
		 * @brief Inverts low and up status
		 * @param invert True to invert, false to not
		 */
		void InvertWrist(bool);

		/**
		 * @brief Turns the wrist solenoid off
		 */
		void WristOff();

		/**
		 * @brief Get the wrist solenoid status
		 * @return the solenoid status
		 */
		unsigned int GetWrist();

		/**
		 * @brief Publishes the wrist solenoid status to the dashboard
		 */
		void PrintWrist();

		// ---------- Components ----------
		SolenoidSubsystemBase m_hand;
		SolenoidSubsystemBase m_wrist;

	protected:
		bool m_wristLowered = false;
		bool m_handOpen = false;
	};
}