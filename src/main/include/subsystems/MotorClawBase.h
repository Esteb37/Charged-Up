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

#include "subsystems/MotorSubsystemBase.h"

namespace TD
{

	class MotorClawBase : public SubsystemBase
	{

	public:
		/**
		 * @brief Construct a new Claw object with no motors attached
		 */
		MotorClawBase();

		static MotorClawBase &GetInstance();

		/**
		 * @brief Construct a new Claw object with a single motor
		 *
		 * @param config The motor type
		 * @param motorPort
		 */
		void Initialize(MotorConfig, unsigned int);

		/**
		 * @brief Construct a new Claw object with a claw motor and a wrist motor
		 *
		 * @param config The motor type
		 * @param motorClawPort The claw motor port
		 * @param motorWristPort The wrist motor port
		 */
		void Initialize(MotorConfig, unsigned int, unsigned int);

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

		// ---------- Hand Motor --------

		/**
		 * @brief Inverts open and close status
		 * @param invert True to invert, false to not
		 */
		void InvertHand(bool);

		/**
		 * @brief Get the hand motor status
		 * @return the motor status
		 */
		unsigned int GetHand();

		/**
		 * @brief Publishes the hand motor status to the dashboard
		 */
		void PrintHand();

		// ---------- Wrist Motor --------

		/**
		 * @brief Inverts low and up status
		 * @param invert True to invert, false to not
		 */
		void InvertWrist(bool);

		/**
		 * @brief Get the wrist motor status
		 * @return the motor status
		 */
		unsigned int GetWrist();

		/**
		 * @brief Publishes the wrist motor status to the dashboard
		 */
		void PrintWrist();

		// ---------- Components ----------
		MotorSubsystemBase m_hand;
		MotorSubsystemBase m_wrist;

	protected:
		bool m_wristLowered = false;
		bool m_handOpen = false;
	};
}