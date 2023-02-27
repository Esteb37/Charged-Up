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

#include "Constants.h"
#include "subsystems/MotorSubsystemBase.h"
#include <frc2/command/Commands.h>
namespace TD
{
	class Intake : virtual public MotorSubsystemBase<MotorTypes::SPARK>
	{
	public:
		/**
		 * @brief Construct a new Intake object without encoder and a single motor
		 * @param motorPort The CAN ID of the motor
		 */
		Intake(unsigned int motorPort);

		/**
		 * @brief Construct a new Intake object without encoder and various motors
		 * @param motorPorts The CAN IDs of the motors
		 */
		Intake(vector<unsigned int>);

		void Periodic() override;

		// ---------- Actions -----------

		void Take();

		void Spit(double speed = 1);

		void Invert(bool invert = true);

		CommandPtr SpitCmd(double speed = 1);

		CommandPtr TakeCmd(double speed = 1);

		void Stop();

	private:
		Timer m_spitTimer = frc::Timer();
	};
}