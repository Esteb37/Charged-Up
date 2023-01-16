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

#include "subsystems/EncoderSubsystemBase.h"

namespace TD
{
	template <class MotorType, class EncoderType>
	class ElevatorBase : public EncoderSubsystemBase<MotorType, EncoderType>
	{
	public:
		/**
		 * @brief Construct a new ElevatorBase object without encoder and a single motor
		 * @param motorPort The CAN ID of the motor
		 */
		ElevatorBase(unsigned int motorPort);

		/**
		 * @brief Construct a new ElevatorBase object without encoder and various motors
		 * @param motorPorts The CAN IDs of the motors
		 */
		ElevatorBase(vector<unsigned int>);

		/**
		 * @brief Construct a new ElevatorBase object with an FRC encoder and a single motor
		 * @param motorPort The CAN ID or PWM port of the motor
		 * @param encoderA The A port of the encoder
		 * @param encoderB The B port of the encoder
		 */
		ElevatorBase(unsigned int, unsigned int, unsigned int);

		/**
		 * @brief Construct a new ElevatorBase object with an FRC encoder and various motors
		 * @param motorPorts The CAN IDs or PWM ports of the motors
		 * @param encoderA The A port of the encoder
		 * @param encoderB The B port of the encoder
		 */
		ElevatorBase(vector<unsigned int>, unsigned int, unsigned int);

		void Periodic() override;

		// ---------- Actions -----------

		/**
		 * @brief Moves the elevator
		 * @param speed Speed and direction to move
		 */
		void Move(double);

		/**
		 * @brief Sets the offset of the elevator height relative to the floor
		 * @param height The height offset
		 */
		void SetHeightToFloor(double);

		/**
		 * @brief Gets the height of the elevator relative to itself
		 *
		 * @return The relative height
		 */
		double GetRelativeHeight();

		/**
		 * @brief Gets the height of the elevator relative to the floor
		 *
		 * @return The height of the elevator relative to the floor
		 */
		double GetAbsoluteHeight();

		/**
		 * @brief Sets the height of the elevator relative to itself
		 * @param height The height to set
		 * @param speed The speed to move at
		 * @return True if the elevator is at the specified height
		 */
		bool SetRelativeHeight(double, double);

		/**
		 * @brief Sets the height of the elevator relative to the floor
		 * @param height The height to set
		 * @param speed The speed to move at
		 * @return True if the elevator is at the specified height
		 */
		bool SetAbsoluteHeight(double, double);

		/**
		 * @brief Sets the minimum and maximum height for safety
		 * @param min The minimum height
		 * @param max The maximum height
		 */
		void SetMinMaxHeight(double, double);

	protected:
		double m_heightToFloor;
	};
}