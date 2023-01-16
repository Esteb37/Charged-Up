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

#include "subsystems/ElevatorBase.h"

using namespace TD;

template <class MotorType, class EncoderType>
ElevatorBase<MotorType, EncoderType>::ElevatorBase(unsigned int motorPort) : EncoderSubsystemBase<MotorType, EncoderType>(motorPort)
{
	SubsystemBase::SetName("Elevator");
}

template <class MotorType, class EncoderType>
ElevatorBase<MotorType, EncoderType>::ElevatorBase(unsigned int motorPort, unsigned int encoderA, unsigned int encoderB) : EncoderSubsystemBase<MotorType, EncoderType>(motorPort, encoderA, encoderB)
{
	SubsystemBase::SetName("Elevator");
}

template <class MotorType, class EncoderType>
ElevatorBase<MotorType, EncoderType>::ElevatorBase(vector<unsigned int> motorPorts) : EncoderSubsystemBase<MotorType, EncoderType>(motorPorts)
{
	SubsystemBase::SetName("Elevator");
}

template <class MotorType, class EncoderType>
ElevatorBase<MotorType, EncoderType>::ElevatorBase(vector<unsigned int> motorPorts, unsigned int encoderA, unsigned int encoderB) : EncoderSubsystemBase<MotorType, EncoderType>(motorPorts, encoderA, encoderB)
{
	SubsystemBase::SetName("Elevator");
}

template <class MotorType, class EncoderType>
void ElevatorBase<MotorType, EncoderType>::Periodic()
{
}

// ---------- Actions -----------

template <class MotorType, class EncoderType>
void ElevatorBase<MotorType, EncoderType>::Move(double speed)
{
	if (MotorSubsystemBase<MotorType>::m_motorCount > 1)
	{
		vector<double> speeds(MotorSubsystemBase<MotorType>::m_motorCount, speed);
		MotorSubsystemBase<MotorType>::SetMotors(speeds);
	}

	else
		MotorSubsystemBase<MotorType>::SetMotor(speed);
}

template <class MotorType, class EncoderType>
void ElevatorBase<MotorType, EncoderType>::SetHeightToFloor(double height)
{
	m_heightToFloor = height;
}

template <class MotorType, class EncoderType>
double ElevatorBase<MotorType, EncoderType>::GetRelativeHeight()
{
	return EncoderSubsystemBase<MotorType, EncoderType>::GetPosition();
}

template <class MotorType, class EncoderType>
double ElevatorBase<MotorType, EncoderType>::GetAbsoluteHeight()
{
	return EncoderSubsystemBase<MotorType, EncoderType>::GetPosition() + m_heightToFloor;
}

template <class MotorType, class EncoderType>
bool ElevatorBase<MotorType, EncoderType>::SetRelativeHeight(double height, double speed)
{
	return EncoderSubsystemBase<MotorType, EncoderType>::SetPosition(height, speed);
}

template <class MotorType, class EncoderType>
bool ElevatorBase<MotorType, EncoderType>::SetAbsoluteHeight(double height, double speed)
{
	return EncoderSubsystemBase<MotorType, EncoderType>::SetPosition(height - m_heightToFloor, speed);
}

template <class MotorType, class EncoderType>
void ElevatorBase<MotorType, EncoderType>::SetMinMaxHeight(double min, double max)
{
	return EncoderSubsystemBase<MotorType, EncoderType>::SetMinMaxPosition(min, max);
}