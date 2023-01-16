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

#include "subsystems/IntakeBase.h"

using namespace TD;

template <class T>
IntakeBase<T> &IntakeBase<T>::GetInstance()
{
	static IntakeBase instance;
	return instance;
}

template <class T>
IntakeBase<T>::IntakeBase(unsigned int port, unsigned int solenoidForward, unsigned int solenoidReverse) : MotorSubsystemBase(config, port), SolenoidSubsystemBase(solenoidForward, solenoidReverse)
{
	SetName("Intake");
}

template <class T>
IntakeBase<T>::IntakeBase(vector<unsigned int> ports, unsigned int solenoidForward, unsigned int solenoidReverse) : MotorSubsystemBase(config, ports), SolenoidSubsystemBase(solenoidForward, solenoidReverse)
{
	SetName("Intake");
}

template <class T>
IntakeBase<T>::IntakeBase(unsigned int port, unsigned int rightForward, unsigned int rightReverse, unsigned int leftForward, unsigned int leftReverse)
	: MotorSubsystemBase(config, port), SolenoidSubsystemBase(rightForward, rightReverse, leftForward, leftReverse)
{
	SetName("Intake");
}

template <class T>
IntakeBase<T>::IntakeBase(vector<unsigned int> ports, unsigned int rightForward, unsigned int rightReverse, unsigned int leftForward, unsigned int leftReverse) : MotorSubsystemBase(config, ports), SolenoidSubsystemBase(rightForward, rightReverse, leftForward, leftReverse)
{
	SetName("Intake");
}

template <class T>
void IntakeBase<T>::Periodic()
{
}

template <class T>
void IntakeBase<T>::Take()
{
	SetMotors(1);
}

template <class T>
void IntakeBase<T>::Spit()
{
	SetMotors(-1);
}

template <class T>
void IntakeBase<T>::Lower()
{
	OpenSolenoids();
}

template <class T>
void IntakeBase<T>::Raise()
{
	CloseSolenoids();
}
