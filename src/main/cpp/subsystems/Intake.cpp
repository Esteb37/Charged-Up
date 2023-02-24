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

#include "subsystems/Intake.h"

namespace TD
{
	template <class MotorType>
	Intake<MotorType>::Intake(unsigned int motorPort) : MotorSubsystemBase<MotorType>(motorPort)
	{
		SubsystemBase::SetName("Intake");
	}

	template <class MotorType>
	Intake<MotorType>::Intake(vector<unsigned int> motorPorts) : MotorSubsystemBase<MotorType>(motorPorts)
	{
		SubsystemBase::SetName("Intake");
        MotorSubsystemBase<MotorType>::InvertMotors({false,true});
	}



	template <class MotorType>
	void Intake<MotorType>::Periodic()
	{
	}

	// ---------- Actions -----------

	template <class MotorType>
	void Intake<MotorType>::Take()
	{
        if(MotorSubsystemBase<MotorType>::m_motorCount <= 1){
            MotorSubsystemBase<MotorType>::SetMotor(1);
        } else {
		    MotorSubsystemBase<MotorType>::SetMotors(1);
        }
	}

    template <class MotorType>
	void Intake<MotorType>::Spit()
	{
		if(MotorSubsystemBase<MotorType>::m_motorCount <= 1){
            MotorSubsystemBase<MotorType>::SetMotor(-1);
        } else {
		    MotorSubsystemBase<MotorType>::SetMotors(-1);
        }
	}

    template <class MotorType>
	void Intake<MotorType>::Invert(bool invert)
	{
		if(MotorSubsystemBase<MotorType>::m_motorCount <= 1){
            MotorSubsystemBase<MotorType>::InvertMotor(invert);
        } else {
		    MotorSubsystemBase<MotorType>::InvertMotors({invert,!invert});
        }
	}

	template class Intake<MotorTypes::SPARK>;
	template class Intake<MotorTypes::VICTOR_PWM>;
    template class Intake<MotorTypes::VICTOR_CAN>;
}