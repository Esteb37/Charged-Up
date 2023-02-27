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
#include "subsystems/EncoderSubsystemBase.h"
#include <frc2/command/Commands.h>

using namespace frc2;
using namespace frc2::cmd;

namespace TD
{
    class Turret : virtual public EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>
    {
    public:
        enum class Poses
        {
            kFront,
            kSide,
            kBack,
            kMoving,
        };

        Turret(unsigned int motorPort);

        void Periodic() override;

        void PrintPose();

        void PrintAngle();

        CommandPtr SetPose(Poses);

        Poses GetPose();

        std::string GetPoseStr();

        static std::string PoseToString(Poses pose);

    private:
        Poses m_pose = Poses::kFront;
    };
}