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
    Date: 17/01/2023
    Language: cpp
    Copyright (c) TecDroid 3354 and Esteban Padilla Cerdio
    Open Source Software; you can modify and/or share it under the terms of
*/

#pragma once

#include "Constants.h"
#include <AHRS.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/geometry/Pose3D.h>
#include <frc/interfaces/Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace frc;
using namespace std;

namespace TD
{

    namespace GyroTypes
    {
        typedef ADIS16448_IMU CLASSIC;
        typedef AHRS NAVX;
    }

    template <class T>
    class CustomGyro
    {
    public:
        CustomGyro();

        // ----------------------- Gyro -----------------------

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetAngle();

        /**
         * @brief Get the absolute angle to which the robot is heading
         * @return double the absolute angle
         */
        units::angle::degree_t GetHeading();

        /**
         * @brief Resets the angle to 0
         */
        void Reset();

        /**
         * @brief Invert the direction of the gyro
         * @param invert True to invert, false to not
         */
        void Invert(bool);

        /**
         * @brief Publish the value of the gyro to the dashboard
         */
        void Print();

        void PrintAngles();

        void PrintYaw();

        void PrintPitch();

        void PrintRoll();

        frc::Rotation2d GetRotation2d();

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetYaw();

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetPitch();

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetRoll();

        double GetDisplacementX();

        double GetDisplacementY();

        double GetDisplacementZ();

        void ResetDisplacement();

        void UpdateDisplacement();

        void PrintDisplacement();

    private:
        T *m_gyro;

        units::angle::degree_t m_heading;

        char m_direction = 1;
    };
}