#pragma once

#include "utility/Threshold.hh"

namespace TD {

    namespace Constants {
        constexpr unsigned char TOP_POV_ID = 0;
        constexpr unsigned char BOTTOM_POV_ID = 4;
        constexpr unsigned char LEFT_POV_ID = 2;
        constexpr unsigned char RIGHT_POV_ID = 3;
    }

    class Controller {
        public:
            virtual bool ButtonTop() = 0;
            virtual bool ButtonBottom() = 0;
            virtual bool ButtonLeft() = 0;
            virtual bool ButtonRight() = 0;

            virtual int Pov() = 0;
            virtual bool PovTop() = 0;
            virtual bool PovBottom() = 0;
            virtual bool PovLeft() = 0;
            virtual bool PovRight() = 0;

            virtual double AxisXLeft() = 0;
            virtual double AxisYLeft() = 0;
            virtual double AxisXRight() = 0;
            virtual double AxisYRight() = 0;

            virtual bool AxisButtonLeft() = 0;
            virtual bool AxisButtonRight() = 0;

            virtual double TriggerLeft() = 0;
            virtual double TriggerRight() = 0;
            virtual bool BumperLeft() = 0;
            virtual bool BumperRight() = 0;

            virtual bool CenterButtonLeft() = 0;
            virtual bool CenterButtonRight() = 0;

            virtual void SetAxisThresholdLeft(double min, double max) = 0;
            virtual void SetAxisThresholdRight(double min, double max) = 0;

            virtual void SetTriggerThresholdLeft(double min, double max) = 0;
            virtual void SetTriggerThresholdRight(double min, double max) = 0;

        protected:
            utility::Threshold axisThresholdLeft{0, 1};
            utility::Threshold axisThresholdRight{0, 1};
            utility::Threshold triggerThresholdLeft{0, 1};
            utility::Threshold triggerThresholdRight{0, 1};
    };
}