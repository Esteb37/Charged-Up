#pragma once

#include <frc/XboxController.h>

#include "human-input/Controller.hh"

namespace TD {
    class XboxController: public Controller {
        public:
            bool ButtonTop() override;
            bool ButtonBottom() override;
            bool ButtonLeft() override;
            bool ButtonRight() override;

            int Pov() override;
            bool PovTop() override;
            bool PovBottom() override;
            bool PovLeft() override;
            bool PovRight() override;

            double AxisXLeft() override;
            double AxisYLeft() override;
            double AxisXRight() override;
            double AxisYRight() override;

            bool AxisButtonLeft() override;
            bool AxisButtonRight() override;

            double TriggerLeft() override;
            double TriggerRight() override;
            bool BumperLeft() override;
            bool BumperRight() override;

            bool CenterButtonLeft() override;
            bool CenterButtonRight() override;

            void SetAxisThresholdLeft(double min, double max) override;
            void SetAxisThresholdRight(double min, double max) override;

            void SetTriggerThresholdLeft(double min, double max) override;
            void SetTriggerThresholdRight(double min, double max) override;
        
        private:
            frc::XboxController controller{0};
    };
}