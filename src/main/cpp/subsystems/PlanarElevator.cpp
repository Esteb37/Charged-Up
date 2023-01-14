#include "subsystems/PlanarElevator.h"

namespace TD {

    void PlanarElevator::Periodic() {
    }

    void PlanarElevator::MoveTo(utility::Point2D const &point) {
        xAxis->SetPosition(point.x, point.y);
    }

    void PlanarElevator::MoveHorizontally(float amount) {
        xAxis->Move(amount);
    }

    void PlanarElevator::MoveVertically(float amount) {
        yAxis->Move(amount);
    }

    void PlanarElevator::GotoTopLeft() {
        MoveTo(plane.topLeft());
    }

    void PlanarElevator::GotoTopRight() {
        MoveTo(plane.topRight());
    }

    void PlanarElevator::GotoBottomLeft() {
        MoveTo(plane.bottomLeft());
    }

    void PlanarElevator::GotoBottomRight() {
        MoveTo(plane.bottomRight());
    }

    void PlanarElevator::GotoOrigin() {
        MoveTo(plane.origin());
    }

    void PlanarElevator::GotoPositiveX() {
        MoveTo(plane.topCenter());
    }

    void PlanarElevator::GotoNegativeX() {
        MoveTo(plane.bottomCenter());
    }

    void PlanarElevator::GotoPositiveY() {
        MoveTo(plane.rightCenter());
    }

    void PlanarElevator::GotoNegativeY() {
        MoveTo(plane.leftCenter());
    }


}