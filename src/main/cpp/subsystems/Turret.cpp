#include "subsystems/Turret.h"

Turret::Turret() {
    followerController.Follow(leadController);
}

void Turret::Periodic() {}
