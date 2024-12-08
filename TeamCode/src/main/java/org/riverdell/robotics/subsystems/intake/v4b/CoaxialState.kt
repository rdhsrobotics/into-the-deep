package org.riverdell.robotics.subsystems.intake.v4b

enum class CoaxialState(val position: Double)
{
    Intake(1.0), Pickup(0.92), Transfer(0.00), Rest(0.30), Intermediate(0.6)
}