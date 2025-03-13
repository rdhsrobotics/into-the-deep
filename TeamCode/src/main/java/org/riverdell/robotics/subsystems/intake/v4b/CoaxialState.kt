package org.riverdell.robotics.subsystems.intake.v4b

enum class CoaxialState(val position: Double)
{
    Intake(0.655), Pickup(0.655), Transfer(0.01), Rest(0.01), Intermediate(0.3)
}