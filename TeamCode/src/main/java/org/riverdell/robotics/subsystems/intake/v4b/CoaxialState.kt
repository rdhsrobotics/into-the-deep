package org.riverdell.robotics.subsystems.intake.v4b

enum class CoaxialState(val position: Double)
{
    Intake(0.95), Pickup(0.95), Transfer(0.25), Rest(0.3), Intermediate(0.6)
}