package org.riverdell.robotics.subsystems.intake.v4b

enum class CoaxialState(val position: Double)
{
    Intake(0.685), Pickup(0.685), Transfer(0.07), Rest(0.07), Intermediate(0.345)
}