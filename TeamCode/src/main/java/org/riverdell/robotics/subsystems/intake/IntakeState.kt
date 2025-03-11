package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionLeft: Double, val positionRight: Double)
{
    WideOpen(0.5, 0.5),
    Open(0.3, 0.7),
    Closed(0.0, 1.0),
}