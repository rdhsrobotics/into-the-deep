package org.riverdell.robotics.subsystems.intake

enum class IntakeState(val positionLeft: Double, val positionRight: Double)
{
    WideOpen(0.4, 0.6),
    Open(0.3, 0.7),
    Closed(0.0, 1.0),
}