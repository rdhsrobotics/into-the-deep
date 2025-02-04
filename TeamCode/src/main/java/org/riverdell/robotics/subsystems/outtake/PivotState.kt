package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.85), PostScore(0.63), Scoring(0.7), Hover(0.33), Pickup(0.25)
}