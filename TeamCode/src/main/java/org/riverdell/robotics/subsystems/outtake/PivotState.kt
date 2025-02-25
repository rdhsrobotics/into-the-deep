package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.85), PostScore(0.65), Scoring(0.7), Hover(0.40),PreHover(0.43),  Pickup(0.33)
}