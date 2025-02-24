package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.85), PostScore(0.7), Scoring(0.7), Hover(0.33),PreHover(0.36),  Pickup(0.25)
}