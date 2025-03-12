package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.85), PostScore(0.75), Scoring(0.7), Hover(0.43),PostHover(0.38),  Pickup(0.33)
}