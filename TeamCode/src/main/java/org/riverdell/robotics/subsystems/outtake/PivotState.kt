package org.riverdell.robotics.subsystems.outtake

enum class PivotState(val rightPosition: Double)
{
    Initialize(0.7), PostScore(0.5), Scoring(0.55), Hover(0.22), Pickup(0.16)
}