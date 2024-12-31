package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.50), Transfer(0.77), Deposit(0.45), Force(0.25), Specimen(0.5)
}