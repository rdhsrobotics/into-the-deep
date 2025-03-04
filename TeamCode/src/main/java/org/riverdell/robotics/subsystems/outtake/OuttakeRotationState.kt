package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.8),
    Transfer(0.35),
    Deposit(0.8),
    AutoPreDeposit(0.5),
    Force(1.0),
    Specimen(1.0)
}