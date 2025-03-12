package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.2),
    Transfer(0.65),
    Deposit(0.1),
    AutoPreDeposit(0.4),
    Force(0.0),
    Specimen(0.0)
}