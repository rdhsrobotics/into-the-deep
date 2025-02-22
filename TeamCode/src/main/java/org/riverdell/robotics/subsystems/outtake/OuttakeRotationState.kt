package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.40),
    Transfer(0.715),
    Deposit(0.32),
    AutoPreDeposit(0.52),
    Force(0.2),
    Specimen(0.2)
}