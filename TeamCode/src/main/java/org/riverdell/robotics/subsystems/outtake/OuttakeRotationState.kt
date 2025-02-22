package org.riverdell.robotics.subsystems.outtake

enum class OuttakeRotationState(val position: Double)
{
    Ready(0.50), Transfer(0.80), Deposit(0.45), AutoPreDeposit(0.6), Force(0.15), Specimen(0.1)
}