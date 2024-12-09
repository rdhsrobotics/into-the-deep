package org.riverdell.robotics.subsystems.outtake

enum class OuttakeCoaxialState(val positinn: Double)
{
    Ready(1.0), Transfer(0.85), Deposit(0.45), OutsideIntake(0.55)
}