package org.riverdell.robotics.subsystems.outtake

enum class OuttakeCoaxialState(val position: Double)
{
    Ready(1.0), Transfer(0.79), Specimen(0.6), Deposit(0.45), OutsideIntake(0.55)
}