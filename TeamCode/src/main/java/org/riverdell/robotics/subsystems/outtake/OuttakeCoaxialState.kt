package org.riverdell.robotics.subsystems.outtake

enum class OuttakeCoaxialState(val position: Double)
{
    Ready(1.0), Transfer(0.82), Specimen(0.25), Deposit(0.45), OutsideIntake(0.63)
}