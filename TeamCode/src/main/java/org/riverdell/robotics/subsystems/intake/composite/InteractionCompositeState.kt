package org.riverdell.robotics.subsystems.intake.composite

enum class InteractionCompositeState
{
    Rest,
    Pickup,
    Confirm,
    WallIntakeViaOuttake,
    OuttakeReady,
    Outtaking,
    SpecimenReady,
    HangIdled,
    HangActivated,
    TouchHang,
    InProgress

}