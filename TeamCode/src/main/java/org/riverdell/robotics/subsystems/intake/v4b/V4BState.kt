package org.riverdell.robotics.subsystems.intake.v4b

enum class V4BState(val position: Double)
{
    //            left.position = 1.0
    //            right.position = 0.0
    Lock(0.21),
    UnlockedIdleHover(0.3),
    Transfer(0.23),
    Intermediate(0.5), //0.5
    SampleScan(0.7), //0.5
    AutoGateway(0.77),

    Gateway(0.9),
//    Focus(0.73),`
    Pickup(0.975) //0.935
}