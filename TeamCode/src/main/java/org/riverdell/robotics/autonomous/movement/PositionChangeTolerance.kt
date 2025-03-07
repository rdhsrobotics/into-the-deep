package org.riverdell.robotics.autonomous.movement

class PositionChangeTolerance (
    @JvmField
    var translateTolerance: Double = 1.1,
    @JvmField
    var translateToleranceVel: Double = 2.5,
    @JvmField
    var headingToleranceRad: Double = 1.4 * Math.PI / 180,
    @JvmField
    var headingToleranceVel: Double = 25.0,
    @JvmField
    var atTargetMillis: Double = 50.0
    @JvmField
    var timeToleranceMs: Double = 100.0
)
{

}
