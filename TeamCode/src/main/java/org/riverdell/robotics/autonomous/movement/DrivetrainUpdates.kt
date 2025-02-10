package org.riverdell.robotics.autonomous.movement

import org.riverdell.robotics.HypnoticRobot.Companion.pwmResolution
import org.riverdell.robotics.autonomous.HypnoticAuto
import kotlin.math.abs

data class DrivetrainUpdates(
    val newFrontLeft: Double,
    val newFrontRight: Double,
    val newBackLeft: Double,
    val newBackRight: Double,
)
{
    fun propagate(pipeline: HypnoticAuto)
    {
        pipeline.robot.hardware.frontLeft.power = newFrontLeft
        pipeline.robot.hardware.frontRight.power = newFrontRight
        pipeline.robot.hardware.backLeft.power = newBackLeft
        pipeline.robot.hardware.backRight.power = newBackRight
    }

    fun equalsUpdate(other: DrivetrainUpdates): Boolean {
        return (abs(other.newFrontLeft - newFrontLeft) < pwmResolution) &&
                (abs(other.newFrontRight - newFrontRight) < pwmResolution) &&
                (abs(other.newBackLeft - newBackLeft) < pwmResolution) &&
                (abs(other.newBackRight - newBackRight) < pwmResolution)
    }
}