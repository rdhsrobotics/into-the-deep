package org.robotics.robotics.xdk.teamcode.subsystem

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import io.liftgate.robotics.mono.pipeline.StageContext
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.robotics.robotics.xdk.teamcode.autonomous.hardware
import org.robotics.robotics.xdk.teamcode.subsystem.claw.ClawExpansionConstants

class AirplaneLauncher(private val opMode: LinearOpMode) : AbstractSubsystem()
{
    private val backingServo by lazy {
        opMode.hardware<Servo>("launcher")
    }

    override fun composeStageContext() = object : StageContext
    {
        override fun isCompleted() = true
        override fun dispose() = reset()
    }

    /**
     * Launches the airplane.
     */
    fun launch()
    {
        backingServo.position = ClawExpansionConstants.MAX_PLANE_POSITION
    }

    /**
     * Arms the airplane servo.
     */
    fun reset()
    {
        backingServo.position = ClawExpansionConstants.DEFAULT_PLANE_POSITION
    }

    override fun doInitialize() = reset()
    override fun dispose() = reset()

    // Servos don't have isBusy states
    override fun isCompleted() = true
}