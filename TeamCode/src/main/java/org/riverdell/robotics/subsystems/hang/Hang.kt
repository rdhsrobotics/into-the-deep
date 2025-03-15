package org.riverdell.robotics.subsystems.hang

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot

class Hang(private val robot: HypnoticRobot) : AbstractSubsystem()
{
    private var state = HangState.Idle
    fun powered()
    {
        state = HangState.Powered
        robot.hardware.hangSecondary.power = -1.0
    }

    fun idle()
    {
        state = HangState.Idle
        robot.hardware.hangSecondary.power = 0.0
    }

    override fun doInitialize()
    {
        idle()
    }

    override fun start()
    {
    }
}