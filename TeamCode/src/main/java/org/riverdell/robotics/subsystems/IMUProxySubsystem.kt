package org.riverdell.robotics.subsystems

import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.riverdell.robotics.HypnoticOpMode

class IMUProxySubsystem(private val hypnoticOpMode: HypnoticOpMode) : AbstractSubsystem()
{
    //val imuState by state(write = { _ -> }, read = { hypnoticOpMode.robot.hardware.imu.robotYawPitchRollAngles })

    var previousPitchRollAngles: YawPitchRollAngles? = null
    val imuLolState by state(write = { _ -> }, read = {
        val newAngles = hypnoticOpMode.robot.hardware.imuLol.robotYawPitchRollAngles
        if (newAngles.yaw.isNaN())
        {
            if (previousPitchRollAngles != null)
            {
                return@state previousPitchRollAngles!!
            }
        }

        return@state newAngles
    })
    //fun imu() = kotlin.runCatching { imuState.current() }.getOrElse { YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L) }
    fun alternativeImu() = kotlin.runCatching { imuLolState.current() }.getOrElse { YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L) }

    override fun doInitialize() {
    }

    override fun start() {
    }

}