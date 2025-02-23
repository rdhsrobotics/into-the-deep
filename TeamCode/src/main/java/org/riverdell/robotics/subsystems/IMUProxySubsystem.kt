package org.riverdell.robotics.subsystems

import com.qualcomm.robotcore.hardware.IMU
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.riverdell.robotics.HypnoticOpMode

class IMUProxySubsystem(private val hypnoticOpMode: HypnoticOpMode) : AbstractSubsystem()
{
    enum class PreferredIMU(val getIMU: (HypnoticOpMode) -> IMU)
    {
        BHI({
            it.robot.hardware.bhiIMU
        }),
        BNO({
            it.robot.hardware.bnoIMU
        })
    }

    var previousYPRA: YawPitchRollAngles? = null
    var preferredIMUType = PreferredIMU.BHI
    var preferredIMU: IMU = preferredIMUType.getIMU(hypnoticOpMode)
    var failures = 0

    val imuState by state(write = { _ -> }, read = {
        val newAngles = preferredIMU.robotYawPitchRollAngles
        if (newAngles.yaw == 0.0 && newAngles.roll == 0.0 && newAngles.pitch == 0.0)
        {
            failures += 1

            if (failures >= 5) {
                println("Switched preferred type as the $preferredIMUType was deemed broken")
                preferredIMUType = if (preferredIMUType == PreferredIMU.BHI)
                    PreferredIMU.BNO else PreferredIMU.BHI
                preferredIMU = preferredIMUType.getIMU(hypnoticOpMode)
                failures = 0

                return@state previousYPRA
                    ?: preferredIMU.robotYawPitchRollAngles
            }

            if (previousYPRA != null)
            {
                return@state previousYPRA!!
            }
        }

        failures = 0
        return@state newAngles
    })

    fun imu() = kotlin.runCatching { imuState.current() }
        .getOrElse {
            YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)
        }

    override fun doInitialize() {
    }

    override fun start() {
    }

}