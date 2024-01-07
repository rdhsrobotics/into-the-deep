package org.robotics.robotics.xdk.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.IMU
import io.liftgate.robotics.mono.Mono
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.lang.IllegalStateException
import java.util.concurrent.TimeUnit

/**
 * A little extension function to get a non-null [HardwareDevice].
 *
 * @author Subham
 * @since 10/23/2023
 */
inline fun <reified T : HardwareDevice> LinearOpMode.hardware(id: String) = runCatching {
    hardwareMap.get(id) as T
}.getOrNull() ?: throw IllegalStateException("$id is null")

fun IMU.normalizedYaw(): Double
{
    val imuResult = robotYawPitchRollAngles
    val currentYaw = imuResult.getYaw(AngleUnit.DEGREES)

    return if (currentYaw < 0)
    {
        360 + currentYaw
    } else
    {
        currentYaw
    }
}

fun scheduleAsyncExecution(inMillis: Long, block: () -> Unit) = Mono.EXECUTION.schedule(block, inMillis, TimeUnit.MILLISECONDS)
fun runRepeating(everyMillis: Long, block: () -> Unit) =
    Mono.EXECUTION.scheduleAtFixedRate(block, 0L, everyMillis, TimeUnit.MILLISECONDS)