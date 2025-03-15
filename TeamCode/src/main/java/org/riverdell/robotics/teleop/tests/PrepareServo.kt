package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.riverdell.robotics.teleop.tests.config.ServoConfig

@TeleOp(
    name = "\uD83D\uDEE0\uFE0F Prepare Servo",
    group = "Tests"
)
class PrepareServo : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        while (opModeIsActive())
        {
            val hardware = hardwareMap[ServoConfig.name] as Servo
            hardware.position = ServoConfig.position
            Thread.sleep(50L)
        }
    }
}