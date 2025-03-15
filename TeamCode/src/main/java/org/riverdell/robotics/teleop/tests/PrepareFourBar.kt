package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(
    name = "\uD83D\uDEE0\uFE0F Prepare V4B",
    group = "Tests"
)
class PrepareFourBar : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val left = hardwareMap["intakeV4BLeft"] as Servo
        val right = hardwareMap["intakeV4BRight"] as Servo
        val coaxial = hardwareMap["intakeV4BCoaxial"] as Servo

        while (opModeIsActive())
        {
            left.position = 1.0
            right.position = 0.0
            coaxial.position = 0.0
            Thread.sleep(50L)
        }
    }
}