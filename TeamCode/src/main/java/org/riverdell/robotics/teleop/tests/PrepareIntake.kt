package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(
    name = "\uD83D\uDEE0\uFE0F Prepare Intake",
    group = "Tests"
)
class PrepareIntake : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val left = hardwareMap["intakeClawLeft"] as Servo
        val right = hardwareMap["intakeClawRight"] as Servo
        val wrist = hardwareMap["intakeWrist"] as Servo

        while (opModeIsActive())
        {
            left.position = 0.0
            right.position = 1.0
            wrist.position = 0.5
            Thread.sleep(50L)
        }
    }
}