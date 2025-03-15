package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(
    name = "\uD83D\uDEE0\uFE0F Prepare Hang",
    group = "Tests"
)
class PrepareHang : LinearOpMode()
{
    override fun runOpMode()
    {
        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val primary = hardwareMap["hangPrimary"] as CRServoImplEx
        val secondary = hardwareMap["hangSecondary"] as CRServoImplEx

        while (opModeIsActive())
        {
            if (gamepad1.right_bumper) {
                primary.power = 1.0
                secondary.power = 1.0
                telemetry.addLine("powering up")
                telemetry.update()
            } else if (gamepad1.left_bumper) {
                primary.power = -1.0
                secondary.power = -1.0

                telemetry.addLine("powering down")
                telemetry.update()
            } else {
                primary.power = 0.0
                secondary.power = 0.0

                telemetry.addLine("powering nothing")
                telemetry.update()
            }
            Thread.sleep(50L)
        }
    }
}