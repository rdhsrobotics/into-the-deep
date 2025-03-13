package org.riverdell.robotics.teleop.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.riverdell.robotics.autonomous.detection.VisionPipeline

@TeleOp(
    name = "Prepare Vision",
    group = "Tests"
)
class PrepareVision : LinearOpMode()
{
    override fun runOpMode()
    {
        val visionPipeline = VisionPipeline(this)
        visionPipeline.initialize()

        waitForStart()
        if (isStopRequested)
        {
            return
        }

        val multipleTelemetry = MultipleTelemetry(
            telemetry,
            FtcDashboard.getInstance().telemetry
        )

        visionPipeline.resume()


        val left = hardwareMap["intakeV4BLeft"] as Servo
        left.position = 0.3

        val right = hardwareMap["intakeV4BRight"] as Servo
        right.position = 0.7

        val coaxial = hardwareMap["intakeV4BCoaxial"] as Servo
        coaxial.position = 0.685

        while (opModeIsActive())
        {
            visionPipeline.periodic()

            multipleTelemetry.addLine("=== Detected Sample ===")
            visionPipeline.detectedSample?.apply {
                multipleTelemetry.addLine("Angle: ${
                    this.angle
                }")
                multipleTelemetry.addLine("Vector: ${
                    this.translate
                }")
                multipleTelemetry.addLine("Color: ${
                    this.color
                }")
                multipleTelemetry.addLine("Area: ${
                    this.area
                }")
            } ?: run {
                multipleTelemetry.addLine("NONE")
            }
            multipleTelemetry.update()

            Thread.sleep(50L)
        }

        visionPipeline.dispose()
    }
}