package org.riverdell.robotics.teleop.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.riverdell.robotics.autonomous.detection.SampleDetectionPipelinePNP
import org.riverdell.robotics.autonomous.detection.VisionPipeline

@TeleOp(
    name = "\uD83D\uDEE0\uFE0F Prepare Vision",
    group = "Tests"
)
class PrepareVision : LinearOpMode() {
    override fun runOpMode() {
        val visionPipeline = VisionPipeline(this)
        visionPipeline.initialize()

        waitForStart()
        if (isStopRequested) {
            return
        }

        var currentStreamType = VisionPipeline.STREAM_VIEW
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

        while (opModeIsActive()) {
            visionPipeline.periodic()

            fun SampleDetectionPipelinePNP.AnalyzedSample.reportToTelemetry(title: String) {
                multipleTelemetry.addLine("=== $title ===")
                multipleTelemetry.addLine("Angle: $angle")
                multipleTelemetry.addLine("Vector: $translate")
                multipleTelemetry.addLine("Color: $color")
                multipleTelemetry.addLine("Area: $area")
            }

            multipleTelemetry.addLine("=== Streaming ===")
            multipleTelemetry.addLine("View: $currentStreamType")

            multipleTelemetry.addLine("=== Detected Sample ===")
            multipleTelemetry.addLine("Found: ${visionPipeline.aggregateSampleCache.size}")

            if (visionPipeline.aggregateSampleCache.isNotEmpty()) {
                val cache = visionPipeline.aggregateSampleCache
                    .sortedBy { it.translate.radius() }

                cache.firstOrNull()
                    ?.apply {
                        reportToTelemetry("Primary Selection")
                    }

                if (cache.size > 1) {
                    cache[1].reportToTelemetry("Secondary Selection (Next Cycle)")
                }
            } else {
                multipleTelemetry.addLine("=== None detected ===")
            }

            multipleTelemetry.update()
            Thread.sleep(50L)
        }

        visionPipeline.dispose()
    }
}