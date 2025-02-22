package org.riverdell.robotics.teleop.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.riverdell.robotics.autonomous.detection.VisionPipeline

@TeleOp(
    name = "Test | Sample Vision",
    group = "Tests"
)
class TestSampleDetection : LinearOpMode() {
    val visionPipeline by lazy { VisionPipeline(this) }
    override fun runOpMode() {
        waitForStart()
        if (isStopRequested) {
            return
        }

        visionPipeline.doInitialize()

        val somethingLikeThis = hardwareMap.get(Servo::class.java, "intakeWrist")

        hardwareMap.get(Servo::class.java, "intakeV4BRight")
            .apply {
                position = 0.77
            }

        hardwareMap.get(Servo::class.java, "intakeV4BLeft")
            .apply {
                position = 0.23
            }

        hardwareMap.get(Servo::class.java, "intakeV4BCoaxial")
            .apply {
                position = 0.95
            }


        somethingLikeThis.position = 0.5
        visionPipeline.resume()

        while (opModeIsActive()) {
            somethingLikeThis.position = 0.5
            Thread.sleep(400)
            visionPipeline.detectedSample = null
            while (visionPipeline.detectedSample == null && opModeIsActive()) {
                visionPipeline.periodic()
            }
            val angle = visionPipeline.detectedSample?.angle ?: 0.0
            //somethingLikeThis.position = 0.5 + (angle - 90) / 290
        }

        visionPipeline.dispose()
    }
}