package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.util.ElapsedTime
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.HypnoticAuto.HypnoticAutoRobot
import org.riverdell.robotics.autonomous.detection.SampleDetectionPipelinePNP
import org.riverdell.robotics.autonomous.detection.SampleDetectionPipelinePNP.AnalyzedSample
import org.riverdell.robotics.autonomous.movement.MecanumTranslations
import org.riverdell.robotics.autonomous.movement.PositionChangeTolerance
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import kotlin.math.abs

fun RootExecutionGroup.visionIntake(opMode: HypnoticAuto, isolated: Boolean = false) {
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline
    var detectedSample: AnalyzedSample? = null
    single("Search for sample") {
        visionPipeline.resume()

        opMode.robot.intakeComposite.prepareForPickup(
            WristState.Lateral,
            wideOpen = true,
            submersibleOverride = 300
        ).join()

        val clock = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        var iterations = 0
        while (detectedSample == null && iterations < 3) {
            if (iterations > 0) {
                MecanumTranslations
                    .getPowers(
                        Pose(0.0, -0.3, 0.0),
                        0.0, 0.0, 0.0
                    )
                    .propagate(opMode)

                Thread.sleep(200L)
                MecanumTranslations
                    .getPowers(
                        Pose(0.0, 0.0, 0.0),
                        0.0, 0.0, 0.0
                    )
                    .propagate(opMode)
            }

            clock.reset()
            while (detectedSample == null && clock.time() < 68) {
                visionPipeline.periodic()
                detectedSample = visionPipeline.detectedSample
            }

            iterations += 1
        }

        Thread.sleep(50L)

        if (detectedSample != null) {
            println("Detected a sample: Translate: " + detectedSample!!.translate + "Angle: " + detectedSample!!.angle)
        } else {
            println("Did not detect a sample.")
        }

        visionPipeline.pause()
    }

    single("Intake sample") {
        println("Picking up")
        if (detectedSample != null) {
            val angle = detectedSample?.angle ?: 0.0
            opMode.robot.hardware.intakeWrist.position =
                WristState.Lateral.position + (angle - 90) / 290.0

            val translate = detectedSample!!.translate.divide(90.0)
                .add(Point(0.5, SampleDetectionPipelinePNP.PICKUP_Y_OFFSET))

            println("Translate: $translate")

            val current = opMode.robot.hardware.pinpoint.position
            val target = Pose(
                current.getX(DistanceUnit.INCH) + translate.x,
                current.getY(DistanceUnit.INCH) + translate.y,
                current.getHeading(AngleUnit.RADIANS)
            )

            if (abs(translate.x) > 2.0 || abs(translate.y) > 1.5) {
                navigateTo(target) {
                    withCustomTolerances(PositionChangeTolerance(
                        translateTolerance = 1.3,
                        headingToleranceRad = 2.0 * Math.PI / 180))
                    withAutomaticDeath(1250.0)
                }
            }

            opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true).join()
            opMode.robot.intakeComposite.confirmAndTransferAndReady()
                .apply {
                    if (isolated) {
                        join()
                    } else {
                        thenAccept {
                            opMode.robot.intakeComposite
                                .initialOuttake(OuttakeLevel.HighBasket)
                        }
                    }
                }
        } else {
            this["abortMission"] = true

            opMode.robot.intakeComposite
                .intakeAndConfirm(slowMode = true, noPick = true)
                .thenComposeAsync {
                    opMode.robot.intakeComposite.confirmAndTransferAndReady()
                        .thenComposeAsync {
                            opMode.robot.intakeComposite.outtakeCompleteAndRestFromOuttakeReady()
                                .thenComposeAsync {
                                    opMode.robot.intakeComposite
                                        .initialOuttakeFromRest(
                                            OuttakeLevel.Bar2,
                                            shouldEnterPreDepositIfAvailable = false
                                        )
                                }
                        }
                }
                .apply {
                    if (isolated) {
                        join()
                    }
                }
        }

        visionPipeline.pause()
    }
}