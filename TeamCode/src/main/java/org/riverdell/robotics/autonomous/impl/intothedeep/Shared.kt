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
import java.util.concurrent.CompletableFuture

fun AnalyzedSample.buildTranslateVector(): Point {
    return translate
        .divide(110.0)
        .add(Point(
            SampleDetectionPipelinePNP.PICKUP_X_OFFSET,
            SampleDetectionPipelinePNP.PICKUP_Y_OFFSET
        ))
}

fun AnalyzedSample.buildRobotTargetVector(opMode: HypnoticAuto): Pose {
    val translate = buildTranslateVector()
    println("Translate: $translate")

    val current = opMode.robot.hardware.pinpoint.position
    return Pose(
        current.getX(DistanceUnit.INCH) - translate.x,
        current.getY(DistanceUnit.INCH) + translate.y,
        current.getHeading(AngleUnit.RADIANS)
    )
}

fun RootExecutionGroup.visionIntake(opMode: HypnoticAuto, isolated: Boolean = false) {
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline
    var detectedSample: AnalyzedSample? = null
    var detectedSampleCache: List<AnalyzedSample> = listOf()
    single("Search for sample") {
        if (this["abortMission"] != null) {
            return@single
        }

        if (isolated) {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true,
                sampleScan = true,
                submersibleOverride = 400
            ).join()

            visionPipeline.resume()
            Thread.sleep(500L)
        }

        val clock = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        var iterations = 0

        while (detectedSample == null && iterations < 4) {
            if (iterations > 0) {
                MecanumTranslations
                    .getPowers(
                        Pose(0.5, 0.0, 0.0),
                        0.0, 0.0, 0.0
                    )
                    .propagate(opMode)

                Thread.sleep(200L)
                MecanumTranslations
                    .getPowers(
                        Pose(-0.5, 0.0, 0.0),
                        0.0, 0.0, 0.0
                    )
                    .propagate(opMode)
                Thread.sleep(100L)

                MecanumTranslations
                    .getPowers(
                        Pose(0.0, 0.0, 0.0),
                        0.0, 0.0, 0.0
                    )
                    .propagate(opMode)
                Thread.sleep(450L)
            } else {
                Thread.sleep(600L)
            }

            clock.reset()
            while (detectedSample == null && clock.time() < 120) {
                visionPipeline.periodic()
                detectedSampleCache = visionPipeline.aggregateSampleCache
                    .sortedBy { it.translate.radius() }
                detectedSample = detectedSampleCache.firstOrNull()
            }

            if (detectedSample != null) {
                if (detectedSampleCache.size > 1) {
                    visionPipeline.preferredNextPick = detectedSampleCache[1]
                    println("Detected a preferred next pick sample: Translate: " + detectedSample!!.translate + " Angle: " + detectedSample!!.angle + " Color: " + detectedSample!!.color)
                }
            }

            iterations += 1
        }

        Thread.sleep(50L)

        if (detectedSample != null) {
            println("Detected a sample: Translate: " + detectedSample!!.translate + " Angle: " + detectedSample!!.angle + " Color: " + detectedSample!!.color)
        } else {
            println("Did not detect a sample.")
        }

        visionPipeline.pause()
    }

    single("Intake sample") {
        if (this["abortMission"] != null) {
            return@single
        }

        if (detectedSample != null) {
            var angle = detectedSample?.angle ?: 0.0
            if (angle > 90) {
                angle -= 180
            }

            opMode.robot.hardware.intakeWrist.position =
                WristState.Lateral.position + angle / 290.0

            val translate = detectedSample!!.buildTranslateVector()
            val target = detectedSample!!.buildRobotTargetVector(opMode)

            if (translate.radius() > SampleDetectionPipelinePNP.MIN_TRANSLATION_RADIUS) {
                navigateTo(target) {
                    withCustomTolerances(PositionChangeTolerance(
                        translateTolerance = 1.1,
                        headingToleranceRad = 2.0 * Math.PI / 180))
                    withAutomaticDeath(1250.0)
                }
            }

            opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true).join()
            opMode.robot.intakeComposite.confirmAndTransferAndReady()
                .apply {
                    if (isolated) {
                        join()
                    }
                }
        } else {
            this["abortMission"] = true

            opMode.robot.intakeComposite
                .intakeAndConfirm(slowMode = true, noPick = true)
                .thenComposeAsync {
                    opMode.robot.intakeComposite.confirmAndTransferAndReady()
                        .thenComposeAsync {
                            if (!isolated) {
                                opMode.robot.intakeComposite.outtakeCompleteAndRestFromOuttakeReady()
                                    .thenComposeAsync {
                                        opMode.robot.intakeComposite
                                            .initialOuttakeFromRest(
                                                OuttakeLevel.Bar2,
                                                shouldEnterPreDepositIfAvailable = false
                                            )
                                    }
                            } else {
                                CompletableFuture.completedFuture(null)
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