package org.riverdell.robotics.autonomous.impl.intothedeep.sixsample

import com.acmerobotics.roadrunner.geometry.Pose2d
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.impl.intothedeep.GroundPickupPosition
import org.riverdell.robotics.autonomous.impl.intothedeep.buildRobotTargetVector
import org.riverdell.robotics.autonomous.impl.intothedeep.visionIntake
import org.riverdell.robotics.autonomous.movement.PositionChangeTolerance
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import java.util.concurrent.CompletableFuture

abstract class SixSampleAutonomous(
    sampleType: SampleType
) : HypnoticAuto(sampleType, { opMode ->
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline

    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val depositHighBucket = Pose(19.2, -5.8, (43.26).degrees)
    val depositHighBucketFinal = Pose(18.0, -5.0, (42.0).degrees)

    val submersibleInitialPose = Pose(-6.0, -52.0, 0.degrees)
    val submersibleInitialPoseSecondCycle = Pose(-6.0, -55.0, 0.degrees)

    val submersibleIntermediate = Pose(-6.96, -47.05, 54.76.degrees)
    val submersibleRotatedIntermediate = Pose(1.07, -54.27, 0.degrees)
    val submersibleReturnIntermediate = Pose(-0.31, -44.64, 59.4.degrees)
    val submersiblePark = Pose(-18.0, -46.56, 180.degrees)

    val abortMoveBack = Pose(-6.0, -52.54, 0.degrees)
    val abortRotated = Pose(4.04, -46.49, 180.degrees)

    val pickupTolerance = PositionChangeTolerance(1.0, 5.0, 0.8 * Math.PI / 180, 3.0, 80.0)
    val initialBasketTolerance = PositionChangeTolerance(2.0, 8.0, 2.5 * Math.PI / 180, 10.0, 100.0)
    val basketTolerance = PositionChangeTolerance(2.5, 25.0, 2.5 * Math.PI / 180, 12.5, 180.0)
    val movingTolerance = PositionChangeTolerance(3.5, 60.0, 5.0 * Math.PI / 180, 100.0, 250.0)

    val pickupPositions = listOf(
        GroundPickupPosition(
            pose = Pose(18.26, -8.35, (77.5).degrees),
            dynamicPosition = 0.46
        ),
        GroundPickupPosition(pose = Pose(21.88, -6.6, (91.3.degrees))),
        GroundPickupPosition(
            pose = Pose(17.0, -15.45, 131.71.degrees),
            extendoMode = true,
            wristState = WristState.Lateral,
            dynamicPosition = 0.6,
            extendoPosition = 250
        ),
    )

    var hasConsumedAbort = false

    fun submersibleCycle(
        depositHighBucketPose: Pose = depositHighBucket,
        initialSubPose: Pose = submersibleInitialPose
    ) {
        single("submerse itself") {
            if (this["abortMission"] != null) {
                return@single
            }

            navigateTo(submersibleIntermediate) {
                withCustomTolerances(movingTolerance)
                withAutomaticDeath(4000.0)
                noStop(true)
            }

            navigateTo(submersibleRotatedIntermediate) {
                withCustomTolerances(movingTolerance)
                withAutomaticDeath(2000.0)
                noStop(true)
            }

            CompletableFuture.runAsync {
                visionPipeline.resume()
                opMode.robot.intakeComposite.prepareForPickup(
                    WristState.Lateral,
                    wideOpen = true,
                    submersibleOverride = 400
                )
            }

            if (visionPipeline.preferredNextPick != null) {
                println("Using preferred next pick for initial sub pose")
            }

            navigateTo(
                visionPipeline.preferredNextPick?.buildRobotTargetVector(opMode)
                    ?: initialSubPose
            ) {
                withAutomaticDeath(3500.0)
                withExtendoOut(true)
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Pickup)) {
                return@single
            }
        }

        visionIntake(opMode)

        single("Return to basket or park") {
            visionPipeline.pause()
            if (this["abortMission"] != null && !hasConsumedAbort) {
                hasConsumedAbort = true
                navigateTo(abortMoveBack)
                navigateTo(abortRotated)
                navigateTo(submersiblePark)
                return@single
            }

            if (this["abortMission"] != null) {
                return@single
            }

            navigateTo(submersibleReturnIntermediate) {
                withCustomTolerances(movingTolerance)
                withAutomaticDeath(3500.0)
                noStop(true)
            }

            CompletableFuture.runAsync {
                if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.OuttakeReady)) {
                    return@runAsync
                }

                opMode.robot.intakeComposite.initialOuttake(OuttakeLevel.HighBasket)
            }

            navigateTo(depositHighBucketPose) {
                withCustomTolerances(initialBasketTolerance)
                withAutomaticDeath(4000.0)
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Outtaking)) {
                return@single
            }

            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
        }
    }

    opMode.robot.hardware.pinpoint.setPosition(
        Pose2D(
            DistanceUnit.INCH,
            startPose.x,
            startPose.y,
            AngleUnit.RADIANS,
            startPose.heading
        )
    )

    fun ExecutionGroup.depositToHighBasket(initial: Boolean = false) {
        single("high basket deposit") {
            if (initial) {
                opMode.robot.intakeComposite
                    .initialOuttakeFromRest(OuttakeLevel.HighBasket)
            } else {
                opMode.robot.intakeComposite
                    .confirmAndTransferAndReady()
                    .thenComposeAsync {
                        opMode.robot.intakeComposite
                            .initialOuttake(OuttakeLevel.HighBasket)
                    }
            }

            if (initial) {
                navigateTo(depositHighBucket) {
                    withCustomTolerances(initialBasketTolerance)
                }
            } else {
                navigateTo(depositHighBucket) {
                    withCustomTolerances(basketTolerance)
                }
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Outtaking)) {
                return@single
            }

            opMode.robot.intakeComposite
                .outtakeCompleteAndRest(
                    waitPeriod = if (initial) 300L else 200L
                )
                .join()
        }
    }

    fun ExecutionGroup.prepareForIntake(
        position: GroundPickupPosition
    ) {

        single("navigate to pickup position") {
            visionPipeline.pause()
            opMode.robot.intakeComposite.prepareForPickup(
                position.wristState,
                // needs to go closer into the wall
                doNotUseAutoMode = position.extendoMode,
                submersibleOverride = position.extendoPosition,
                wideOpen = true,
            ).thenAcceptAsync {
                if (position.dynamicPosition != null) {
                    opMode.robot.intake.dynamicWrist(position.dynamicPosition!!)
                }
            }

            navigateTo(position.pose) {
                withExtendoOut(true)
                withAutomaticDeath(if (position.extendoMode) 1500.0 else 2000.0)
                withCustomTolerances(pickupTolerance)
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Pickup)) {
                return@single
            }
        }
    }

    fun ExecutionGroup.confirmIntakeAndTransfer(extendoMode: Boolean = false) {
        single("confirm intake") {
            visionPipeline.pause()
            opMode.robot.intakeComposite
                .intakeAndConfirm {
                    if (extendoMode) {
                        opMode.robot.extension.slides.goTo(0)
                    }
                }
                .join()
        }
    }

    // preload
    depositToHighBasket(initial = true)

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer(it.extendoMode)
        depositToHighBasket()
    }

    submersibleCycle()
    submersibleCycle(
        depositHighBucketFinal,
        initialSubPose = submersibleInitialPoseSecondCycle
    )

    single("park near submersible") {
        navigateTo(submersiblePark)
    }
})