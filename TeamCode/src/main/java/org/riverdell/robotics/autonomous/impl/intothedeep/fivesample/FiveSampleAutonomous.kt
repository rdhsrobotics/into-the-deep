package org.riverdell.robotics.autonomous.impl.intothedeep.fivesample

import com.acmerobotics.roadrunner.geometry.Pose2d
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.impl.intothedeep.GroundPickupPosition
import org.riverdell.robotics.autonomous.impl.intothedeep.visionIntake
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.ActionWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.composite.IntakeConfig
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.slides.ExtensionConfig
import org.riverdell.robotics.subsystems.slides.LiftConfig

abstract class FiveSampleAutonomous(
    sampleType: SampleType
) : HypnoticAuto(sampleType, { opMode ->
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline

    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val depositHighBucket = Pose(-14.0, 23.5, (45.0).degrees)

    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 30.0),
        FieldWaypoint(Pose(-70.0, 13.0, (70.0).degrees), 30.0),
        FieldWaypoint(Pose(-70.0, -38.0, (180.0).degrees), 20.0),
        FieldWaypoint(Pose(-70.0, -42.0, (180.0).degrees), 15.0)
    )

    val toSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 25.0),
        FieldWaypoint(Pose(-80.0, 20.0, 0.0), 30.0),
        ActionWaypoint {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true,
                doNotUseAutoMode = false,
                submersibleOverride = (opMode.robot as HypnoticAutoRobot).activeY
            )

            visionPipeline.resume()
        },
        FieldWaypoint(
            Pose((opMode.robot as HypnoticAutoRobot).activeX.toDouble(), 10.0, 0.0),
            25.0
        ),
        FieldWaypoint(
            Pose((opMode.robot as HypnoticAutoRobot).activeX.toDouble(), -16.0, 0.0),
            20.0
        )
    )

    val toBasket = listOf(
        FieldWaypoint(Pose(-52.1, 20.0, 10.0), 25.0),
        FieldWaypoint(depositHighBucket.add(Pose(-20.0, -20.0, 0.0)), 25.0),
        FieldWaypoint(depositHighBucket.add(Pose(-15.0, -15.0, 0.0)), 20.0),
        FieldWaypoint(depositHighBucket.add(Pose(-10.0, -3.0, 0.0)), 15.0),
        FieldWaypoint(depositHighBucket.add(Pose(5.0, 5.0)), 10.0),
    )

    fun submersibleCycle() {
        single("submerse itself") {
            visionPipeline.pause()
            purePursuitNavigateTo(*toSubmersible.toTypedArray()) {
                withAutomaticDeath(10000.0)
                withCustomMaxTranslationalSpeed(1.0)
                withCustomMaxRotationalSpeed(1.0)
            }
        }

        visionIntake(opMode)

        single("Return to basket or park") {
            if (this["abortMission"] != null) {
                navigateTo(Pose(-82.0, 0.0, 180.0.degrees))
                navigateTo(Pose(-82.0, -25.0, 180.0.degrees))
                return@single
            }

            visionPipeline.pause()
            purePursuitNavigateTo(*toBasket.toTypedArray()) {
                withAutomaticDeath(10000.0)
                withCustomMaxTranslationalSpeed(1.0)
                withCustomMaxRotationalSpeed(1.0)
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Outtaking)) {
                return@single
            }

            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
        }
    }

    opMode.robot.drivetrain.localizer.poseEstimate = startPose

    fun ExecutionGroup.depositToHighBasket(initial: Boolean = false) {
        single("high basket deposit") {
            visionPipeline.pause()
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

            navigateTo(depositHighBucket) {
                withCustomHeadingTolerance(2.0)
                withCustomTranslationalTolerance(1.5)
                withExtendoOut(true)
            }

            if (!opMode.robot.intakeComposite.waitForState(InteractionCompositeState.Outtaking)) {
                return@single
            }

            opMode.robot.intakeComposite.outtakeCompleteAndRest().join()
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
                if (position.dynamicPosition != null)
                {
                    opMode.robot.intake.dynamicWrist(position.dynamicPosition!!)
                }
            }

            navigateTo(position.pose) {
                withExtendoOut(true)
                withAutomaticDeath(if (position.extendoMode) 7000.0 else 5000.0)
                withCustomHeadingTolerance(0.8)
                withCustomTranslationalTolerance(0.7)
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

    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(-11.75, 19.0, (90.0).degrees)),
        GroundPickupPosition(pose = Pose(-11.75, 34.75, (90.0).degrees)),
        GroundPickupPosition(
            pose = Pose(  -21.37, 24.28, 2.266),
            extendoMode = true,
            wristState = WristState.Lateral,
            dynamicPosition = 0.6,
            extendoPosition = 300
        ),
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer(it.extendoMode)
        depositToHighBasket()
    }

    submersibleCycle()

    single("park near submersible") {
        if (this["abortMission"] == null)
        {
            visionPipeline.pause()
            opMode.robot.intakeComposite
                .initialOuttakeFromRest(
                    OuttakeLevel.Bar2,
                    shouldEnterPreDepositIfAvailable = false
                )

            purePursuitNavigateTo(*parkSubmersible.toTypedArray()) {
                withAutomaticDeath(9000.0)
                withCustomMaxTranslationalSpeed(0.5)
                withCustomMaxRotationalSpeed(0.5)
            }
        }
    }
}, {
    if (instance.gamepad1.right_trigger > 0.1) {
        it.activeX -= (it.opMode.gamepad1.right_trigger * 0.5).toInt()
    }

    if (instance.gamepad1.left_trigger > 0.1) {
        it.activeY -= (it.opMode.gamepad1.left_trigger * 0.5).toInt()
    }

    it.opMode.telemetry.addLine("Target X (Position): ${it.activeX}")
    it.opMode.telemetry.addLine("Target Y (Extension): ${it.activeY}")
}
)