package org.riverdell.robotics.autonomous.impl.intothedeep.fivesample

import com.acmerobotics.roadrunner.geometry.Pose2d
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.single
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.impl.intothedeep.GroundPickupPosition
import org.riverdell.robotics.autonomous.impl.intothedeep.visionIntake
import org.riverdell.robotics.autonomous.movement.PositionChangeTolerance
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigatePurePursuit
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.ActionWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

abstract class FiveSampleAutonomous(
    sampleType: SampleType
) : HypnoticAuto(sampleType, { opMode ->
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline

    val startPose = Pose2d(0.0, 0.0, 0.degrees)
    val depositHighBucket = Pose(19.5, -5.3, (43.58).degrees)

    val pickupTolerance = PositionChangeTolerance(1.0, 5.0, 0.9 * Math.PI / 180, 5.0, 100.0)
    val initialBasketTolerance = PositionChangeTolerance(2.5, 15.0, 2.5 * Math.PI / 180, 10.0, 200.0)
    val basketTolerance = PositionChangeTolerance(2.75, 25.0, 2.5 * Math.PI / 180, 15.0, 200.0)

    // TODO
    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 30.0),
        FieldWaypoint(Pose(-70.0, 13.0, (70.0).degrees), 30.0),
        FieldWaypoint(Pose(-70.0, -38.0, (180.0).degrees), 20.0),
        FieldWaypoint(Pose(16.9, -0.0, 180.degrees), 15.0)
    )

    val toSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 25.0),
        FieldWaypoint(Pose(8.78, -15.21, 46.degrees), 30.0),
        ActionWaypoint {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true,
                doNotUseAutoMode = false,
                submersibleOverride = (opMode.robot as HypnoticAutoRobot).activeY
            )

            visionPipeline.resume()
        },
        FieldWaypoint(Pose(25.27, -10.66, 0.degrees), 25.0),
        FieldWaypoint(Pose(25.27, -10.66, 0.degrees), 20.0)
    )

    val toBasket = listOf(
        FieldWaypoint(Pose(25.27, -10.66, 0.degrees), 30.0),
        FieldWaypoint(Pose(3.97, -5.61, 55.degrees), 30.0),
        FieldWaypoint(depositHighBucket, 15.0),
    )

    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(18.26, -8.35, 78.34.degrees)),
        GroundPickupPosition(pose = Pose(21.88, -6.9, 91.76.degrees)),
        GroundPickupPosition(
            pose = Pose(17.3, -15.45, 131.71.degrees),
            extendoMode = true,
            wristState = WristState.Lateral,
            dynamicPosition = 0.6,
            extendoPosition = 250
        ),
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
            visionPipeline.pause()
            if (this["abortMission"] != null) {
                navigateTo(Pose(17.25, -8.8, 0.degrees))
                navigateTo(Pose(-82.0, -25.0, 180.0.degrees))
                return@single
            }

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
                withAutomaticDeath(if (position.extendoMode) 2500.0 else 3500.0)
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

    /*submersibleCycle()

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
    }*/
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