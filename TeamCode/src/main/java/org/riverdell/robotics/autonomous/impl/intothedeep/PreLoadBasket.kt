package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.simultaneous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.guidedvectorfield.Vector2D
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.ActionWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel


@Autonomous(name = "4+0 Basket", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val visionPipeline = (opMode.robot as HypnoticAutoRobot).visionPipeline

    val startPose = Pose2d(0.0, 0.0, 0.degrees)

    val depositHighBucket = Pose(-14.0, 23.5, (45.0).degrees)

    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 25.0),
        FieldWaypoint(Pose(-70.0, 13.0, (70.0).degrees), 25.0),
        FieldWaypoint(Pose(-70.0, -38.0, (180.0).degrees), 25.0),
        FieldWaypoint(Pose(-70.0, -35.0, (180.0).degrees), 10.0)
    )

    val toSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 25.0),
        FieldWaypoint(Pose(-80.0, 20.0, 0.0), 20.0),
        ActionWaypoint {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true,
                doNotUseAutoMode = false,
                submersibleOverride = (opMode.robot as HypnoticAutoRobot).activeY
            )

            visionPipeline.portal.setProcessorEnabled(visionPipeline.sampleDetection, true)
            visionPipeline.portal.resumeStreaming()
        },
        FieldWaypoint(
            Pose((opMode.robot as HypnoticAutoRobot).activeX.toDouble(), -8.0, 0.0),
            25.0
        ),
        FieldWaypoint(
            Pose((opMode.robot as HypnoticAutoRobot).activeX.toDouble(), -18.0, 0.0),
            10.0
        )
    )

    val toBasket = listOf(
        FieldWaypoint(Pose(-52.1, 20.0, 10.0), 25.0),
        FieldWaypoint(depositHighBucket.add(Pose(-5.0, -5.0, 0.0)), 25.0),
        FieldWaypoint(depositHighBucket, 5.0),
    )

    opMode.robot.drivetrain.localizer.poseEstimate = startPose

    fun ExecutionGroup.depositToHighBasket(initial: Boolean = false) {
        single("high basket deposit") {
            if (initial) {
                opMode.robot.intakeComposite
                    .initialOuttakeFromRest(OuttakeLevel.SomethingLikeThat)
            } else {
                opMode.robot.intakeComposite
                    .confirmAndTransferAndReady()
                    .thenComposeAsync {
                        opMode.robot.intakeComposite
                            .initialOuttake(OuttakeLevel.SomethingLikeThat)
                    }
            }

            navigateTo(depositHighBucket)
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
            opMode.robot.intakeComposite.prepareForPickup(
                position.wristState,
                // needs to go closer into the wall
                doNotUseAutoMode = position.extendoMode,
                wideOpen = true,
            )

            navigateTo(position.pose) {
                withExtendoOut()
                disableAutomaticDeath()
            }
            /*if (position.purePursuitPoints != null) {
                *//* purePursuitNavigateTo(*position.purePursuitPoints.toTypedArray()) {
                     withAutomaticDeath(5000.0)
                 }*//*
                navigateTo(position.pose) {*//*
                    withCustomMaxRotationalSpeed(0.4)
                    withCustomMaxTranslationalSpeed(0.4)*//*
                    disableAutomaticDeath()
                }
            } else {
                navigateTo(position.pose)
            }*/
        }
    }

    fun ExecutionGroup.confirmIntakeAndTransfer(extendoMode: Boolean = false) {
        single("confirm intake") {
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

    val lastPickup = Pose(-45.7, 4.0, (180).degrees)
    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(-11.5, 18.0, (90.0).degrees)),
        GroundPickupPosition(pose = Pose(-11.5, 34.5, (90.0).degrees)),
        GroundPickupPosition(
            pose = lastPickup,
            extendoMode = true,
            wristState = WristState.Perpendicular
        ),
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer(it.extendoMode)
        depositToHighBasket()
    }

    fun submersibleIntake() {
        single("submerse itself") {
            purePursuitNavigateTo(*toSubmersible.toTypedArray()) {
                withAutomaticDeath(10000.0)
                withCustomMaxTranslationalSpeed(1.0)
                withCustomMaxRotationalSpeed(1.0)
            }
        }

        single("Search for sample") {
            opMode.robot.intakeComposite.prepareForPickup(
                WristState.Lateral,
                wideOpen = true
            ).join()

            Thread.sleep(400L)

            var position = 400
            while (visionPipeline.sampleDetection.guidanceVector == null) {
                position -= 100
                if (position <= 0) {
                    opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true, noPick = true)
                        .join()
                    opMode.robot.intakeComposite.confirmAndTransferAndReady().join()

                    return@single
                }

                opMode.robot.extension.extendToAndStayAt(position).join()
                Thread.sleep(300L)
            }
        }

        var rotationAngle = 0.0
        var vector = Vector2d(0.0, 0.0)
        single("Pick up sample") {
            val selection = (opMode.robot as HypnoticAutoRobot)
                .visionPipeline.sampleDetection

            val guidanceVector = selection.guidanceVector

            if (guidanceVector == null) {
                opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true, noPick = true).join()
                opMode.robot.intakeComposite.confirmAndTransferAndReady().join()

                return@single
            }

            visionPipeline.pause()

            rotationAngle = 0.5 + (selection.guidanceRotationAngle / 200.0)
            vector = guidanceVector
        }

        single("Go to sample") {
            val current = opMode.robot.drivetrain.localizer.pose
            val target = Pose(current.x + vector.x, current.y + vector.y, 0.0)
            opMode.robot.hardware.intakeWrist.position = rotationAngle
            navigateTo(target) { withExtendoOut() }
        }

        single("Intake sample") {
            opMode.robot.intakeComposite.intakeAndConfirm(slowMode = true).join()
            opMode.robot.intakeComposite.confirmAndTransferAndReady()
                .thenAcceptAsync {
                    opMode.robot.intakeComposite
                        .initialOuttake(OuttakeLevel.SomethingLikeThat)
                }
        }

        single("Return to basket") {
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

    submersibleIntake()

    single("park near submersible") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.Bar1)

        purePursuitNavigateTo(*parkSubmersible.toTypedArray()) {
            withAutomaticDeath(9000.0)
            withCustomMaxTranslationalSpeed(0.5)
            withCustomMaxRotationalSpeed(0.5)
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
})