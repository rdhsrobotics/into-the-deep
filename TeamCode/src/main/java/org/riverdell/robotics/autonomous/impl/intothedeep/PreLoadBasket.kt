package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.ExecutionGroup
import io.liftgate.robotics.mono.pipeline.simultaneous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Point
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState
import org.riverdell.robotics.subsystems.intake.composite.InteractionCompositeState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "4+0 Basket", group = "Test")
class PreLoadBasket : HypnoticAuto({ opMode ->
    val startPose = Pose2d(0.0, 0.0, 0.degrees)

    val depositHighBucket = Pose(16.05, 23.90, (45.0).degrees)
    // Pose (16.05

    val parkSubmersible = listOf(
        FieldWaypoint(depositHighBucket, 25.0),
        FieldWaypoint(Pose(70.29, 15.78, (70.0).degrees), 25.0),
        FieldWaypoint(
            Pose(70.0, -38.0, (180.0).degrees), 25.0
        ),
        FieldWaypoint(
            Pose(70.0, -30.0, (180.0).degrees), 10.0
        ),
    )

    opMode.robot.drivetrain.localizer.poseEstimate = startPose

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
                wideOpen = true
            )

            navigateTo(position.pose)
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

    val lastPickup = Pose(47.0, 0.5, (180).degrees)
    val pickupPositions = listOf(
        GroundPickupPosition(pose = Pose(14.5, 18.5, (90.0).degrees)),
        GroundPickupPosition(pose = Pose(14.5, 34.0, (90.0).degrees)),
        GroundPickupPosition(
            pose = lastPickup,
            extendoMode = true,
//            purePursuitPoints = listOf(
//                FieldWaypoint(depositHighBucket, 25.0),
//                FieldWaypoint(Pose(44.67, 3.30, (180.0).degrees), 25.0),
//                FieldWaypoint(Pose(55.0, 4.60, (180.0).degrees), 20.0),
//                FieldWaypoint(lastPickup, 15.0),
//            ),
            wristState = WristState.Perpendicular
        ),
    )

    // ground pickup positions
    pickupPositions.forEach {
        prepareForIntake(it)
        confirmIntakeAndTransfer(it.extendoMode)
        depositToHighBasket()
    }

    single("park near submersible") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.Bar1)

        purePursuitNavigateTo(*parkSubmersible.toTypedArray()) {
            withAutomaticDeath(9000.0)
            withCustomMaxTranslationalSpeed(0.5)
            withCustomMaxRotationalSpeed(0.5)
        }
    }
})