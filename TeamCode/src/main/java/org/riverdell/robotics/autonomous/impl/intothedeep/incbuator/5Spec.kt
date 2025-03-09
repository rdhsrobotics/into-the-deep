package org.riverdell.robotics.autonomous.impl.intothedeep.incbuator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.intake.WristState

@Disabled
@Autonomous(name = "5 Specimen", group = "Test")
class `5Spec` : HypnoticAuto({ opMode ->
    fun depositPoseOffsetBy(offset: Double) = Pose(-4.61 - offset, 40.92, 0.000)
    val waypoints = arrayOf(
        FieldWaypoint(Pose(41.80, 38.0, 0.0), 15.0),

        FieldWaypoint(Pose(43.50, 75.0, 0.0), 20.0),
        FieldWaypoint(Pose(43.50, 77.0, 0.0), 5.0),
        FieldWaypoint(Pose(58.50, 77.0, 0.0), 5.0),
        FieldWaypoint(Pose(55.50, 16.0, 0.0), 15.0),

        FieldWaypoint(Pose(58.50, 75.0, 0.0), 20.0),
        FieldWaypoint(Pose(58.50, 77.0, 0.0), 5.0),
        FieldWaypoint(Pose(70.0, 77.0, 0.0), 5.0),
        FieldWaypoint(Pose(68.87, 16.0, 0.0), 15.0),

        FieldWaypoint(Pose(70.0, 75.0, 0.0), 20.0),
        FieldWaypoint(Pose(76.0, 77.0, 0.0), 5.0),
        FieldWaypoint(Pose(83.0, 77.0, 0.0), 3.0),

        FieldWaypoint(Pose(80.58, 16.0, 0.0), 15.0)
    )

    opMode.robot.intakeComposite.inToOut()

    var depositXOffset = 0.0
    fun depositSpecimen()
    {
        opMode.robot.intakeComposite
            .specimenCompleteAndRest()
            .join()
    }

    single("initial deposit") {
        navigateTo(depositPoseOffsetBy(0.0))
        depositSpecimen()
    }

    /*single("collect samples") {
        purePursuitNavigateTo(*waypoints) {
            disableAutomaticDeath()
        }
    }*/

    fun cycleSpecimen()
    {
        single("prepare for intake") {
            navigateTo(Pose(13.17, 13.39, (88.18).degrees))
            opMode.robot.intakeComposite
                .prepareForPickup(WristState.Lateral, wideOpen = true)
                .join()

            Thread.sleep(1000L)
        }

        single("intake and start transfer") {
            opMode.robot.intakeComposite.intakeAndConfirm().join()
            opMode.robot.intakeComposite
                .confirmAndTransferAndReady()
        }

        single("relocate") {
            navigateTo(depositPoseOffsetBy(depositXOffset++ * 5.0))
        }

        single("deposit") {
            depositSpecimen()
        }
    }

    (0..4).forEach { _ ->
        cycleSpecimen()
    }
})