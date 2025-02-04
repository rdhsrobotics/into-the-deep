package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.outtake.PivotState

@Autonomous(name = " A 2+0 and Park")
class TestTwoSpec : HypnoticAuto({ robot ->
    single(" A 2+0 and Park") {
        // Initial setup
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)

        // Define waypoints
        val waypoints = listOf(
            FieldWaypoint(Pose(0.0, -31.4, 0.degrees), 25.0), // Specimen Scoring Line Up
            FieldWaypoint(Pose(-21.74, -19.19, 0.degrees), 25.0), // Waypoint to avoid hitting bar
            FieldWaypoint(Pose(-36.6, -28.8, 0.degrees), 25.0),
            FieldWaypoint(Pose(-38.0, -49.0, 0.degrees), 25.0),
            FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 25.0),
            FieldWaypoint(Pose(-45.6, -15.72, 0.degrees), 25.0),
            FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 25.0),
            FieldWaypoint(Pose(-54.37, -54.0, 0.degrees), 25.0),
            FieldWaypoint(Pose(-55.8, -17.5, 0.degrees), 25.0),
            FieldWaypoint(Pose(-56.9, -56.2, 0.degrees), 25.0),
            FieldWaypoint(Pose(-64.3, -66.1, 0.degrees), 25.0), // Diagonal Right
            FieldWaypoint(Pose(-63.3, -25.4, 0.degrees), 25.0), // All the way back and park
            FieldWaypoint(Pose(-63.3, -45.4, 0.degrees), 25.0),
            FieldWaypoint(Pose(-23.3, -17.0, 0.degrees), 25.0)
        )

        // Navigate through waypoints
        purePursuitNavigateTo(*waypoints.toTypedArray()) {
            withAutomaticDeath(9000.0) //  for safety something something
            withCustomMaxTranslationalSpeed(0.5) // Adjust speed for smooth movement
            withCustomMaxRotationalSpeed(0.5) // Adjust rotational speed for smooth turns
            disableAutomaticDeath() // Disable automatic timeout if needed
        }

        // Outtake actions
        robot.robot.outtake.setClawState(ClawState.Open)
        robot.robot.outtake.setPivotState(PivotState.Hover)

        // Additional movements
        purePursuitNavigateTo(
            FieldWaypoint(Pose(-33.3, -10.33, 270.degrees), 25.0)
        ) {
            withAutomaticDeath(5000.0)
            withCustomMaxTranslationalSpeed(0.4)
            withCustomMaxRotationalSpeed(0.4)
        }

        robot.robot.outtake.setClawState(ClawState.Closed)
        robot.robot.outtake.setPivotState(PivotState.PostScore)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)

        purePursuitNavigateTo(
            FieldWaypoint(Pose(0.0, -38.4, 0.degrees), 25.0)
        ) {
            withAutomaticDeath(5000.0)
            withCustomMaxTranslationalSpeed(0.4)
            withCustomMaxRotationalSpeed(0.4)
        }

        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        (OuttakeLevel.bar2)
    }
})