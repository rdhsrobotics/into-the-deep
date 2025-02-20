package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.outtake.PivotState

@Autonomous(name = "Speed 2+0 ")
class TestTwoSpec : HypnoticAuto({ robot ->
    single("Ultra fast 2+0") {
        // Define waypoints for Pure Pursuit navigation
        val waypoints = arrayOf(
            FieldWaypoint(Pose(0.0, -28.4, 0.degrees), 5.0), // Specimen Scoring Line Up
            FieldWaypoint(Pose(-21.74, -19.19, 0.degrees), 5.0), // Waypoint a little back to avoid hitting bar
            FieldWaypoint(Pose(-36.6, -28.8, 0.degrees), 5.0),
            FieldWaypoint(Pose(-38.0, -49.0, 0.degrees), 5.0),
            FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 5.0),
            FieldWaypoint(Pose(-45.6, -15.72, 0.degrees), 5.0),
            FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 5.0),
            FieldWaypoint(Pose(-54.37, -54.0, 0.degrees), 5.0),
            FieldWaypoint(Pose(-55.8, -17.5, 0.degrees), 5.0),
            FieldWaypoint(Pose(-56.9, -56.2, 0.degrees), 5.0),
            FieldWaypoint(Pose(-64.3, -66.1, 0.degrees), 5.0), // Diagonal Right
            FieldWaypoint(Pose(-63.3, -25.4, 0.degrees), 5.0), // All the way back and park
            FieldWaypoint(Pose(-63.3, -45.4, 0.degrees), 5.0),
            FieldWaypoint(Pose(-23.3, -17.0, 0.degrees), 5.0),
            FieldWaypoint(Pose(-33.3, -10.33, 270.degrees), 5.0),
            FieldWaypoint(Pose(0.0, -36.4, 0.degrees), 5.0) // Return to starting position
        )

        // Initial setup: Move outtake to low bar position
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)

        // Perform Pure Pursuit navigation through all waypoints
        purePursuitNavigateTo(*waypoints) {
            disableAutomaticDeath() // Optional: Disable automatic stopping if needed
        }

        // Perform actions at specific waypoints
        single("Set Outtake Level to Bar2") {
            navigateTo(Pose(0.0, -28.4, 0.degrees)) // Specimen Scoring Line Up
            robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        }

        single("Move Back to Avoid Bar") {
            navigateTo(Pose(-21.74, -19.19, 0.degrees)) // Waypoint a little back to avoid hitting bar
            robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)
        }

        single("Open Claw at Waypoint") {
            navigateTo(Pose(-44.0, -51.65, 0.degrees))
            robot.robot.outtake.setClawState(ClawState.Open)
        }

        single("Raise Elevator") {
            navigateTo(Pose(-54.37, -54.0, 0.degrees))
            robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.highBar)
        }

        single("Final Adjustments") {
            navigateTo(Pose(-33.3, -10.33, 270.degrees))
            robot.robot.outtake.setClawState(ClawState.Open)
            robot.robot.outtake.setPivotState(PivotState.Hover)
        }

        single("Close Claw and Post-Score") {
            navigateTo(Pose(0.0, -36.4, 0.degrees))
            robot.robot.outtake.setClawState(ClawState.Closed)
            robot.robot.outtake.setPivotState(PivotState.PostScore)
        }

        // Final outtake adjustment
        single("Final Outtake Adjustment") {
            robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        }
    }
})