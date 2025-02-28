package org.riverdell.robotics.autonomous.impl.intothedeep

import com.acmerobotics.roadrunner.geometry.Pose2d
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

@Autonomous(name = "Super Speed 2+0 ")
class TestTwoSpec : HypnoticAuto({ robot ->
    single("Ultra fast 2+0") {
        // Define waypoints for Pure Pursuit navigation
        val startPose = Pose2d(0.0, 0.0, 0.degrees)


        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)
        Thread.sleep(1000)
        navigateTo(Pose(0.0, -28.4, 0.degrees)) //Specimen Scoring Line Up
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        (OuttakeLevel.bar2)
        Thread.sleep(2000)
        robot.robot.outtake.setClawState(ClawState.Open)
        navigateTo(Pose(-21.74, -13.19, 0.degrees))
        //robot.robot.outtake.setClawState(ClawState.Closed)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)
        robot.robot.outtake.setClawState(ClawState.Closed)

        Thread.sleep(1000)
        FieldWaypoint(Pose(-21.74, -19.19, 0.degrees), 10.0) // Waypoint a little back to avoid hitting bar
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)

        FieldWaypoint(Pose(-36.6, -28.8, 0.degrees), 10.0)//main
        FieldWaypoint(Pose(-38.0, -49.0, 0.degrees), 10.0)//up to sample
        FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 5.0)//diagonal right
        FieldWaypoint(Pose(-45.6, -10.0, 0.degrees), 7.0)//PUSH BAck

        FieldWaypoint(Pose(-44.0, -51.65, 0.degrees), 10.0)//back up
        FieldWaypoint(Pose(-54.37, -57.0, 0.degrees), 10.0)//diagonal right
        FieldWaypoint(Pose(-55.8, -10.0, 0.degrees), 10.0)//PUSH BACK

        FieldWaypoint(Pose(-56.9, -56.2, 0.degrees), 5.0)//up to sample
        FieldWaypoint(Pose(-60.3, -66.1, 0.degrees), 15.0) // Diagonal Right
        FieldWaypoint(Pose(-63.3, -11.4, 0.degrees), 5.0) // All the way back and park

        navigateTo(Pose(-25.0, 2.0, 270.degrees))
        robot.robot.outtake.setClawState(ClawState.Open)
        robot.robot.outtake.setPivotState(PivotState.Hover)
        Thread.sleep(1000)
        navigateTo(Pose(-35.0, 2.0, 270.degrees))
        robot.robot.outtake.setPivotState(PivotState.PostHover)
        Thread.sleep(1000)
        robot.robot.outtake.setClawState(ClawState.Closed)
        Thread.sleep(1000)
        robot.robot.outtake.setPivotState(PivotState.Scoring)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)
        Thread.sleep(1000)
        navigateTo(Pose(-4.5, -21.0, 0.degrees))
        navigateTo(Pose(-4.5, -28.5, 0.degrees))
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        (OuttakeLevel.bar2)
        Thread.sleep(1000)
        robot.robot.outtake.setClawState(ClawState.Open)
        navigateTo(Pose(-4.5, -13.19, 0.degrees))
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)
        (OuttakeLevel.rest)


        navigateTo(Pose(-40.0, -8.0, 0.degrees)) //park pose
        robot.robot.outtake.setPivotState(PivotState.PostScore)


    }
})