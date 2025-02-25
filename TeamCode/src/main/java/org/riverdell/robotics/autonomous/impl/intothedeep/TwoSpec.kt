package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.subsystems.outtake.PivotState

@Autonomous(name = "2+0 ??")
class TwoSpec : HypnoticAuto({ robot ->
    single("2+0??") {
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)
        Thread.sleep(2000)
        navigateTo(Pose(0.0, -28.4, 0.degrees)) //Specimen Scoring Line Up
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        (OuttakeLevel.bar2)
        Thread.sleep(2000)
        robot.robot.outtake.setClawState(ClawState.Open)
        navigateTo(Pose(-21.74, -13.19, 0.degrees))
        //robot.robot.outtake.setClawState(ClawState.Closed)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)
        robot.robot.outtake.setClawState(ClawState.Closed)

        /*navigateTo(Pose(-36.6, -28.8, 0.degrees))

        navigateTo(Pose(-38.0, -49.0, 0.degrees))

        navigateTo(Pose(-44.0, -51.65, 0.degrees))

        navigateTo(Pose(-45.6, -15.72, 0.degrees))

        navigateTo(Pose(-44.0, -51.65, 0.degrees))

        navigateTo(Pose(-54.37, -54.0, 0.degrees))

        navigateTo(Pose(-55.8, -17.5, 0.degrees))

        navigateTo(Pose(-56.9, -56.2, 0.degrees))

        navigateTo(Pose(-64.3, -66.1, 0.degrees)) //Diagonal Right

        navigateTo(Pose(-63.3, -25.4, 0.degrees)) //All the way back and park

        navigateTo(Pose(-63.3, -45.4, 0.degrees))

        navigateTo(Pose(-23.3, -17.0, 0.degrees))*/



        navigateTo(Pose(-30.0, 2.0, 270.degrees))
        robot.robot.outtake.setClawState(ClawState.Open)
        robot.robot.outtake.setPivotState(PivotState.PreHover)
        Thread.sleep(1000)
        navigateTo(Pose(-35.0, 2.0, 270.degrees))
        robot.robot.outtake.setPivotState(PivotState.Pickup)
        Thread.sleep(1000)
        robot.robot.outtake.setClawState(ClawState.Closed)
        Thread.sleep(1000)
        robot.robot.outtake.setPivotState(PivotState.PostScore)
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
        navigateTo(Pose(-40.0, -5.0, 0.degrees))



    }
})