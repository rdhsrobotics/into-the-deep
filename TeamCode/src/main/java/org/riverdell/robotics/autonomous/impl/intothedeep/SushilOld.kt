package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "Specimen + Park")
class SushilOld : HypnoticAuto({ robot ->
    single("Specimen + Park") {
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBar2)
        (OuttakeLevel.lowBar2)

        navigateTo(Pose(0.0, -31.4, 0.degrees)) //Specimen Scoring Line Up
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.bar2)
        (OuttakeLevel.bar2)

        navigateTo(Pose(-21.74, -19.19, 0.degrees)) //Waypoint a little back to avoid hitting bar.
        //robot.robot.outtake.setClawState(ClawState.Closed)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)

        navigateTo(Pose(-36.6, -28.8, 0.degrees))

        navigateTo(Pose(-38.0, -49.0, 0.degrees))

        navigateTo(Pose(-44.0, -51.65, 0.degrees))

        navigateTo(Pose(-45.6, -15.72, 0.degrees))

        navigateTo(Pose(-44.0, -51.65, 0.degrees))

        navigateTo(Pose(-54.37, -51.0, 0.degrees))

        navigateTo(Pose(-55.8, -17.5, 0.degrees))

        navigateTo(Pose(-56.9, -56.2, 0.degrees))

        navigateTo(Pose(-64.3, -66.1, 0.degrees)) //Diagonal Right

        navigateTo(Pose(-63.3, -22.4, 0.degrees)) //All the way back and park
    }
})