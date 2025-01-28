package org.riverdell.robotics.autonomous.impl.intothedeep

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.subsystems.outtake.ClawState
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel
import org.riverdell.robotics.autonomous.movement.purepursuit.FieldWaypoint

@Autonomous(name = "hors")
class Sushil : HypnoticAuto({ robot ->
    single("hors") {

        purePursuitNavigateTo(
            FieldWaypoint(Pose(0.0, -28.4, 0.degrees), 10.0),
            FieldWaypoint(Pose(-21.49, -10.27,270.degrees),10.0),
            FieldWaypoint(Pose(-36.6, -10.9, 180.degrees), 10.0),
            FieldWaypoint(Pose(-37.24, -33.0, 180.degrees), 10.0),
            FieldWaypoint(Pose(-45.6, -39.8, 180.degrees), 10.0),
            FieldWaypoint(Pose(-46.00, -3.23, 180.degrees), 10.0),
            FieldWaypoint(Pose(-45.6, -37.43, 180.degrees), 10.0),
            FieldWaypoint(Pose(-54.9, -37.04, 180.degrees), 10.0),
            FieldWaypoint(Pose(-54.33, -2.24, 180.degrees), 10.0),
            FieldWaypoint(Pose(-56.5, -38.26, 180.degrees), 10.0),
            FieldWaypoint(Pose(-61.81, -34.63, 180.degrees), 10.0),
            FieldWaypoint(Pose(-53.00, -4.14, 180.degrees), 10.0), //change
            FieldWaypoint(Pose(-31.92, 6.33, 180.degrees), 10.0)
        ) {
            disableAutomaticDeath()
        }
//        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.Bar2)
//        (OuttakeLevel.Bar2)
//        robot.robot.outtake.setClawState(ClawState.Open)
//        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.Rest)
    }
})
