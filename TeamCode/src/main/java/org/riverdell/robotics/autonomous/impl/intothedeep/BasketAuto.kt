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

@Autonomous(name = "Basket 0+1")
class BasketAuto : HypnoticAuto({ robot ->
    single("Basket") {
        robot.robot.outtake.setClawState(ClawState.Closed)
        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.lowBasket)
        (OuttakeLevel.lowBasket)

        navigateTo(Pose(-1.10, -16.4, 45.degrees)) //Specimen Scoring Line Up
        robot.robot.outtake.setClawState(ClawState.Open)

        navigateTo(Pose(-0.0, 0.0, 0.degrees)) //Specimen Scoring Line Up

        robot.robot.intakeComposite.outtakeLevel(OuttakeLevel.rest)
        (OuttakeLevel.rest)


        robot.robot.outtake.setPivotState(PivotState.PostScore)



    }
})