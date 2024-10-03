package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateTo
import org.riverdell.robotics.autonomous.movement.purePursuitNavigateTo
import org.riverdell.robotics.autonomous.movement.purepursuit.ActionWaypoint
import org.riverdell.robotics.autonomous.movement.purepursuit.PositionWaypoint

@Autonomous(name = "Test | Move Backwards", group = "Test")
class TestMoveBackwards : HypnoticAuto({ _ ->
    single("go to position") {
        navigateTo(Pose(-5.0, -5.0, 0.degrees))
    }/*
    single("go to position") {
        navigateTo(Pose(-7.0, 15.0, 50.degrees))
    }

    single("drop pixel") {
    }

    single("go to position") {

        purePursuitNavigateTo(
            PositionWaypoint(Pose(-10.0, 0.0, 90.degrees), 10.0),
            ActionWaypoint {
                //
            },
            PositionWaypoint(Pose(-10.0, 10.0, 90.degrees), 10.0),

        )
    }*/
})